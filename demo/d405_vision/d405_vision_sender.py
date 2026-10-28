#!/usr/bin/env python3
"""Low-latency D405 H.264 sender for XRoboToolkit Remote Vision."""

import argparse
import dataclasses
import logging
import os
import re
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

from device_discovery import (
    discover_video_devices,
    format_device_table,
    select_capture_source,
)

try:
    import gi

    gi.require_version("Gst", "1.0")
    from gi.repository import Gst
except (ImportError, ValueError) as exc:
    raise SystemExit(
        "GStreamer Python bindings are required (python3-gi and gir1.2-gstreamer-1.0)."
    ) from exc


LOG = logging.getLogger("d405_vision")
CONTROL_PORT = 13579
CAMERA_MAGIC = b"\xca\xfe"
FRAME_HEADER = struct.Struct(">I")
INT32_LE = struct.Struct("<i")


@dataclasses.dataclass(frozen=True)
class CameraRequest:
    width: int
    height: int
    fps: int
    bitrate: int
    enable_hevc: int
    render_mode: int
    port: int
    camera: str
    ip: str


class ControlFrameReader:
    """Read length-framed control messages without treating idle as disconnect."""

    def __init__(self, sock, read_size=4096, max_body_bytes=2 * 1024 * 1024):
        self._socket = sock
        self._read_size = read_size
        self._max_body_bytes = max_body_bytes
        self._buffer = bytearray()

    def next_body(self):
        while True:
            if len(self._buffer) >= FRAME_HEADER.size:
                body_length = FRAME_HEADER.unpack_from(self._buffer, 0)[0]
                if body_length <= 0 or body_length > self._max_body_bytes:
                    raise ValueError(
                        "invalid control body length: %d" % body_length
                    )
                frame_size = FRAME_HEADER.size + body_length
                if len(self._buffer) >= frame_size:
                    body = bytes(self._buffer[FRAME_HEADER.size:frame_size])
                    del self._buffer[:frame_size]
                    return body

            try:
                chunk = self._socket.recv(self._read_size)
            except socket.timeout:
                # Remote Vision commonly sends no command after OPEN_CAMERA.
                # Keep the session and PING heartbeat alive while waiting.
                continue
            if not chunk:
                raise ConnectionError("control connection closed")
            self._buffer.extend(chunk)


def encode_command(command, data=b""):
    command_bytes = command.encode("utf-8")
    body = (
        INT32_LE.pack(len(command_bytes))
        + command_bytes
        + INT32_LE.pack(len(data))
        + data
    )
    return FRAME_HEADER.pack(len(body)) + body


def parse_command_body(body):
    if len(body) < 8:
        raise ValueError("protocol body is too short")
    command_length = INT32_LE.unpack_from(body, 0)[0]
    if command_length < 0 or 8 + command_length > len(body):
        raise ValueError("invalid command length")
    command = body[4:4 + command_length].decode("utf-8")
    data_length_offset = 4 + command_length
    data_length = INT32_LE.unpack_from(body, data_length_offset)[0]
    data_offset = data_length_offset + 4
    if data_length < 0 or data_offset + data_length > len(body):
        raise ValueError("invalid data length")
    return command, body[data_offset:data_offset + data_length]


def annex_b_nal_types(payload):
    """Return H.264 NAL unit types from an Annex-B byte stream."""
    types = []
    offset = 0
    size = len(payload)
    while offset + 4 <= size:
        if payload[offset:offset + 4] == b"\x00\x00\x00\x01":
            header = offset + 4
        elif payload[offset:offset + 3] == b"\x00\x00\x01":
            header = offset + 3
        else:
            offset += 1
            continue
        if header < size:
            types.append(payload[header] & 0x1F)
        offset = header + 1
    return types


def parse_camera_request(data):
    if len(data) < 10 or data[:2] != CAMERA_MAGIC or data[2] != 1:
        raise ValueError("invalid camera request header")
    if len(data) < 3 + 7 * 4:
        raise ValueError("camera request is too short")

    values = struct.unpack_from("<7i", data, 3)
    offset = 3 + 7 * 4
    strings = []
    for _ in range(2):
        if offset >= len(data):
            raise ValueError("camera request string length is missing")
        length = data[offset]
        offset += 1
        if offset + length > len(data):
            raise ValueError("camera request string is truncated")
        strings.append(data[offset:offset + length].decode("utf-8"))
        offset += length

    return CameraRequest(
        width=values[0],
        height=values[1],
        fps=values[2],
        bitrate=values[3],
        enable_hevc=values[4],
        render_mode=values[5],
        port=values[6],
        camera=strings[0],
        ip=strings[1],
    )


def available_encoder(preferred="auto"):
    candidates = (
        ["nvv4l2h264enc", "v4l2h264enc", "openh264enc", "x264enc"]
        if preferred == "auto"
        else [preferred]
    )
    for name in candidates:
        if Gst.ElementFactory.find(name) is not None:
            return name
    raise RuntimeError(
        "No H.264 GStreamer encoder found. Install gstreamer1.0-plugins-ugly "
        "(x264enc) or choose an installed encoder."
    )


def parse_ipv4_addresses(output):
    """Return ``(interface, address)`` pairs from ``ip -4 -o addr`` output."""
    addresses = []
    for line in output.splitlines():
        fields = line.split()
        if "inet" not in fields:
            continue
        inet_index = fields.index("inet")
        if inet_index == 0 or inet_index + 1 >= len(fields):
            continue
        address = fields[inet_index + 1].split("/", 1)[0]
        addresses.append((fields[inet_index - 1], address))
    return addresses


def local_ipv4_addresses():
    try:
        result = subprocess.run(
            ["ip", "-4", "-o", "addr", "show", "scope", "global"],
            check=False,
            capture_output=True,
            text=True,
            timeout=3,
        )
    except (OSError, subprocess.SubprocessError):
        return []
    return parse_ipv4_addresses(result.stdout)


def source_ip_for_remote(remote_ip):
    if not remote_ip:
        return None
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.connect((remote_ip, CONTROL_PORT))
            return probe.getsockname()[0]
    except OSError:
        return None


def quest_ip_path():
    return Path(__file__).resolve().parent.parent / ".quest_ip"


def _encoder_clause(encoder, bitrate_bps):
    bitrate_kbps = max(100, int(bitrate_bps / 1000))
    if encoder == "x264enc":
        return (
            f"x264enc tune=zerolatency speed-preset=ultrafast "
            f"bitrate={bitrate_kbps} key-int-max=15 bframes=0 "
            f"rc-lookahead=0 sync-lookahead=0 byte-stream=true aud=true"
        )
    if encoder == "openh264enc":
        return (
            f"openh264enc bitrate={bitrate_bps} complexity=1 "
            f"gop-size=30 usage-type=screen"
        )
    if encoder == "v4l2h264enc":
        return (
            "v4l2h264enc extra-controls=\"controls,"
            f"video_bitrate={bitrate_bps},h264_i_frame_period=30\""
        )
    if encoder == "nvv4l2h264enc":
        return (
            "nvv4l2h264enc maxperf-enable=1 insert-sps-pps=true "
            f"idrinterval=15 bitrate={bitrate_bps}"
        )
    raise ValueError(f"unsupported encoder: {encoder}")


def build_pipeline(
    *,
    device,
    test_source,
    capture_format,
    source_width,
    source_height,
    fps,
    output_width,
    output_height,
    bitrate,
    encoder,
):
    if output_width < 2 or output_height < 1:
        raise ValueError("output dimensions must be positive")
    eye_width = output_width // 2
    if eye_width < 1:
        raise ValueError("output width must fit two eye views")

    if test_source:
        source = (
            f"videotestsrc is-live=true pattern=smpte "
            f"! video/x-raw,width={source_width},height={source_height},"
            f"framerate={fps}/1"
        )
    else:
        if not re.fullmatch(r"[/A-Za-z0-9_.-]+", device or ""):
            raise ValueError("invalid V4L2 device path")
        normalized_format = capture_format.upper()
        if normalized_format in ("YUY2", "YUYV"):
            source = (
                f"v4l2src device={device} do-timestamp=true "
                f"! video/x-raw,format=YUY2,width={source_width},"
                f"height={source_height},framerate={fps}/1"
            )
        elif normalized_format == "UYVY":
            source = (
                f"v4l2src device={device} do-timestamp=true "
                f"! video/x-raw,format=UYVY,width={source_width},"
                f"height={source_height},framerate={fps}/1"
            )
        elif normalized_format in ("MJPG", "MJPEG"):
            source = (
                f"v4l2src device={device} do-timestamp=true "
                f"! image/jpeg,width={source_width},height={source_height},"
                f"framerate={fps}/1 ! jpegdec"
            )
        elif normalized_format == "AUTO":
            source = f"v4l2src device={device} do-timestamp=true ! decodebin"
        else:
            raise ValueError(
                "capture format must be YUY2, UYVY, MJPG, or AUTO"
            )
        source += (
            f" ! videoconvert ! videoscale "
            f"! video/x-raw,width={source_width},height={source_height},"
            f"framerate={fps}/1"
        )

    if encoder == "nvv4l2h264enc":
        encoder_prefix = (
            "videoconvert ! video/x-raw,format=I420 ! nvvidconv "
            "! video/x-raw(memory:NVMM),format=NV12"
        )
    elif encoder == "v4l2h264enc":
        encoder_prefix = "videoconvert ! video/x-raw,format=I420"
    else:
        encoder_prefix = "videoconvert"

    if encoder == "x264enc":
        h264_caps = (
            "video/x-h264,stream-format=byte-stream,alignment=au,"
            "profile=baseline"
        )
    else:
        h264_caps = "video/x-h264,stream-format=byte-stream,alignment=au"

    return (
        f"{source} ! tee name=src "
        "src. ! queue max-size-buffers=1 leaky=downstream ! comp.sink_0 "
        "src. ! queue max-size-buffers=1 leaky=downstream ! comp.sink_1 "
        "compositor name=comp "
        f"sink_0::xpos=0 sink_0::ypos=0 sink_0::width={eye_width} "
        f"sink_0::height={output_height} "
        f"sink_1::xpos={eye_width} sink_1::ypos=0 "
        f"sink_1::width={eye_width} sink_1::height={output_height} "
        f"! video/x-raw,width={output_width},height={output_height},"
        f"framerate={fps}/1 ! {encoder_prefix} ! "
        f"{_encoder_clause(encoder, bitrate)} "
        f"! {h264_caps} "
        "! h264parse config-interval=-1 "
        "! appsink name=sink emit-signals=true sync=false "
        "max-buffers=1 drop=true"
    )


class StreamSession:
    def __init__(self, pipeline, target_ip, target_port):
        self._pipeline = pipeline
        self._target_ip = target_ip
        self._target_port = target_port
        self._send_socket = None
        self._appsink = None
        self._bus = None
        self._stopping = threading.Event()
        self._closed = threading.Event()
        self._monitor = None
        self._sent_frames = 0
        self._sent_bytes = 0
        self._last_log_time = 0.0

    def start(self):
        sock = socket.create_connection(
            (self._target_ip, self._target_port), timeout=5.0
        )
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(2.0)
        self._send_socket = sock

        self._bus = self._pipeline.get_bus()
        self._appsink = self._pipeline.get_by_name("sink")
        if self._appsink is None:
            raise RuntimeError("GStreamer appsink was not created")
        self._appsink.connect("new-sample", self._on_new_sample)

        result = self._pipeline.set_state(Gst.State.PLAYING)
        if result == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("GStreamer pipeline failed to start")

        self._monitor = threading.Thread(
            target=self._monitor_loop, name="d405-pipeline-monitor", daemon=True
        )
        self._monitor.start()
        LOG.info(
            "Streaming D405 to %s:%d", self._target_ip, self._target_port
        )

    def _on_new_sample(self, sink):
        if self._stopping.is_set():
            return Gst.FlowReturn.EOS
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.ERROR
        try:
            buffer = sample.get_buffer()
            success, mapped = buffer.map(Gst.MapFlags.READ)
            if not success:
                return Gst.FlowReturn.ERROR
            try:
                payload = bytes(mapped.data)
            finally:
                buffer.unmap(mapped)
            if not payload or self._send_socket is None:
                return Gst.FlowReturn.OK
            self._send_socket.sendall(FRAME_HEADER.pack(len(payload)) + payload)
            self._sent_frames += 1
            self._sent_bytes += len(payload)
            now = time.monotonic()
            if self._sent_frames == 1 or now - self._last_log_time >= 5.0:
                LOG.info(
                    "Video sent: frames=%d bytes=%d last_frame=%d bytes",
                    self._sent_frames,
                    self._sent_bytes,
                    len(payload),
                )
                self._last_log_time = now
            if self._sent_frames == 1:
                LOG.info(
                    "First H.264 access unit NAL types: %s "
                    "(SPS=7, PPS=8, IDR=5)",
                    annex_b_nal_types(payload),
                )
        except (BrokenPipeError, ConnectionError, OSError) as exc:
            LOG.warning("Video TCP connection lost: %s", exc)
            self._stopping.set()
            return Gst.FlowReturn.EOS
        return Gst.FlowReturn.OK

    def _monitor_loop(self):
        while not self._stopping.is_set():
            message = self._bus.timed_pop_filtered(
                200 * Gst.MSECOND, Gst.MessageType.ERROR | Gst.MessageType.EOS
            )
            if message is None:
                continue
            if message.type == Gst.MessageType.ERROR:
                error, debug = message.parse_error()
                LOG.error("GStreamer error: %s (%s)", error.message, debug)
            break
        self.stop()

    def stop(self):
        if self._closed.is_set():
            return
        self._closed.set()
        self._stopping.set()
        try:
            self._pipeline.set_state(Gst.State.NULL)
        except Exception:
            pass
        if self._send_socket is not None:
            try:
                self._send_socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._send_socket.close()
            self._send_socket = None
        if (
            self._monitor is not None
            and self._monitor.is_alive()
            and threading.current_thread() is not self._monitor
        ):
            self._monitor.join(timeout=1.0)
        LOG.info(
            "D405 stream stopped (frames=%d, bytes=%d)",
            self._sent_frames,
            self._sent_bytes,
        )


class SenderServer:
    def __init__(self, args):
        self.args = args
        self.session = None

    def _start_stream(self, request):
        if request.camera not in ("REALSENSE_D405", "ZED"):
            LOG.warning(
                "Unexpected camera type %s; accepting it as a generic H.264 client",
                request.camera,
            )
        if not request.ip or request.port <= 0:
            raise ValueError("camera request has no valid headset TCP target")
        LOG.info(
            "OPEN_CAMERA request: camera=%s target=%s:%d video=%dx%d fps=%d "
            "bitrate=%d hevc=%d render_mode=%d",
            request.camera,
            request.ip,
            request.port,
            request.width,
            request.height,
            request.fps,
            request.bitrate,
            request.enable_hevc,
            request.render_mode,
        )
        self.stop_stream()

        encoder = available_encoder(self.args.encoder)
        device = self.args.device
        capture_format = self.args.capture_format
        if not self.args.test_source:
            selected, capture_format = resolve_capture_source(
                self.args, quiet=False
            )
            device = selected.path
        output_width = request.width if request.width >= 2 else 2560
        output_height = request.height if request.height > 0 else 720
        fps = self.args.fps
        description = build_pipeline(
            device=device,
            test_source=self.args.test_source,
            capture_format=capture_format,
            source_width=self.args.source_width,
            source_height=self.args.source_height,
            fps=fps,
            output_width=output_width,
            output_height=output_height,
            bitrate=self.args.bitrate or request.bitrate,
            encoder=encoder,
        )
        LOG.info("Pipeline: %s", description)
        try:
            pipeline = Gst.parse_launch(description)
        except Exception as exc:
            raise RuntimeError(f"cannot create GStreamer pipeline: {exc}") from exc
        if pipeline is None:
            raise RuntimeError("cannot create GStreamer pipeline")
        self.session = StreamSession(pipeline, request.ip, request.port)
        self.session.start()

    def stop_stream(self):
        if self.session is not None:
            self.session.stop()
            self.session = None

    def preflight(self):
        if not self.args.test_source:
            try:
                resolve_capture_source(self.args, quiet=False)
            except Exception as exc:
                raise RuntimeError("D405 camera preflight failed: %s" % exc) from exc
        log_network_hint()

    def _handle_client(self, client):
        client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        client.settimeout(1.0)
        LOG.info("Remote Vision control connected from %s", client.getpeername())
        heartbeat_stop = threading.Event()
        send_lock = threading.Lock()
        frame_reader = ControlFrameReader(client)

        def heartbeat_loop():
            while not heartbeat_stop.wait(1.0):
                try:
                    with send_lock:
                        client.sendall(encode_command("PING", b"d405"))
                except OSError:
                    return

        heartbeat = threading.Thread(
            target=heartbeat_loop, name="remote-vision-heartbeat", daemon=True
        )
        heartbeat.start()
        try:
            with send_lock:
                client.sendall(encode_command("PING", b"d405"))
            while True:
                body = frame_reader.next_body()
                command, data = parse_command_body(body)
                LOG.info("Control command: %s (%d bytes)", command, len(data))
                if command == "OPEN_CAMERA":
                    self._start_stream(parse_camera_request(data))
                elif command == "CLOSE_CAMERA":
                    self.stop_stream()
                elif command == "PING":
                    with send_lock:
                        client.sendall(encode_command("PONG", data))
                elif command == "PONG":
                    pass
        except (ConnectionError, OSError, ValueError, RuntimeError) as exc:
            if not isinstance(exc, ConnectionError):
                LOG.warning("Control session ended: %s", exc)
        finally:
            heartbeat_stop.set()
            heartbeat.join(timeout=1.0)
            self.stop_stream()
            client.close()
            LOG.info("Remote Vision control disconnected")

    def serve_forever(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((self.args.bind_host, self.args.bind_port))
            server.listen(1)
            LOG.info(
                "Waiting for Quest Remote Vision on %s:%d",
                self.args.bind_host,
                self.args.bind_port,
            )
            while True:
                client, _ = server.accept()
                self._handle_client(client)


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description="Low-latency Intel RealSense D405 sender for XRoboToolkit Remote Vision"
    )
    parser.add_argument("--bind", default=f"0.0.0.0:{CONTROL_PORT}")
    parser.add_argument(
        "--device",
        default="auto",
        help="D405 V4L2 color node, or auto (default)",
    )
    parser.add_argument("--source-width", type=int, default=1280)
    parser.add_argument("--source-height", type=int, default=720)
    parser.add_argument(
        "--capture-format",
        default="AUTO",
        choices=("YUY2", "YUYV", "UYVY", "MJPG", "MJPEG", "AUTO", "auto"),
        help="V4L2 capture format; AUTO selects from the D405 color node",
    )
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--bitrate", type=int, default=4000000)
    parser.add_argument(
        "--encoder",
        default="auto",
        help="auto, x264enc, openh264enc, v4l2h264enc, or nvv4l2h264enc",
    )
    parser.add_argument("--test-source", action="store_true")
    parser.add_argument(
        "--list-devices",
        action="store_true",
        help="List V4L2 nodes and identify the D405 color node",
    )
    parser.add_argument("--check", action="store_true")
    return parser


def resolve_capture_source(args, quiet=False):
    devices = discover_video_devices()
    device, capture_format = select_capture_source(
        devices,
        requested_device=args.device,
        width=args.source_width,
        height=args.source_height,
        fps=args.fps,
    )
    if args.capture_format.upper() not in ("AUTO",):
        requested_format = args.capture_format.upper()
        if requested_format in ("YUYV",):
            requested_format = "YUY2"
        if requested_format in ("MJPEG",):
            requested_format = "MJPG"
        capture_format = requested_format
    if not quiet:
        LOG.info(
            "Selected %s (%s, USB %s:%s, serial=%s), capture=%s",
            device.path,
            device.card,
            device.vendor_id or "?",
            device.product_id or "?",
            device.serial or "?",
            capture_format,
        )
    return device, capture_format


def log_network_hint():
    addresses = local_ipv4_addresses()
    if not addresses:
        LOG.warning(
            "Cannot list local IPv4 addresses; run 'hostname -I' and enter the "
            "JAKA host address reachable from Quest."
        )
        return

    quest_ip = ""
    try:
        quest_ip = quest_ip_path().read_text(encoding="utf-8").strip()
    except OSError:
        pass

    recommended = source_ip_for_remote(quest_ip)
    if recommended is None and len(addresses) == 1:
        recommended = addresses[0][1]
    if recommended is not None:
        LOG.info(
            "In Quest Remote Vision, enter JAKA host IP %s "
            "(not the JAKA controller 192.168.71.x and not the Quest IP)",
            recommended,
        )
    else:
        LOG.warning(
            "Multiple host IPv4 addresses found; enter the one on Quest's LAN "
            "in Remote Vision."
        )
    LOG.info(
        "Host IPv4 addresses: %s",
        ", ".join("%s=%s" % item for item in addresses),
    )
    if quest_ip:
        LOG.info("Last detected Quest IP: %s", quest_ip)


def validate_pipeline(description):
    pipeline = Gst.parse_launch(description)
    if pipeline is None:
        raise RuntimeError("cannot create GStreamer pipeline")
    try:
        pipeline.set_state(Gst.State.PAUSED)
        return_value, state, _ = pipeline.get_state(5 * Gst.SECOND)
        bus = pipeline.get_bus()
        message = bus.pop_filtered(Gst.MessageType.ERROR)
        if message is not None:
            error, debug = message.parse_error()
            raise RuntimeError("%s (%s)" % (error.message, debug))
        if return_value == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("GStreamer could not reach PAUSED state")
    finally:
        pipeline.set_state(Gst.State.NULL)


def main(argv=None):
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s"
    )
    Gst.init(None)
    host, separator, port_text = args.bind.rpartition(":")
    if not separator or not host:
        parser.error("--bind must be HOST:PORT")
    args.bind_host = host
    try:
        args.bind_port = int(port_text)
    except ValueError:
        parser.error("--bind port must be an integer")

    if args.list_devices:
        try:
            devices = discover_video_devices()
            print(format_device_table(devices))
            if not args.test_source:
                selected, capture_format = resolve_capture_source(args, quiet=True)
                print(
                    "Selected D405 color source: %s (%s)"
                    % (selected.path, capture_format)
                )
        except Exception as exc:
            print(f"device discovery failed: {exc}", file=sys.stderr)
            return 1
        return 0

    if args.check:
        try:
            encoder = available_encoder(args.encoder)
            device = args.device
            capture_format = args.capture_format
            if not args.test_source:
                selected, capture_format = resolve_capture_source(args, quiet=False)
                device = selected.path
            description = build_pipeline(
                device=device,
                test_source=args.test_source,
                capture_format=capture_format,
                source_width=args.source_width,
                source_height=args.source_height,
                fps=args.fps,
                output_width=2560,
                output_height=720,
                bitrate=args.bitrate,
                encoder=encoder,
            )
            validate_pipeline(description)
        except Exception as exc:
            print(f"check failed: {exc}", file=sys.stderr)
            return 1
        print(f"encoder: {encoder}")
        print(description)
        return 0

    server = SenderServer(args)
    try:
        server.preflight()
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        print(exc, file=sys.stderr)
        return 1
    finally:
        server.stop_stream()
    return 0


if __name__ == "__main__":
    sys.exit(main())
