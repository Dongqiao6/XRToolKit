import struct
import unittest
from pathlib import Path

from device_discovery import (
    VideoDevice,
    VideoFormat,
    VideoMode,
    parse_formats_ext,
    select_capture_source,
)
from d405_vision_sender import (
    CameraRequest,
    build_pipeline,
    encode_command,
    parse_ipv4_addresses,
    parse_camera_request,
    parse_command_body,
)


def camera_request_data(request):
    camera = request.camera.encode()
    ip = request.ip.encode()
    return (
        b"\xca\xfe\x01"
        + struct.pack(
            "<7i",
            request.width,
            request.height,
            request.fps,
            request.bitrate,
            request.enable_hevc,
            request.render_mode,
            request.port,
        )
        + bytes([len(camera)])
        + camera
        + bytes([len(ip)])
        + ip
    )


class D405VisionSenderTest(unittest.TestCase):
    def test_command_round_trip(self):
        packet = encode_command("OPEN_CAMERA", b"abc")
        body_length = struct.unpack(">I", packet[:4])[0]
        command, data = parse_command_body(packet[4:4 + body_length])
        self.assertEqual(command, "OPEN_CAMERA")
        self.assertEqual(data, b"abc")

    def test_camera_request_round_trip(self):
        request = CameraRequest(
            2560, 720, 30, 4000000, 0, 2, 12345,
            "REALSENSE_D405", "192.168.1.20",
        )
        self.assertEqual(parse_camera_request(camera_request_data(request)), request)

    def test_pipeline_duplicates_mono_view(self):
        pipeline = build_pipeline(
            device="/dev/video0",
            test_source=False,
            capture_format="YUY2",
            source_width=1280,
            source_height=720,
            fps=30,
            output_width=2560,
            output_height=720,
            bitrate=4000000,
            encoder="x264enc",
        )
        self.assertIn("comp.sink_0", pipeline)
        self.assertIn("comp.sink_1", pipeline)
        self.assertIn("sink_0::xpos=0", pipeline)
        self.assertIn("sink_1::xpos=1280", pipeline)
        self.assertIn("width=2560,height=720", pipeline)
        self.assertIn("tune=zerolatency", pipeline)

    def test_test_source_uses_smpte_without_camera(self):
        pipeline = build_pipeline(
            device="auto",
            test_source=True,
            capture_format="AUTO",
            source_width=1280,
            source_height=720,
            fps=30,
            output_width=2560,
            output_height=720,
            bitrate=4000000,
            encoder="x264enc",
        )
        self.assertIn("videotestsrc", pipeline)
        self.assertIn("pattern=smpte", pipeline)
        self.assertNotIn("v4l2src", pipeline)

    def test_mjpeg_capture_uses_decoder(self):
        pipeline = build_pipeline(
            device="/dev/video0",
            test_source=False,
            capture_format="MJPG",
            source_width=1280,
            source_height=720,
            fps=30,
            output_width=2560,
            output_height=720,
            bitrate=4000000,
            encoder="x264enc",
        )
        self.assertIn("image/jpeg", pipeline)
        self.assertIn("jpegdec", pipeline)

    def test_uyvy_capture_is_supported(self):
        pipeline = build_pipeline(
            device="/dev/video2",
            test_source=False,
            capture_format="UYVY",
            source_width=1280,
            source_height=720,
            fps=30,
            output_width=2560,
            output_height=720,
            bitrate=4000000,
            encoder="x264enc",
        )
        self.assertIn("format=UYVY,width=1280,height=720", pipeline)

    def test_ipv4_address_diagnostics(self):
        addresses = parse_ipv4_addresses(
            "2: eth0    inet 192.168.71.10/24 brd 192.168.71.255 scope global eth0\n"
            "3: wlan0   inet 192.168.1.20/24 brd 192.168.1.255 scope global wlan0\n"
        )
        self.assertEqual(
            addresses,
            [("eth0", "192.168.71.10"), ("wlan0", "192.168.1.20")],
        )

    def test_formats_ext_separates_depth_and_color(self):
        formats = parse_formats_ext(
            """
	[0]: 'Z16 ' (16-bit Depth)
		Size: Discrete 1280x720
			Interval: Discrete 0.033s (30.000 fps)
	[1]: 'YUYV' (YUYV 4:2:2)
		Size: Discrete 1280x720
			Interval: Discrete 0.033s (30.000 fps)
"""
        )
        self.assertEqual(formats[0].name, "Z16")
        self.assertEqual(formats[1].name, "YUYV")
        self.assertEqual(formats[1].modes[0], VideoMode(1280, 720, 30))

    def test_auto_selects_d405_color_node(self):
        color = VideoDevice(
            path="/dev/video2",
            card="Intel(R) RealSense(TM) Depth Camera 405",
            product_id="0b5b",
            serial="d405",
            formats=(
                VideoFormat(
                    "YUYV", (VideoMode(1280, 720, 30),)
                ),
            ),
        )
        other_camera = VideoDevice(
            path="/dev/video7",
            card="Another Camera",
            product_id="1234",
            formats=(
                VideoFormat(
                    "MJPG", (VideoMode(1280, 720, 30),)
                ),
            ),
        )
        selected, capture_format = select_capture_source(
            [
                VideoDevice(path="/dev/video0", product_id="0b5b", formats=(
                    VideoFormat("Z16", (VideoMode(1280, 720, 30),)),
                )),
                color,
                other_camera,
            ],
            requested_device="auto",
            width=1280,
            height=720,
            fps=30,
        )
        self.assertEqual(selected.path, "/dev/video2")
        self.assertEqual(capture_format, "YUY2")

    def test_prefers_yuy2_even_when_mjpg_is_listed_first(self):
        camera = VideoDevice(
            path="/dev/video3",
            product_id="0b5b",
            formats=(
                VideoFormat("MJPG", (VideoMode(1280, 720, 30),)),
                VideoFormat("YUYV", (VideoMode(1280, 720, 30),)),
            ),
        )
        _, capture_format = select_capture_source(
            [camera],
            requested_device="auto",
            width=1280,
            height=720,
            fps=30,
        )
        self.assertEqual(capture_format, "YUY2")

    def test_explicit_depth_node_is_rejected(self):
        depth = VideoDevice(
            path="/dev/video0",
            product_id="0b5b",
            formats=(VideoFormat("Z16", (VideoMode(1280, 720, 30),)),),
        )
        with self.assertRaisesRegex(RuntimeError, "not a color-capture node"):
            select_capture_source(
                [depth],
                requested_device="/dev/video0",
                width=1280,
                height=720,
                fps=30,
            )

    def test_d405_profile_matches_sbs_output(self):
        profile = Path(__file__).with_name("video_source_d405.yml").read_text(
            encoding="utf-8"
        )
        self.assertIn('- name: "REALSENSE_D405"', profile)
        self.assertIn('camera: "ZED"', profile)
        self.assertIn('name: "CamWidth"\n      type: "int"\n      value: 2560', profile)
        self.assertIn('name: "CamHeight"\n      type: "int"\n      value: 720', profile)


if __name__ == "__main__":
    unittest.main()
