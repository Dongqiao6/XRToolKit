"""Discover the Intel RealSense D405 color V4L2 node."""

from __future__ import annotations

import dataclasses
import re
import subprocess
from pathlib import Path


D405_PRODUCT_ID = "0b5b"
_FORMAT_RE = re.compile(r"^\s*\[\d+\]:\s*'([^']+)'")
_SIZE_RE = re.compile(r"^\s*Size: Discrete\s+(\d+)x(\d+)")
_FPS_RE = re.compile(r"^\s*Interval: Discrete\s+\S+\s+\(([\d.]+)\s+fps\)")
_VIDEO_RE = re.compile(r"^video(\d+)$")
_COLOR_FORMATS = {
    "YUYV": "YUY2",
    "YUY2": "YUY2",
    "UYVY": "UYVY",
    "MJPG": "MJPG",
    "MJPEG": "MJPG",
}


@dataclasses.dataclass(frozen=True)
class VideoMode:
    width: int
    height: int
    fps: int


@dataclasses.dataclass(frozen=True)
class VideoFormat:
    name: str
    modes: tuple[VideoMode, ...] = ()


@dataclasses.dataclass(frozen=True)
class VideoDevice:
    path: str
    card: str = ""
    bus_info: str = ""
    vendor_id: str = ""
    product_id: str = ""
    serial: str = ""
    formats: tuple[VideoFormat, ...] = ()

    @property
    def color_formats(self) -> tuple[str, ...]:
        formats = []
        for video_format in self.formats:
            capture_format = _COLOR_FORMATS.get(video_format.name.upper())
            if capture_format is not None and capture_format not in formats:
                formats.append(capture_format)
        return tuple(formats)

    @property
    def color_format(self) -> str | None:
        formats = self.color_formats
        for preferred in ("YUY2", "UYVY", "MJPG"):
            if preferred in formats:
                return preferred
        return None

    @property
    def format_names(self) -> str:
        return ", ".join(video_format.name for video_format in self.formats) or "none"

    def supports(self, width: int, height: int, fps: int) -> bool:
        for video_format in self.formats:
            if video_format.name.upper() not in _COLOR_FORMATS:
                continue
            if any(
                mode.width == width
                and mode.height == height
                and abs(mode.fps - fps) <= 1
                for mode in video_format.modes
            ):
                return True
        return False

    def color_modes(self) -> str:
        modes = []
        for video_format in self.formats:
            if video_format.name.upper() not in _COLOR_FORMATS:
                continue
            modes.extend(
                "%dx%d@%d" % (mode.width, mode.height, mode.fps)
                for mode in video_format.modes
            )
        return ", ".join(sorted(set(modes))) or "none"

    @property
    def is_d405(self) -> bool:
        identity = " ".join((self.card, self.bus_info)).upper()
        return self.product_id.lower() == D405_PRODUCT_ID or "D405" in identity


def parse_video_info(output: str) -> dict[str, str]:
    values = {}
    for line in output.splitlines():
        key, separator, value = line.partition(":")
        if separator:
            values[key.strip()] = value.strip()
    return values


def parse_formats_ext(output: str) -> tuple[VideoFormat, ...]:
    formats = []
    name = None
    modes = []
    current_size = None

    for line in output.splitlines():
        format_match = _FORMAT_RE.match(line)
        if format_match:
            if name is not None:
                formats.append(VideoFormat(name, tuple(modes)))
            name = format_match.group(1).strip()
            modes = []
            current_size = None
            continue
        if name is None:
            continue
        size_match = _SIZE_RE.match(line)
        if size_match:
            current_size = (int(size_match.group(1)), int(size_match.group(2)))
            continue
        fps_match = _FPS_RE.match(line)
        if fps_match and current_size is not None:
            modes.append(
                VideoMode(
                    current_size[0],
                    current_size[1],
                    int(round(float(fps_match.group(1)))),
                )
            )

    if name is not None:
        formats.append(VideoFormat(name, tuple(modes)))
    return tuple(formats)


def _read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8").strip()
    except (OSError, UnicodeError):
        return ""


def read_usb_identity(device_path: str, sysfs_root: str | Path) -> dict[str, str]:
    node = Path(sysfs_root) / Path(device_path).name / "device"
    try:
        candidate = node.resolve()
    except OSError:
        return {}

    for parent in (candidate, *candidate.parents):
        vendor_id = _read_text(parent / "idVendor")
        product_id = _read_text(parent / "idProduct")
        if vendor_id and product_id:
            return {
                "vendor_id": vendor_id,
                "product_id": product_id,
                "serial": _read_text(parent / "serial"),
            }
    return {}


def _run_v4l2_ctl(*arguments: str) -> str:
    result = subprocess.run(
        ["v4l2-ctl", *arguments],
        check=False,
        capture_output=True,
        text=True,
        timeout=5,
    )
    return result.stdout


def discover_video_devices(
    dev_root: str | Path = "/dev",
    sysfs_root: str | Path = "/sys/class/video4linux",
) -> list[VideoDevice]:
    nodes = []
    for path in Path(dev_root).glob("video*"):
        match = _VIDEO_RE.match(path.name)
        if match and path.exists():
            nodes.append((int(match.group(1)), path))

    devices = []
    for _, path in sorted(nodes):
        info = parse_video_info(
            _run_v4l2_ctl("--device", str(path), "--info")
        )
        formats = parse_formats_ext(
            _run_v4l2_ctl("--device", str(path), "--list-formats-ext")
        )
        identity = read_usb_identity(str(path), sysfs_root)
        devices.append(
            VideoDevice(
                path=str(path),
                card=info.get("Card type", ""),
                bus_info=info.get("Bus info", ""),
                vendor_id=identity.get("vendor_id", ""),
                product_id=identity.get("product_id", ""),
                serial=identity.get("serial", ""),
                formats=formats,
            )
        )
    return devices


def select_capture_source(
    devices: list[VideoDevice],
    *,
    requested_device: str,
    width: int,
    height: int,
    fps: int,
) -> tuple[VideoDevice, str]:
    requested_device = requested_device.strip()
    if requested_device.lower() != "auto":
        matches = [device for device in devices if device.path == requested_device]
        if not matches:
            raise RuntimeError(
                "V4L2 device %s was not found; run with --list-devices"
                % requested_device
            )
        device = matches[0]
    else:
        d405_devices = [device for device in devices if device.is_d405]
        if not d405_devices:
            raise RuntimeError(
                "No Intel RealSense D405 was found (USB product ID %s). "
                "Run with --list-devices and keep the other cameras untouched."
                % D405_PRODUCT_ID
            )
        color_devices = [
            device for device in d405_devices if device.color_format is not None
        ]
        if not color_devices:
            formats = "; ".join(
                "%s: %s" % (device.path, device.format_names)
                for device in d405_devices
            )
            raise RuntimeError(
                "The D405 has no V4L2 color node. Found only: %s" % formats
            )
        device = max(
            color_devices,
            key=lambda item: (
                item.supports(width, height, fps),
                item.color_format == "YUY2",
                -int(Path(item.path).name.removeprefix("video")),
            ),
        )

    capture_format = device.color_format
    if capture_format is None:
        raise RuntimeError(
            "%s is not a color-capture node (formats: %s). "
            "Use --list-devices to select the D405 color node."
            % (device.path, device.format_names)
        )
    if device.formats and not device.supports(width, height, fps):
        raise RuntimeError(
            "%s does not expose a color mode %dx%d@%d. Available color modes: %s"
            % (
                device.path,
                width,
                height,
                fps,
                device.color_modes(),
            )
        )
    return device, capture_format


def format_device_table(devices: list[VideoDevice]) -> str:
    if not devices:
        return "No V4L2 video devices found."
    lines = []
    for device in devices:
        role = "D405" if device.is_d405 else "other"
        color = device.color_format or "no color"
        lines.append(
            "%s | %s | USB %s:%s | serial=%s | %s | formats=%s"
            % (
                device.path,
                role,
                device.vendor_id or "?",
                device.product_id or "?",
                device.serial or "?",
                color,
                device.format_names,
            )
        )
    return "\n".join(lines)
