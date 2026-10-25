import struct
import unittest
from pathlib import Path

from d405_vision_sender import (
    CameraRequest,
    build_pipeline,
    encode_command,
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

    def test_d405_profile_matches_sbs_output(self):
        profile = Path(__file__).with_name("video_source_d405.yml").read_text(
            encoding="utf-8"
        )
        self.assertIn('- name: "REALSENSE_D405"', profile)
        self.assertIn('camera: "REALSENSE_D405"', profile)
        self.assertIn('name: "CamWidth"\n      type: "int"\n      value: 2560', profile)
        self.assertIn('name: "CamHeight"\n      type: "int"\n      value: 720', profile)


if __name__ == "__main__":
    unittest.main()
