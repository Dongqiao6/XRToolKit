"""D435 vision teleoperation bridge for the JAKA S5 dual arm.

Modules
-------
config    YAML configuration and defaults
camera    RealSense colour + depth capture
pose      MediaPipe pose landmarks (33 points)
hands     wrist-crop hand landmarks (pinch / fist gestures)
mapping   depth deprojection and camera -> robot base transform
clutch    session and per-arm clutch state machine
overlay   OpenCV HUD
node      ROS 2 node tying everything together
"""

__version__ = "0.1.0"
