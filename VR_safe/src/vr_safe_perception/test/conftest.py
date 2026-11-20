"""让 pytest 直接从源码树 import vr_safe_perception（不要求先 colcon build）。

本文件在 .../src/vr_safe_perception/test/ 下，Python 包的父目录是
.../src/vr_safe_perception/，把它放进 sys.path 才能 import 里面的同名包。
"""

import os
import sys

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PKG_ROOT not in sys.path:
    sys.path.insert(0, PKG_ROOT)
