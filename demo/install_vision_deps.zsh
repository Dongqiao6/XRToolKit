#!/usr/bin/env zsh

setopt ERR_EXIT

if ! command -v apt-get >/dev/null 2>&1; then
  print -u2 "此脚本仅适用于 Ubuntu/Debian。"
  exit 1
fi

sudo apt-get update
sudo apt-get install -y \
  python3-gi \
  gir1.2-gstreamer-1.0 \
  gir1.2-gst-plugins-base-1.0 \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-ugly \
  v4l-utils

print "D405 视频依赖安装完成。"
print "检查设备：v4l2-ctl --list-devices"
print "检查编码器：gst-inspect-1.0 x264enc"
