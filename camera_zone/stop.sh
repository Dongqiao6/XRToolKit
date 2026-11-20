#!/usr/bin/env bash
# 关：先停检测（相机会正常关流），再关 rerun 窗口。打印耗时。
# 用法：./stop.sh            检测和窗口都关
#       ./stop.sh keep-view  只停检测，窗口留着看回放
#
# 可以从任意目录调用。pid / 日志按本目录区分，只停本目录启的那一份，
# 不会误杀同一台机器上其他 camera_zone 副本的进程。
#
# 停检测发 SIGTERM：程序把它当 Ctrl+C 处理，会走清理把相机关掉。
# 不发 SIGINT——后台启动的程序被 bash 设成忽略 SIGINT，发了等于没发。
# 千万别用 kill -9：相机 SDK 来不及关流，固件会卡成「能打开不出帧」。
cd "$(dirname "$0")"
DIR="$(pwd)"
TAG="$(printf '%s' "$DIR" | md5sum | cut -c1-8)"
PIDF="/tmp/camera_zone-$TAG.pid"
VPIDF="/tmp/camera_zone-$TAG.viewer"
t0=$(date +%s.%N)

pid=$(pgrep -f "$DIR/zone_rerun.py" | head -1)
if [ -z "$pid" ]; then
    echo "检测没在跑"
else
    # rerun 查看器是本进程的子进程；记住它，只关本目录开的那个窗口，
    # 不碰同一台机器上其他副本的窗口。
    view=$(pgrep -P "$pid" -f "[r]erun_cli/rerun" | head -1)
    if [ -n "$view" ]; then echo "$view" > "$VPIDF"; fi
    kill -TERM "$pid"
    for i in $(seq 1 250); do kill -0 "$pid" 2>/dev/null || break; sleep 0.1; done
    if kill -0 "$pid" 2>/dev/null; then
        echo "✗ 检测进程 $pid 25 秒没退出，没敢硬杀（会卡死相机）。看 /tmp/camera_zone-$TAG.log"
        exit 1
    fi
    t1=$(date +%s.%N)
    printf "✓ 检测已停（PID %s），耗时 %.1f 秒，相机已释放\n" "$pid" "$(echo "$t1 - $t0" | bc)"
fi
rm -f "$PIDF"

if [ "$1" != "keep-view" ]; then
    v=""
    if [ -f "$VPIDF" ]; then
        v=$(cat "$VPIDF")
        if ! kill -0 "$v" 2>/dev/null; then v=""; fi
    fi
    if [ -n "$v" ]; then
        kill -TERM "$v"
        for i in $(seq 1 50); do kill -0 "$v" 2>/dev/null || break; sleep 0.1; done
        t2=$(date +%s.%N)
        printf "✓ rerun 窗口已关，总耗时 %.1f 秒\n" "$(echo "$t2 - $t0" | bc)"
    fi
    rm -f "$VPIDF"
fi
