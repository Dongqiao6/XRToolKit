#!/usr/bin/env bash
# 开：工位占用检测 + rerun 三维窗口（机械臂跟真机关节角）。
# 用法：./start.sh            默认：可达空间当保护范围
#       ./start.sh polygon    改用工位多边形
#
# 可以从任意目录调用（先 cd 到本脚本所在目录）。pid / 日志按本目录区分，
# 同一台机器上存在多份 camera_zone 副本时互不干扰。
# 已经在跑就不重复开。打印从启动到「第一帧判定出来」的耗时。
cd "$(dirname "$0")"
DIR="$(pwd)"
TAG="$(printf '%s' "$DIR" | md5sum | cut -c1-8)"
MODE="${1:-reach}"
PIDF="/tmp/camera_zone-$TAG.pid"
LOG="/tmp/camera_zone-$TAG.log"
VPIDF="/tmp/camera_zone-$TAG.viewer"
rm -f "$VPIDF"

# 只认本目录的进程：命令里带 zone_rerun.py 的绝对路径，别家的副本不会误判。
old=$(pgrep -f "$DIR/zone_rerun.py" | head -1)
if [ -n "$old" ]; then
    echo "已经在跑了（PID $old），不重复开。要重开先 ./stop.sh"
    exit 0
fi

# 锁 CPU/GPU 频率（11-16 用户定：重启后会失效，跟着程序自动锁上，免得忘）。
# 已锁就跳过；没锁就用 sudo -n 锁（要先在 JAKA 上装一次免密规则，只放行 jetson_clocks）。
# 不想锁：NO_CLOCKS=1 ./start.sh
clocks_locked() {
    [ "$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq)" = \
      "$(cat /sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq)" ] &&
    [ "$(cat /sys/class/devfreq/17000000.gpu/min_freq)" = \
      "$(cat /sys/class/devfreq/17000000.gpu/max_freq)" ]
}
if [ "${NO_CLOCKS:-0}" != 1 ]; then
    if clocks_locked; then
        echo "✓ 频率已锁（CPU/GPU 满频）"
    elif sudo -n /usr/bin/jetson_clocks >/dev/null 2>&1 && clocks_locked; then
        echo "✓ 频率刚锁上（CPU/GPU 满频）"
    else
        echo "⚠ 频率没锁（多半是刚重启过），帧率会低一些、抖一些。"
        echo "  手动锁：sudo jetson_clocks    想让这里自动锁：看本目录 README.md「锁频」一节"
    fi
fi

t0=$(date +%s.%N)
DISPLAY="${DISPLAY:-:1}" nohup "$DIR/.venv/bin/python" -u "$DIR/zone_rerun.py" \
    --robot --joints-live --zone-mode "$MODE" > "$LOG" 2>&1 &
echo $! > "$PIDF"

# 等第一条判定打出来才算「开好了」：那时相机出帧、模型加载、窗口连上都已完成
for i in $(seq 1 600); do
    if grep -qE "^\[ *0\.0s\]" "$LOG"; then
        t1=$(date +%s.%N)
        printf "✓ 已开（PID %s），从启动到出第一帧判定 %.1f 秒\n" "$(cat $PIDF)" "$(echo "$t1 - $t0" | bc)"
        grep -E "^相机 \[|摆正|保护范围" "$LOG" | sed 's/^/  /'
        exit 0
    fi
    if ! kill -0 "$(cat $PIDF)" 2>/dev/null; then
        echo "✗ 启动失败，日志最后几行："; grep -vE "^W0000|^I0000|INFO|WARN" "$LOG" | tail -8
        exit 1
    fi
    sleep 0.1
done
echo "✗ 60 秒还没出第一帧，看日志 $LOG"; exit 1
