# 生产模式拒绝未标定或过期的外参

外参缺失、`extrinsic_calib.json` 的 `ok` 不为 true，或标定地面与当前 `zone_config.json` 差超过 0.5°/2 cm 时，生产启动直接失败；开发用 `--allow-uncalibrated` 显式降级到设计值并打警告。相机位置一变，旧外参必须失效，不能静默继续用。

## Considered Options

- 自动回退设计值：能启动，但 `sensors.json` 自己标着 `extrinsics_calibrated: false`，安全距离会错。
- 只要有文件就用：当前 `extrinsic_calib.json` 就是 `ok:false`，等于把一次失败的标定当成功。
