# 08: 实时相机窗口与 RJ 恢复

**What to build:** 感知节点发布 15 FPS 压缩彩色图；独立 `vr_safe_camera_view` 叠加保护区、许可、观察原因和恢复提示；`REVOKED` 红框、`SLOW` 黄框、`GRANTED` 绿框；操作员 `RJ` 长按 1.5 s 请求恢复许可。

**Blocked by:** 02, 04

**Status:** implemented（overlay 测试 + 构建；真 Quest/相机 HIL 待现场）

- [x] `/vr_safe/camera/compressed` 15 FPS JPEG，仅在 viewer 开启时发布。
- [x] viewer 显示区域、顶部许可/观察/原因、底部恢复提示；REVOKED 红框。
- [x] `REVOKED` 且 `CLEAR/NEAR` 时提示“按 RJ 长按 1.5s 恢复”；OCCUPIED/UNKNOWN 提示不能恢复。
- [x] `RJ` 长按 1.5 s 调用 `/vr_safe/confirm_resume`，确认结果通过 `/vr_safe/teleop/confirm_status` 回到画面。
- [x] `start.sh --camera-view` / `--no-camera-view`；有 DISPLAY 时默认开。
- [ ] 真相机窗口、红框/文字、RJ 恢复的现场 HIL。
