# 03: 相机观察源

**What to build:** 把 camera_zone 的判定核心搬成独立的 `vr_safe_perception`：Orbbec 335L 实时帧或录像回放 → 五档判定 → 四值 `SafetyObservation`，支持 polygon/reach 保护区、区域/外参标定与有效性检查；相机或深度失效时输出 `UNKNOWN`。

**Blocked by:** 02

**Status:** implemented（代码/回放/测试）；真相机 HIL 待现场

- [ ] 实时 Orbbec 与录像回放都能产出 `/vr_safe/observation`。
- [ ] 五档到四值映射正确：空闲→CLEAR、旁边有人→NEAR、看不清→UNKNOWN、手伸入/占用→OCCUPIED。
- [ ] 同一段录像在改动前后可重复对照，结论一致。
- [ ] polygon 读现有标定格式；reach 用机械臂可达空间；默认 polygon。
- [ ] 外参缺失、`ok:false` 或地面不匹配时生产拒绝启动；开发开关可显式降级。
- [ ] 相机断开/深度全空/画面质量持续不可信时输出 `UNKNOWN`，不是 `CLEAR`。
- [ ] 标定、预览、录像/回放工具可用，不依赖 rerun/table_zones。
