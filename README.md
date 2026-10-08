# FSD 城市自动驾驶模拟器

基于 TypeScript、Three.js 和 Vite 的浏览器自动驾驶演示，使用 OpenStreetMap 的巴黎凯旋门周边路网。

支持自动导航、跟车、变道、行人避让和红绿灯通行；结合采样轨迹优化与 Hybrid A*，处理绕行、倒车和非道路脱困。界面可显示候选轨迹、预测轨迹与驾驶统计，并调整交通密度。

[在线体验](https://jiaxuanzou0714.github.io/fsd-sim/)

## 启动

需要 Node.js 22.12+ 的 LTS 版本。

```bash
npm install
npm run dev
```

打开终端显示的本地地址（默认 `http://localhost:5173`），点击「开始体验」。地址后加 `#play` 可跳过开场说明。

```bash
npm run build    # 类型检查并构建到 dist/
npm run preview  # 预览构建结果
npm test         # 运行仿真测试
```

## 操作

| 操作 | 功能 |
| --- | --- |
| 点击小地图或路面 | 设置目的地 |
| `F` | 启用 / 退出自动驾驶 |
| `WASD` / 方向键 | 手动驾驶，自动驾驶时触发接管 |
| `空格` | 制动，自动驾驶时触发接管 |
| `R` | 随机目的地 |
| `J` / `B` / `K` | 生成横穿行人 / 违停车辆 / 自行车 |
| `O` / `V` | 从非道路区域 / 单行道逆向位置测试脱困 |
| `C` | 切换视角 |
| `T` | 切换 1× / 2× / 4× 速度 |
| `P` | 暂停 / 继续 |

## 说明

感知直接读取仿真状态，其他车辆和行人由规则驱动，规划权重为人工设定。

地图构建脚本位于 `tools/build-map.mjs`，数据 © [OpenStreetMap 贡献者](https://www.openstreetmap.org/copyright)，按 ODbL 许可使用。
