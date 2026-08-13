# 验收截图归档目录

本目录存放项目各阶段的实测验证照片/截图，按 Bring-up 顺序编号归档。

## 归档命名规范

| 文件名 | 内容 | 对应验收项 |
|--------|------|-----------|
| `01_wiring.png` | 整机接线照片 | 引脚接线表 (README §1) |
| `02_boot_log.png` | 串口启动日志截图（含 `[OK] IWDG enabled, timeout ~2s` 与复位源检测） | 最小系统 + 看门狗 (debugging_guide §1.1) |
| `03_keil_build.png` | Keil MDK 编译结果截图 (`0 Error(s), 0 Warning(s)`) | 编译验收 |
| `04_oled_runtime.png` | OLED 运行画面（姿态角 / ADC / 模式 / 心跳） | 全模块联调 |
| `05_vofa_waveform.png` | VOFA+ FireWater 7 通道波形截图 | 串口 DMA 上报 + 滤波链 (§2.5) |
| `06_servo_tracking.mp4` | 舵机跟随传感器转动视频 | 执行器闭环 (§1.4) |
