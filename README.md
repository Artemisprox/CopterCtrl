# CopterCtrl

基于串口（RS-485/Modbus-RTU风格）电机通信与100Hz PID闭环控制示例。

## 文件说明

- `motor_comm.py`：电机通信驱动封装，包含 CRC16、03/04读、06/10写、42使能、41保存、43清故障。
- `motor_control.py`：上层100Hz定时控制线程，支持速度环与位置+速度串级PID闭环。

## 快速开始

1. 安装依赖：

```bash
pip install pyserial
```

2. 修改 `motor_comm.py` 中 `RegisterMap` 的寄存器地址为你的驱动器实际映射。
3. 运行：

```bash
python motor_control.py
```

> 默认串口参数：19200, 8E1（与文档中示例一致）。
