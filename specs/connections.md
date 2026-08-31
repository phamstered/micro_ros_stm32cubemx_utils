
### Damiao Jetson Device

使用 `ssh damiao` 登陆 Jetson

### Damiao STM32H723 Board

./scripts/flash.sh 刷写固件到此开发版

### Wire Connection

Jetson 与 STM32 通过 usb virtual com port 连接
```
> lsusb
Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 001 Device 005: ID 8087:0029 Intel Corp. AX200 Bluetooth
Bus 001 Device 004: ID 0483:5740 STMicroelectronics Virtual COM Port
```

### Tasks
1. 先禁用 vofa 串口调试代码，避免数据冲突
2. 集成 micro_ros 到 freertos，并刷写新固件
3. 在 damiao jetson 中安装 micro-ros agent，使用 Virtual COM Port 连接 STM32 Board，**Jetson中使用 Ros2 Humble 版本**
4. 测试 micro-ros 节点订阅发布

