
## 打开与下载

使用 PDS 2022.2-SP6.4 打开根目录 HDMI_RESOURCE_OPT2_20261102.pds。器件为 Logos2 PG2L100H / FBG676 / -6。摄像头接 HDMI IN、显示器接 HDMI OUT，IMU 和供电接线沿用原版。

直接下载时，在 Programmer 中选择 program/HDMI_RESOURCE_OPT2_20261102.sbit。

## 文件用途

- source：运行所需 HDL 和查找表。
- ipcore：PLL、DDR3 等 IP 依赖。ddr3_test 是原 DDR3 控制器 IP 名称，sim_lib 是其配套库，均不是外加测试程序，必须保留。
- program：已编译下载位流。
- reports：PDS 原生综合、资源、布线和时序报告。
- configure_project.tcl：源文件、IP、器件和约束配置。
- prepare_tables.tcl：从原夜间增益表生成紧凑 ROM 文件；修改 night_gain_q8.mem 后先执行本脚本再编译，build.tcl 会自动执行。
- HDMI_IN_DDR3_HDMI_OUT_top.fdc：物理引脚；timing_constraints.fdc：时钟与时序例外；low_latency_priority.rcf：布线优先级。

## 功能与状态

保留畸变、仿射固定裁剪、透视、IMU、CLAHE、夜间增强和降噪。S3 短按原图/矫正、长按约 2 秒切换亮度增强；S4 短按参数面板、长按调整 SYNC；S5 短按仿射、长按约 1 秒切换透视。姿态模式开启时保持矫正路径。



