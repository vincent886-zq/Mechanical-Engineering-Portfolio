# 机械设计与机电自动化作品集

Mechanical Engineering Portfolio · 结构设计 / 工程图 / 有限元分析 / 科研采集与控制软件

以图纸、模型、分析结果和程序源码展示具体工作。点击下方图片进入案例，查看简介、资料和实现细节。

## 项目目录

| 快递包装箱分拨输送系统 | 轮腿机器人底盘工程图 | 击球机器人有限元分析 |
| :---: | :---: | :---: |
| [![分拨系统](projects/parcel-sorting/images/overview.png)](projects/parcel-sorting/README.md) | [![轮腿底盘](projects/wheel-leg/images/assembly.png)](projects/wheel-leg/README.md) | [![机械臂前六阶模态](projects/robot-fea/images/modal-six.png)](projects/robot-fea/README.md) |
| [查看图文介绍与模型下载 →](projects/parcel-sorting/README.md) | [查看图纸与 DWG 下载 →](projects/wheel-leg/README.md) | [查看工况与仿真结果 →](projects/robot-fea/README.md) |
| SW 零件与装配、DWG、PDF | 最终总装图与零件图 | 模态、接触瞬态、脉冲响应 |

### 慧鱼教学机器人

[![慧鱼三自由度机械臂](projects/fischertechnik-arm/images/assembly.png)](projects/fischertechnik-arm/README.md)

[慧鱼三自由度机械臂与手机上位机 →](projects/fischertechnik-arm/README.md)

模块化机械结构、D-H 运动学与 SolidWorks 几何核对，结合 STM32、ESP-01S 和 App Inventor 手机界面设计。

### 自动存取车控制原型

[![双 STM32 控制架构](projects/automatic-parking-control/diagrams/architecture.svg)](projects/automatic-parking-control/README.md)

[自动存取车：STM32、Wi-Fi 指令与限位采集 →](projects/automatic-parking-control/README.md)

以嵌入式代码为主：双 STM32F103C8、6 路电机控制接口、14 路限位输入、4 个车位的存取命令；附源码与 SVG 程序逻辑图。

## 科研采集与控制软件

面向磁悬浮铣削电主轴实验的平台软件：分别展示八路电流采集、六路位移采集、操作 HMI 与两种通信手轮。采集单位、协议和软件版本独立说明，不混用实验结果。

| 八路电流采集与校准 | 六路位移 / 电压采集 |
| :---: | :---: |
| [![电流采集界面](projects/current-acquisition/images/interface.png)](projects/current-acquisition/README.md) | [![位移采集链](projects/research-daq/diagrams/pipeline.svg)](projects/research-daq/README.md) |
| [电流换算、零点档案、CSV、上下位机应用源码 →](projects/current-acquisition/README.md) | [CRC、样本时间轴、CSV / RAW / 元数据、上位机源码 →](projects/research-daq/README.md) |

| 磁悬浮电主轴 HMI | 有线 USB / Wi-Fi 手轮 |
| :---: | :---: |
| [![电主轴操作界面](projects/spindle-hmi/images/interface.png)](projects/spindle-hmi/README.md) | [![电子手轮原型](projects/dual-link-handwheel/images/prototype.jpg)](projects/dual-link-handwheel/README.md) |
| [XYZ / G54、主轴状态、受限 G 代码、双版本源码 →](projects/spindle-hmi/README.md) | [EC11、OLED、授权握手、STM32 / ESP 源码 →](projects/dual-link-handwheel/README.md) |

## 资料获取

快递分拨案例提供 SolidWorks 原生模型及二维 DWG、PDF 图纸；轮腿案例提供最终 DWG。模型请保留解压后的目录关系，避免丢失装配引用。

超过 100 MiB 的单个文件不放入普通 Git 文件；其余资料可通过 [GitHub 主页](https://github.com/vincent886-zq) 联系交流。

---

[GitHub 主页](https://github.com/vincent886-zq) · [小米散热背夹第三方控制软件](https://github.com/vincent886-zq/Cooling-Fan-R01P-M-Third-party-control-software)
