# 第十七届全国大学生智能汽车竞赛智能视觉组源代码

比赛录像可以从[这里](https://www.bilibili.com/video/BV1eB4y1x76N)观看。

工程基于[逐飞RT1064_RTThread开源库](https://gitee.com/seekfree/RT1064_RTThread_Library.git)开发，使用`μVision V5.36.0.0 MDK-ARM 5.36.0.0`编译(IAR的文件夹懒得删了)。

使用或参考到的开源库包括：

| 简介                      | 地址                                                                                       |
| ------------------------- | ------------------------------------------------------------------------------------------ |
| 逐飞RT1064_RTThread开源库 | [seekfree/RT1064_RTThread_Library](https://gitee.com/seekfree/RT1064_RTThread_Library.git) |
| 线性代数计算库            | [libeigen/eigen](https://gitlab.com/libeigen/eigen.git)                                    |
| 卡尔曼滤波器库            | [mherb/kalman](https://github.com/mherb/kalman.git)                                        |
| 机器人本地化定位库        | [cra-ros-pkg/robot_localization](https://github.com/cra-ros-pkg/robot_localization.git)    |
| 线性ADRC库                | [psiorx/ADRC](https://github.com/psiorx/ADRC.git)                                          |
| 计算机视觉库              | [opencv/opencv](https://github.com/opencv/opencv.git)                                      |
| Apriltag算法              | [AprilRobotics/apriltag](https://github.com/AprilRobotics/apriltag.git)                    |
| openmv固件                | [openmv/openmv](https://github.com/openmv/openmv.git)                                      |
| OpenArt固件               | [NXPmicro/OpenART](https://github.com/NXPmicro/OpenART.git)                                |
| 旅行商问题求解算法        | [beckysag/traveling-salesman](https://github.com/beckysag/traveling-salesman.git)          |
| 宏函数展开                | [swansontec/map-macro](https://github.com/swansontec/map-macro.git)                        |
| icm20948驱动              | [vedranMv/tm4c_icm20948](https://github.com/vedranMv/tm4c_icm20948.git)                    |

由于使用了子模块，下载工程时请使用递归克隆，即`git clone --recursive https://github.com/FredBill1/RT1064_Smartcar.git`。

项目中与这个仓库配套使用的仓库有：

| 简介                                                                                   | 地址                                                                                              |
| -------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------- |
| 基于python tkinter框架的无线串口上位机                                                 | [FredBill1/FBScope](https://github.com/FredBill1/FBScope.git)                                     |
| 基于tensorflow2.9的模型训练、量化脚本                                                  | [FredBill1/TFLite_Smartcar](https://github.com/FredBill1/TFLite_Smartcar.git)                     |
| 修改的OpenArt固件，支持逆透视变换、更大的FrameBuffer和更适用于比赛任务的find_rects函数 | [FredBill1/OpenART_SRC](https://github.com/FredBill1/OpenART_SRC.git)  (需要切换到`fredbill`分支) |

我们采用了双核方案，切换源代码主从核的方式为修改`/Project/USER/inc/MCU_ID.hpp`中的`MCU_ID`宏定义，`0`为主核，`1`为从核。

因为代码是一个人编写的，很多部分也都是一边学一边写，注释也基本没有，所以很多地方的代码可读性都比较差，也不一定是最优的写法，如果有什么问题，请指正。

详细的工程架构介绍和实现比赛任务的逻辑细节见[技术报告](中国矿业大学（北京）_地灵殿的装修队_智能视觉组.pdf)的内容。