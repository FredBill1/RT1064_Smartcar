#ifndef _nodes_hpp
#define _nodes_hpp

// 无线串口接收进程, 用于遥控
bool wirelessNode();

// apriltag检测, 需要初始化mt9v03x
bool apriltagDetectNode();

// 矩形检测, 需要初始化mt9v03x
bool findRectNode();

// 上位机传图, 需要初始化usb_cdc和mt9v03x
bool imgUSBXferNode();

// 使用rtthread timer驱动的周期性电机转速控制
bool motorControlNode();

// 用按键和ips屏幕测试电机
bool testMotorNode();

// 测试qtimer
bool testQTimerNode();

// 结合上位机测试电机PWM, 需要禁用wirelessNode和motorControlNode
bool testMotorPwmNode();

// 测试按键, 需要关闭编码器初始化
bool testKeyNode();

// 主从机传输测试
bool uartMasterTest();
bool uartSlaveTest();

// 卡尔曼滤波定位
bool poseKalmanNode();

// 测试卡尔曼滤波类
bool testPoseKalmanNode();

// 测试局部路径规划类
bool testLocalPlannerNode();

// 测试TSP类
bool testTSPSolverNode();

// 测试Canny边缘检测
bool testCannyNode();

#endif  // _nodes_hpp