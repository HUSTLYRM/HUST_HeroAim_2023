# HUST_HeroAim_2023

华中科技大学狼牙战队2023赛季英雄视觉 - 辅瞄程序



## 1.项目简介

本程序主要应用在英雄机器人上，采用的均为最简洁的方式，易于阅读理解与实践上手。

硬件上采用`大恒相机` + `8mm镜头` + `NUC11` + `串口`。

功能：

- 定点击打（*）
- 运动预测
- 打前哨站（*）

**效果预览：9 / 13 ≈ 70%**

- [复活赛前的效果记录_哔哩哔哩_bilibili](https://www.bilibili.com/video/BV1CP411b7xh/?vd_source=d3e5165825082cd17457aab2378b8f54)

## 2.环境依赖

```c++
GCC 11.4.0
OpenVINO 2021.4 （Test: 2023.0 也可以）
OpenCV 4.7.0
Eigen 3.4.0
Sophus 1.22.10
Glog 
CeresSolver 2.1.0
Jsoncpp
```

## 3. 程序主要结构

> 
>
> 为了突出主要的代码结构，略去了部分不重要的文件树枝
>
> 

```c++
.
├── armor_detector			装甲板检测模块
│   ├── CMakeLists.txt
│   ├── include
│   │   ├── ArmorFinder.h
│   │   ├── Detector.h
│   │   ├── Inference.hpp
│   └── src
│       ├── ArmorFinder.cpp
│       ├── Detector.cpp	关键函数（*）
│       ├── Inference.cpp
├── AutoAim.cpp				辅瞄主程序
├── AutoAim.h
├── driver					相关的驱动代码（相机、串口）
├── pose_estimate			位姿解算模块（pnp+预测）
│   ├── CMakeLists.txt
│   ├── include
│   │   ├── ExtendedKalman.hpp
│   │   ├── NormalEKF.h
│   │   ├── PoseSolver.h
│   │   ├── Predictor.h
│   │   ├── Predictor_main.h
│   └── src
│       ├── NormalEKF.cpp
│       ├── PoseSolver.cpp
│       ├── Predictor.cpp
│       └── Predictor_main.cpp
└── utils					工具模块
```

## 4.主要功能实现

### 4.1击打前哨站

1. 击打前哨站的逻辑前需要使用ROI提高识别的精度

2. 粗略计算一个装甲板能被连续识别的帧数并适当多取一些（处理一次的时间为10ms左右）
3. 使用循环数组的数据结构（存储每一次解算的相对3D坐标，随着时间的推移，新的内容顶替前面的3D坐标）
4. 从循环数组中提取距离最近的部分坐标进行均值滤波，作为击打点的估计坐标
5. 根据弹速、发弹时间消耗以及前哨站旋转时间确定发弹延迟（即真正击打的装甲板是下一个转过来的装甲板）
6. 当重新进入击打前哨站模式时，清空数组

### 4.2辅瞄

1. 将xyz坐标转换成pitch_yaw_distance
2. 扩展卡尔曼滤波

### 4.3可视化调试工具

借助UDP协议以及VOFA+工具，进行数据的可视化查看与调试（使用方式见下方开源仓库）

[liulog/Debug_udp_VOFA (github.com)](https://github.com/liulog/Debug_udp_VOFA)

## 5.致谢

感谢沈阳航空航天大学TUP战队顾昊以及狼牙战队视觉组的老人们！

## 6.其他

基于nanodet-plus修改的四点网络，在docs中记录了作为一个新人如何一步步修改代码，最终成功训练四点模型，测试效果推理速度略逊于YOLOX-nano，但是不失为新手入门四点的一种参考。

[HUSTLYRM/Nanodet_FP (github.com)](https://github.com/HUSTLYRM/Nanodet_FP)
