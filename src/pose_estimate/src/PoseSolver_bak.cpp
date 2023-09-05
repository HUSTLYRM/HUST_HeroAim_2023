// //
// // Created by zhiyu on 2021/8/20.
// //

// #include "../include/PoseSolver.h"

// using namespace std;
// namespace ly
// {
//     Eigen::Matrix<double, 8, 1> angle_state_vector;

//     PoseSolver::PoseSolver()
//     {

//         // 设置相机内外参数
//         setCameraMatrix(CameraParam::fx, CameraParam::fy, CameraParam::u0, CameraParam::v0);
//         // 设置相机畸变
//         setDistortionCoefficients(CameraParam::k1, CameraParam::k2, CameraParam::p1, CameraParam::p2, CameraParam::k3);

//         // 初始化，gimbal_to_world 只有旋转向量, 跟pitch、raw、roll有关
//         gimbal_to_world = Sophus::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 0));

//         double degree=-8;
//         double DEG_TO_ARC = 0.0174532925199433;
//         Eigen::Vector3d euler_angle(degree*DEG_TO_ARC,0,0); // 
//         Eigen::Matrix3d rotation_matrix;
// 		rotation_matrix = Eigen::AngleAxisd(euler_angle[2], Eigen::Vector3d::UnitZ()) *
//        	                                  Eigen::AngleAxisd(euler_angle[1], Eigen::Vector3d::UnitY()) *
//                                           Eigen::AngleAxisd(euler_angle[0], Eigen::Vector3d::UnitX());

//         // 初始化 相机相 对于 云台 （只有一个平移向量）
//         // TODO 这里可能需要改动，因为存在一定的夹角
//         // FIXME 不是只有平移矩阵了，相机和枪管有一定的角度
//         camera_to_gimbal = Sophus::SE3(rotation_matrix, Eigen::Vector3d(CameraParam::camera_trans_x, CameraParam::camera_trans_y, CameraParam::camera_trans_z));

//         // 创建预测器
//         predictor = new Predictor();

//         if (GlobalParam::SOCKET)
//         {
//             // 创建了socket连接
//             // 端口号3000
//             udpsender = new UDPSender("192.168.1.3", 3000);
//         }
//     }

//     // 设置相机内参
//     void PoseSolver::setCameraMatrix(double fx, double fy, double u0, double v0)
//     {
//         camera_matrix = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));
//         camera_matrix.ptr<double>(0)[0] = fx;
//         camera_matrix.ptr<double>(0)[2] = u0;
//         camera_matrix.ptr<double>(1)[1] = fy;
//         camera_matrix.ptr<double>(1)[2] = v0;
//         camera_matrix.ptr<double>(2)[2] = 1.0f;
//     }

//     // 设置畸变系数矩阵
//     void PoseSolver::setDistortionCoefficients(double k_1, double k_2, double p_1, double p_2, double k_3)
//     {
//         distortion_coefficients = cv::Mat(5, 1, CV_64FC1, cv::Scalar::all(0));
//         distortion_coefficients.ptr<double>(0)[0] = k_1;
//         distortion_coefficients.ptr<double>(1)[0] = k_2;
//         distortion_coefficients.ptr<double>(2)[0] = p_1;
//         distortion_coefficients.ptr<double>(3)[0] = p_2;
//         distortion_coefficients.ptr<double>(4)[0] = k_3;
//     }

//     // 更新imu的信息数据
//     void PoseSolver::setimu(float pitch, float yaw, float roll)
//     {
//         Eigen::Matrix3d rotation_matrix3;

//         // 创建一个旋转矩阵
//         rotation_matrix3 = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) *    // 绕z轴旋转0弧度
//                            Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitY()) * // 绕y轴旋转-yaw弧度
//                            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX()); // 绕x轴旋转pitch弧度

//         // 创建一个平移向量
//         // gimbal相对于世界坐标系只有一个旋转，没有平移
//         gimbal_to_world = Sophus::SE3(rotation_matrix3, Eigen::Vector3d(0, 0, 0)); // x,y,z平移均为0
//     }

//     /**
//      * @brief 求bc的夹角
//      * 
//      * @param a 
//      * @param b 
//      * @param c 
//      * @return float 
//      */
//     float PoseSolver::cosineLaw(float a, float b, float c)
//     {
//         double value = fmin(fmax((a * a + b * b - c * c) / (2 * a * b), -1), 1);
//         DLOG(INFO) << "a: " << a << " b: " << b << " c: " << c << " acos: " << value;
//         return acos(value) / M_PI * 180;
//     }

//     /**
//      * @brief 更新时间
//      * 
//      * @param delta_time 
//      */
//     void PoseSolver::update_delta_t(double &delta_time)
//     {
//         delta_t = delta_time;
//     }

//     /**
//      * @brief
//      *
//      * @param armor 指定的要解算位姿的装甲板
//      * @param imu_data 当前的imu数据，包括pitch、yaw、roll，都是相对于世界坐标系的数据
//      *                 pitch、roll数据是根据实际得到的，yaw轴数据是相当于初始上电时的基准来的
//      *                 pitch 当前云台与水平面的夹角
//      */
//     void PoseSolver::solveArmorBA(ArmorBlob &armor, const SerialPortData &imu_data)
//     {

//         // 装甲板的位姿
//         const static Sophus::SE3 armor_pose(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 0));

//         // TODO: 更新识别大小装甲板的规则
//         // 最新的装甲板编号中 1 是英雄， 最新规则中只有英雄和大基地是大装甲板
//         // FIXME 对于平衡步兵要单独增加判断
//         if (armor._class == 1)
//         { // large armor
//             points_3d = points_large_3d;
//             // halflength_of_armor = 0.1125f;              // 装甲板
//             // halfheight_of_armor = 0.0275f;
//         }
//         else
//         { // small armor
//             // 假定就是
//             points_3d = points_small_3d;
//         }

//         // pnp 解算，并计算重投影误差， SOLVEPNP_IPPE只会返回两组解 
//         solvePnPGeneric(points_3d, armor.corners, camera_matrix, distortion_coefficients,
//                             rvecs, tvecs, false, cv::SOLVEPNP_IPPE, noArray(), noArray(), reproject_error);

//         // TODO 进行 BA 优化
//         // 返回的第一组解
//         cv::Mat R1;
//         cv::Rodrigues(rvecs[0], R1); // 旋转矢量转换为旋转矩阵
//         Eigen::Matrix3d Rotate_M1 = Eigen::Matrix3d::Identity();
//         cv::cv2eigen(R1, Rotate_M1);

//         // 返回的第二组解
//         cv::Mat R2;
//         cv::Rodrigues(rvecs[1], R2); // 旋转矢量转换为旋转矩阵
//         Eigen::Matrix3d Rotate_M2 = Eigen::Matrix3d::Identity();
//         cv::cv2eigen(R2, Rotate_M2);

//         // 四元素
//         Eigen::Quaterniond q4_1;
//         Eigen::Isometry3d T_;

//         // detector检测出来的四个角点
//         Eigen::Matrix<double, 4, 2> points_2;

//         // 四个角点
//         // 0 1
//         // 3 2
//         points_2(0, 0) = armor.corners[0].x;points_2(0, 1) = armor.corners[0].y;
//         points_2(1, 0) = armor.corners[1].x;points_2(1, 1) = armor.corners[1].y;
//         points_2(2, 0) = armor.corners[2].x;points_2(2, 1) = armor.corners[2].y;
//         points_2(3, 0) = armor.corners[3].x;points_2(3, 1) = armor.corners[3].y;

//         // 装甲板的宽/2 高/2
//         Eigen::Vector2d two_ = Eigen::Vector2d{-points_3d[0].x, -points_3d[0].y};

//         //
//         //          装甲板的宽/2,高/2  四点坐标  相机内参矩阵  旋转矩阵 平移向量  （第一组解）
//         T_ = bundleAdjustment(two_, points_2, camera_matrix, R1, tvecs[0]);
//         // BA优化

//         std::cout << "BA优化完成" << std::endl;

//         // 优化后的旋转矩阵
//         q4_1 = Eigen::Quaterniond(T_.rotation().matrix());

//         // 平移向量, 第一组解是z轴相反的
//         Eigen::Vector3d translate1(tvecs[0].ptr<double>(0)[0], tvecs[0].ptr<double>(0)[1], -tvecs[0].ptr<double>(0)[2]);

//         // 第二组解的旋转矩阵
//         // Eigen::Quaterniond q4_2 = Eigen::Quaterniond(Rotate_M2);
//         // DLOG(WARNING) << "reproject_error" << reproject_error << std::endl;

//         // 对于优化后的结果进行滤波
//         // 重投影误差
//         // if ((reproject_error[1] / reproject_error[0]) > 1.5 /*&&(reproject_error[1]+reproject_error[2]>4)*/)
//         // {
//         //     // reproject_error[0] 比较小，也就是选取的这部分的重投影误差小，就继续使用
//         //     // DLOG(WARNING) << "reproject_error[1] / reproject_error[0] > 1.5"<<std::endl;
//         //     if (angle_fliter->dont_believe_measure)
//         //     {
//         //         // DLOG(WARNING) << "dont_believe_mesaure" << std::endl;
//         //         angle_fliter->setMeasurementNoise(0.0001, 0.0001, 0.0001, 0.0001); // 调回测量噪声
//         //         angle_fliter->dont_believe_measure = false;
//         //     }
//         //     // 获取滤波后的滤波后的数据，后验状态估计
//         //     angle_state_vector = angle_fliter->runKalman(q4_1, delta_t);
//         //     // DLOG(WARNING)<<"q4_1 "<<q4_1<<" delta_t"<<delta_t<<std::endl;
//         //     // DLOG(WARNING)<<"angle_state_vector "<<angle_state_vector<<std::endl;

//         // }
//         // else
//         // {
//         //     // DLOG(ERROR) << "reproject_error" << reproject_error << std::endl;
//         //     angle_fliter->setMeasurementNoise(0.001, 0.001, 0.001, 0.001); // 调大测量噪声
//         //     angle_state_vector = angle_fliter->runKalman(q4_1, delta_t);
//         //     angle_fliter->dont_believe_measure = true;
//         //     // angle_fliter->if_first_jump=true;
//         // }

//         // 旋转矩阵
//         // Eigen::Matrix3d rotation_matrix3;
//         // Quaterniond qqq(angle_state_vector[0], angle_state_vector[2], angle_state_vector[4], angle_state_vector[6]);
//         // rotation_matrix3 = qqq.toRotationMatrix();

//         // 旋转矩阵、平移向量（优化后的旋转矩阵、第一组解的平移向量）
//         armor_to_camera = Sophus::SE3(T_.rotation(), translate1);
//         // armor_to_camera = Sophus::SE3(rotation_matrix3, translate1) * armor_pose;


//         // cv2eigen(tvec, e_T);
//         // // 旋转向量转旋转矩阵
//         // Rodrigues(rvec, m_R);
//         // cv2eigen(m_R, e_R);

//         // 将 armor_pose 从装甲板坐标系转换成相机坐标系
//         // armor_to_camera = Sophus::SE3(e_R, e_T) * armor_pose;
//         // 相机坐标系坐标 转换成 世界坐标系
//         camera_to_world = gimbal_to_world * camera_to_gimbal;
//         // 装甲板坐标 转换到 云台坐标系
//         armor_to_gimbal = camera_to_gimbal * armor_to_camera;
//         // 装甲板坐标 转换到 世界坐标系
//         armor_to_world = camera_to_world * armor_to_camera;

//         // 得到当前的yaw轴数据      角度转成弧度
//         float rcv_yaw = imu_data.yaw / 100.0f * M_PI / 180.0f;

//         // 得到当前的pitch轴数据    角度转弧度
//         float rcv_pitch = imu_data.pitch / 100.0f * M_PI / 180.0f;

//         // roll轴的数据
//         // float rcv_roll = roll.getMetric() / roll.getSize();
//         // TODO
//         float rcv_roll = imu_data.roll / 100.0f * M_PI / 180.0f;

//         // 更新当前的imu数据, 更新云台 -> 世界坐标系的转化矩阵
//         setimu(rcv_pitch, rcv_yaw, rcv_roll);

//         // DLOG(INFO) << "                              corners: " << armor.corners;

//         // double z = atan2(m_R.at<double>(1, 0), m_R.at<double>(0, 0));

//         // TODO 后续继续理解这个yaw值，这个yaw求的是装甲板的yaw方向上的倾斜角度
//         // m_R 装甲板坐标系对相机坐标系的yaw偏角
//         // FIXME 还要进一步阅读理解 yaw轴的偏差角度

//         // TODO 更新旋转矩阵，要重新测试，获取装甲板对世界坐标系的
//         Eigen::Matrix3d rotation_matrix = armor_to_camera.rotation_matrix();
//         cv::eigen2cv(rotation_matrix, m_R);


//         // 角度
//         yaw = atan2(-m_R.at<double>(2, 0), sqrt(pow(m_R.at<double>(2, 0), 2) + pow(m_R.at<double>(2, 2), 2))) / M_PI * 180;

//         DLOG(INFO) << "armor_angle"<< yaw;
//         // double x = atan2(m_R.at<double>(2, 1), m_R.at<double>(2, 2));
//         // DLOG(INFO) << "z: " << z << " y: " << yaw << " x: " << x;

//         // 装甲板在云台坐标系下的坐标
//         // const auto &a = armor_to_gimbal;
//         // Point3d t = {a.translation()[0],
//         //              a.translation()[1],
//         //              a.translation()[2]};

//         // DLOG(INFO) << t;

//         // 装甲板在世界坐标系下的坐标
//         // 得到yaw轴的坐标
//         Point3d trans = {-armor_to_world.translation()[0], // x
//                          -armor_to_world.translation()[1], // y
//                          armor_to_world.translation()[2]}; // z

//         // 确定一下pitch、yaw、roll和rotation() 坐标的对应关系

//         // 装甲板的角度
//         armor.angle = yaw;

//         // 世界坐标系中的坐标
//         armor.x = trans.x;
//         armor.y = trans.y;
//         armor.z = trans.z;


//         // 使用VOFA+ 进行udp通信调试
//         if (GlobalParam::SOCKET)
//         {   
//             outpostPoseDataFrame.x = armor.x;
//             outpostPoseDataFrame.y = armor.y;
//             outpostPoseDataFrame.z = armor.z;
//             outpostPoseDataFrame.pitch = armor.angle;
//             // outpostPoseDataFrame.yaw = imu_data.yaw;

//             // 原始pnp解算结果
//             const vector<Point3f> points_small = {Point3f(-0.675f, -0.0275f, 0.f),
//                                                 Point3f(0.0675f, -0.0275f, 0.f),
//                                                 Point3f(0.0675f, 0.0275f, 0.f),
//                                                 Point3f(-0.0675f, 0.0275f, 0.f)};

//             // BA优化前的yaw angle
//             solvePnP(points_small, armor.corners, camera_matrix, distortion_coefficients,
//                      rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
//             cv2eigen(tvec, e_T);

//             // 旋转向量转旋转矩阵
//             Rodrigues(rvec, m_R);
//             cv2eigen(m_R, e_R);
//             yaw = atan2(-m_R.at<double>(2, 0), sqrt(pow(m_R.at<double>(2, 0), 2) + pow(m_R.at<double>(2, 2), 2))) / M_PI * 180;


//             outpostPoseDataFrame.yaw = yaw;
//             // outpostPoseDataFrame.yaw = armor.angle;
//             // outpostPoseDataFrame.roll = ;
//             udpsender->send(outpostPoseDataFrame);
//         }

//     }

//     /**
//      * 辅瞄模式的状态估计
//      * armors 识别得到的装甲板估计
//      * delta_t 时间间隔
//      * imu_data 从串口中读到的imu_data
//      * serialport 串口
//      */
//     void PoseSolver::getPoseInCamera(vector<ArmorBlob> &armors, double delta_t, const SerialPortData &imu_data, SerialPort *SerialPort_)
//     {
//         if (armors.size() < 1)
//             return;

//         // 辅瞄模式不打哨兵和前哨站
//         // 对每一个 装甲板的位姿进行解算
//         for (int i = 0; i < armors.size(); i++)
//         {
//             if (armors[i]._class == 7 || armors[i]._class == 6)
//                 armors.erase(armors.begin() + i);
//             else
//                 solveArmor(armors[i], imu_data);
//         }

//         // 会自动根据陀螺和反陀螺
//         vector<ArmorBlob> candidates;
//         DLOG(INFO) << "state: " << (StateParam::state == ANTITOP ? "ANTITOP" : "AUTOAIM");

//         // 进入反陀螺模式 ANTITOP
//         if (StateParam::state == ANTITOP)
//         {
//             // 和上一帧锁定的装甲板编号一样的装甲板 为 候选装甲板
//             for (const auto &a : armors)
//             {
//                 if (a._class == last_armor._class)
//                 {
//                     candidates.push_back(a);
//                 }
//             }

//             // 此时没有识别到候选装甲板, 一种情况是确实不存在了，还有一种情况是当时没有识别出来（掉帧了）
//             if (candidates.size() < 1)
//             {
//                 lost_cnt++;
//                 if (lost_cnt > 50)
//                 { // 增加了掉帧缓冲， 这样不会立马退出反陀螺模式
//                     StateParam::state = AUTOAIM;
//                     lost_cnt = 0;
//                     circle.clear(); // 清空 circle （循环数组，存放装甲板的3D坐标）
//                     DLOG(INFO) << "exit top mode";
//                 }
//                 DLOG(INFO) << "lost cnt: " << lost_cnt;
//                 return;
//             }
//             // 连续识别，清零掉帧计数
//             lost_cnt = 0;

//             // 将候选框加入到circle里   相当于一个循环数组
//             for (const auto &a : candidates)
//             {
//                 circle.update({a.x, a.y, a.z});
//             }

//             // 按照距离上一帧装甲板的距离进行排序
//             sort(candidates.begin(), candidates.end(), [&](const ArmorBlob &a, const ArmorBlob &b)
//                  { return calcDiff(a, last_armor) < calcDiff(b, last_armor); });

//             // 找到最接近上一帧装甲板的候选装甲板
//             armor = candidates.at(0);

//             // center 这个装甲板的中心坐标
//             center.x = armor.x;
//             center.y = armor.y;
//             center.z = armor.z;
//             if (circle.getSize() == circle.size())
//             {
//                 // 获取中心(取了连续100帧相同类别的均值作为中心坐标)
//                 // 计算中心
//                 center = circle.getMetric() / circle.size();
//                 DLOG(INFO) << "x: " << center.x << " z: " << center.z << " y: " << center.y;

//                 // 遍历候选装甲板 更新选中的装甲板
//                 for (const auto &a : candidates)
//                 {
//                     if (abs(last_armor.angle - a.angle) < 15 && calcDiff(armor, last_armor) <= 0.2)
//                     {
//                         armor = a;
//                     }
//                 }
//                 DLOG(INFO) << "last armor - armor: " << abs(last_armor.angle - armor.angle) << " diff: " << calcDiff(armor, last_armor);

//                 // 如果选中的装甲板不满足信息, 就更新退出时间
//                 // TODO: 大于号是不是要改成小于号
//                 if (calcDiff(armor, last_armor) > 0.2)
//                 {
//                     top_exit = std::chrono::steady_clock::now();
//                 }

//                 exit_duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - top_exit).count() / 1000.0;
//                 DLOG(INFO) << "duration " << exit_duration << " top cnt: " << top_cnt;

//                 // 持续时间>2，那么就调整为辅瞄模式AUTOAIM
//                 if (exit_duration > 2)
//                 {
//                     StateParam::state = AUTOAIM;
//                     last_armor = armor;
//                     top_cnt = 0;
//                     return;
//                 }

//                 // 180°
//                 double w = 0.5 * 360;
//                 // 获得与车的水平距离
//                 double distance = sqrt(center.x * center.x + center.z * center.z);
//                 // 估计子弹飞行时间（水平距离）
//                 double time = distance / speed;
//                 // 不 击打
//                 SerialParam::send_data.shootStatus = 0;

//                 // 候选框
//                 for (const auto &a : candidates)
//                 {
//                     DLOG(INFO) << "center diff: " << abs(a.x - center.x);
//                     // 找到了满足距离中心的条件（反陀螺）
//                     if (abs(a.x - center.x) < 0.02)
//                     {

//                         shoot_duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - top_exit).count() / 1000.0;

//                         shoot_begin = std::chrono::steady_clock::now();
//                         DLOG(INFO) << "        shoot duration: " << shoot_duration;

//                         thread([this, a, time, w, SerialPort_]() { // 发送数据线程
//                             int sleep_time = (90 / w - time) * 1000;
//                             // 休眠一会
//                             std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
//                             SerialParam::send_data.shootStatus = 1;
//                             // 开始击打
//                             SerialPort_->writeData(&SerialParam::send_data);
//                         })
//                             .detach();
//                         break;
//                     }
//                 }
//             }

//             DLOG(INFO) << "                                           x: " << center.x << " y: " << center.y << " z: " << center.z;

//             // 解算数据
//             // yaw轴根据卡尔曼的x,y得到, 这里设置的false，不使用预测器，center可以暂时认定是车辆的中心
//             SerialParam::send_data.yaw = predictor->getYaw(center, delta_t, speed, imu_data, false);

//             // pitch轴根据解算弹道得到， 直接根据中心和速度估算
//             SerialParam::send_data.pitch = predictor->fitTrajectory(center, speed);

//             DLOG(INFO) << "                                           angle: " << yaw;
//             DLOG(INFO) << "                                           send yaw: " << SerialParam::send_data.yaw << " send pitch: " << SerialParam::send_data.pitch;
//             DLOG(INFO) << "                                           recv yaw: " << SerialParam::recv_data.yaw << "  recv pitch: " << SerialParam::recv_data.pitch;
//             DLOG(INFO) << "                                           x: " << -armor.x << " y: " << -armor.y << " z: " << armor.z;

//             // 当前绑定的装甲板是中心
//             last_armor = armor;
//         }
//         else
//         { // AUTOAIM

//             right_clicked = imu_data.right_clicked; // 接收到发送的右击数据
//             if (last_right_clicked == 0 && right_clicked == 1)
//                 first = true;

//             if (first)
//             { // 按下右键时瞄准中心装甲
//                 if (armors.size() < 1)
//                     return;
//                 first = false;
//                 // 按照距离中心的距离来进行排序，找到最靠近中心的装甲板
//                 sort(armors.begin(), armors.end(), [](const ArmorBlob &a, const ArmorBlob &b) -> bool
//                      {
//                     const Rect& r1 = a.rect;
//                     const Rect& r2 = b.rect;
//                     return abs(r1.x+r1.y+r1.height/2+r1.width/2-1024/2-1280/2) < abs(r2.x+r2.height/2-1024/2+r2.y+r2.width/2-1280/2); });
//                 armor = armors.at(0);
//                 top_pri = armor._class;
//             }

//             int target = chooseArmor(armors);    // 选择装甲板
//             SerialParam::send_data.num = target; // 选定的目标

//             // 优先级最高的
//             for (const auto &a : armors)
//             {
//                 if (a._class == target)
//                     candidates.push_back(a);
//             }

//             DLOG(INFO) << "target: " << target << " size: " << candidates.size();
//             if (candidates.size() < 1)
//                 return;
//             // DLOG(INFO) << "target: " << target << " size: " << candidates.size();

//             // candidates都是装甲板和上一帧锁定的装甲板类别一致的
//             if (target == last_armor._class)
//             { // 上一帧出现过
//                 // 按照和上一帧出现的装甲板距离进行降序排序（相当于一个追踪的效果）
//                 // 上一帧同装甲
//                 if (candidates.size() > 1)
//                 {
//                     sort(candidates.begin(), candidates.end(), [&](const ArmorBlob &a, const ArmorBlob &b)
//                          { return calcDiff(a, last_armor) < calcDiff(b, last_armor); });
//                     // DLOG(INFO) << "diff: " << calcDiff(candidates.at(0), last_armor) << " " << calcDiff(candidates.at(1), last_armor);
//                 }

//                 armor = candidates.at(0);

//                 // 掉帧缓冲
//                 // 确定当前选定的装甲板是哪个
//                 if (lost_cnt < 20 && calcDiff(armor, last_armor) > 0.25)
//                 {
//                     armor = last_armor;
//                     lost_cnt++;
//                 }
//                 else
//                 {
//                     lost_cnt = 0;
//                     armor = candidates.at(0);
//                 }
//                 DLOG(INFO) << "lost cnt: " << lost_cnt;

//                 if (calcDiff(armor, last_armor) > 0.3 && fabs(armor.angle - last_armor.angle) > 20)
//                 {
//                     top_cnt++;                                 // 反陀螺计数
//                     predictor->setStatePost(armor.x, armor.z); // 设置后验状态, 测量数据
//                     top_begin = std::chrono::steady_clock::now();
//                 }

//                 DLOG(INFO) << "last armor - armor: angle: " << fabs(last_armor.angle - armor.angle) << " diff: " << calcDiff(armor, last_armor);

//                 double duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - top_begin).count() / 1000.0;
//                 if (duration > 1.5)
//                 {
//                     top_cnt = 0;
//                 }
//                 DLOG(INFO) << "duration " << duration << " top cnt: " << top_cnt;

//                 // 连续检测到5次满足反陀螺条件，转反陀螺模式
//                 if (top_cnt > 5)
//                 {                                // 每次选择的装甲板类别一样，但是 连续5次识别都不满足一些约束条件，就认为其进入了反陀螺，然后设置状态为反陀螺
//                     StateParam::state = ANTITOP; // 反陀螺模式
//                     top_exit = std::chrono::steady_clock::now();
//                     top_cnt = 0;
//                     DLOG(INFO) << "top mode";
//                     SerialParam::send_data.shootStatus = 0; // 暂停射击（给反陀螺一定的收敛时间）
//                 }
//                 else
//                 {
//                     SerialParam::send_data.shootStatus = 1; // 开始射击（AUTOAIM射击）
//                 }
//             }
//             else
//             {

//                 // 上一帧锁定的目标和当前确定的目标不一致
//                 DLOG(INFO) << "lost cnt: " << lost_cnt;
//                 DLOG(INFO) << "no same class.";

//                 // 重置预测器
//                 predictor->reset();

//                 // 掉帧缓冲
//                 if (lost_cnt < 30 && calcDiff(armor, last_armor) > 0.1)
//                 {
//                     armor = last_armor;
//                     lost_cnt++;
//                 }
//                 else
//                 {
//                     lost_cnt = 0;
//                     // 根据候选框的绝对距离排序
//                     sort(candidates.begin(), candidates.end(), [](const ArmorBlob &a, const ArmorBlob &b) -> bool
//                          { return a.x * a.x + a.z * a.z < b.x * b.x + b.z * b.z; });
//                     // 取出候选框中最近的那个目标
//                     armor = candidates.at(0);
//                 }
//             }

//             // 装甲板的3D坐标
//             cur = Point3d(armor.x, armor.y, armor.z);

//             // 得到yaw轴数据 predictor = False
//             SerialParam::send_data.yaw = predictor->getYaw(cur, delta_t, speed, imu_data); // 默认yaw轴是考虑预测的
//             // 启用弹道拟合
//             SerialParam::send_data.pitch = predictor->fitTrajectory(cur, speed); // 默认pitch轴是不考虑预测的

//             // // 使用VOFA+ 进行udp通信调试
//             // if(GlobalParam::SOCKET){
//             //     // outpostPoseDataFrame.x = armor.x;
//             //     // outpostPoseDataFrame.y = armor.y;
//             //     // outpostPoseDataFrame.z = armor.z;
//             //     // outpostPoseDataFrame.pitch = SerialParam::send_data.pitch;
//             //     outpostPoseDataFrame.yaw = armor.angle;
//             //     // outpostPoseDataFrame.roll = 0;
//             //     udpsender->send(outpostPoseDataFrame);
//             // }

//             DLOG(INFO) << "                                           right clicked: " << right_clicked;
//             DLOG(INFO) << "                                           angle: " << yaw;
//             DLOG(INFO) << "                                           send yaw: " << SerialParam::send_data.yaw << " send pitch: " << SerialParam::send_data.pitch;
//             DLOG(INFO) << "                                           recv yaw: " << SerialParam::recv_data.yaw << "  recv pitch: " << SerialParam::recv_data.pitch;
//             DLOG(INFO) << "                                           x: " << -armor.x << " y: " << -armor.y << " z: " << armor.z;
//             // 更新跟踪的目标
//             last_armor = armor;
//             last_right_clicked = right_clicked;
//         }
//         DLOG(INFO) << "target class: " << armor._class;
//     }

//     /**
//      * @brief 前哨站模式
//      *
//      * @param armors
//      * @param delta_t
//      * @param imu_data
//      * @param SerialPort_
//      */
//     void PoseSolver::outpostMode(vector<ArmorBlob> &armors, double delta_t, const SerialPortData &imu_data, SerialPort *SerialPort_)
//     {
//         if (armors.size() < 1)
//             return;

//         for (ArmorBlob &a : armors)
//             solveArmor(a, imu_data);

//         vector<ArmorBlob> candidates;

//         DLOG(INFO) << "state: OUTPOST";

//         for (auto &a : armors)
//         {
//             if (a._class == 7)
//             {
//                 a.x *= 6.2 / 5;
//                 a.y *= 6.2 / 5;
//                 a.z *= 6.2 / 5;
//                 candidates.push_back(a);
//             }
//         }

//         if (candidates.size() < 1)
//         {
//             last_armor = armors.at(0);
//             return;
//         }

//         right_clicked = imu_data.right_clicked;
//         if (last_right_clicked == 0 && right_clicked == 1)
//             first = true;
//         if (first)
//         {
//             outpost.clear();
//             first = false;
//         }
//         last_right_clicked = right_clicked;
//         DLOG(INFO) << " right clicked: " << right_clicked;

//         for (const auto &a : candidates)
//         {
//             outpost.update({a.x, a.y, a.z});
//         }

//         sort(candidates.begin(), candidates.end(), [&](const ArmorBlob &a, const ArmorBlob &b)
//              { return calcDiff(a, last_armor) < calcDiff(b, last_armor); });

//         armor = candidates.at(0);
//         center.x = armor.x;
//         center.y = armor.y;
//         center.z = armor.z;

//         // 更新roll
//         if (abs(imu_data.roll - roll.getMetric() / roll.getSize()) < 1000)
//         {
//             roll.update(imu_data.roll);
//         }

//         // 满足后，可以认为中心已经找到，然后就进行处理
//         if (outpost.getSize() == outpost.size())
//         {
//             // 估计前哨站中心
//             center = outpost.getMetric() / outpost.size();

//             double w = 0.4 * 360;
//             // 距离
//             double distance = sqrt(center.x * center.x + center.y * center.y + center.z * center.z);
//             // 发弹延时0.01
//             double time = distance / speed;
//             SerialParam::send_data.shootStatus = 0;

//             for (const auto &a : candidates)
//             {
//                 DLOG(INFO) << "center diff: " << abs(a.x - center.x);
//                 if (abs(a.x - center.x) < 0.02 && abs(a.y - center.y) < 0.05)
//                 {
//                     thread([this, a, time, w, SerialPort_]()
//                            {
//                         int sleep_time = (120/w-time)*1000 - 0.01;
//                         std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
//                         SerialParam::send_data.shootStatus = 1;
//                         SerialPort_->writeData(&SerialParam::send_data);
//                         DLOG(INFO) << "                shoot"; })
//                         .detach();
//                     break;
//                 }
//             }
//         }

//         DLOG(INFO) << "                                    center x: " << center.x << " y: " << center.y << " z: " << center.z << " roll: " << roll.getMetric() / roll.getSize();

//         if (abs(roll.getMetric() / roll.getSize()) > 500)
//         {
//             // TODO roll偏差较大，认为此时在斜坡上
//             // FIXME 调整弹道
//             double offset = predictor->fitTrajectory(center, speed, true);
//             double bias = 0;
//             SerialParam::send_data.yaw = predictor->getYaw(center, delta_t, speed, imu_data, false) + (offset + bias) * sin(roll.getMetric() / roll.getSize() / 100 / 180 * 3.14) - 300;
//             SerialParam::send_data.pitch = predictor->fitTrajectory(center, speed, false) - offset + (offset + bias) * cos(-roll.getMetric() / roll.getSize() / 100 / 180 * 3.14);
//         }
//         else
//         {
//             // 解算yaw轴的数据，false表示不需要预测
//             SerialParam::send_data.yaw = predictor->getYaw(center, delta_t, speed, imu_data, false);
//             // TODO 弹道补偿
//             // FIXME 下面有一个超参数50，重力补偿得到的
//             SerialParam::send_data.pitch = predictor->fitTrajectory(center, speed, false) + 50;
//         }

//         // 使用VOFA+ 进行udp通信调试
//         if (GlobalParam::SOCKET)
//         {
//             outpostPoseDataFrame.x = center.x;
//             outpostPoseDataFrame.y = center.y;
//             outpostPoseDataFrame.z = center.z;
//             outpostPoseDataFrame.pitch = SerialParam::send_data.pitch;
//             outpostPoseDataFrame.yaw = SerialParam::send_data.yaw;
//             outpostPoseDataFrame.roll = 0;
//             udpsender->send(outpostPoseDataFrame);
//         }

//         DLOG(INFO) << "                                           angle: " << yaw;
//         DLOG(INFO) << "                                           send yaw: " << SerialParam::send_data.yaw << " send pitch: " << SerialParam::send_data.pitch;
//         DLOG(INFO) << "                                           recv yaw: " << SerialParam::recv_data.yaw << "  recv pitch: " << SerialParam::recv_data.pitch << "  recv roll: " << imu_data.roll;
//         DLOG(INFO) << "                                           x: " << armor.x << " y: " << armor.y << " z: " << armor.z;
//         last_armor = armor;
//     }

//     // 不动哨兵，没有哨兵了
//     void PoseSolver::sentinelMode(vector<ArmorBlob> &armors, double delta_t, const SerialPortData &imu_data, SerialPort *SerialPort_)
//     {
//         if (armors.size() < 1)
//             return;
//         for (ArmorBlob &a : armors)
//             solveArmor(a, imu_data);
//         vector<ArmorBlob> candidates;
//         // DLOG(INFO) << "state: SENTINEL";
//         for (const auto &a : armors)
//         {
//             if (a._class == 6)
//             {
//                 candidates.push_back(a);
//             }
//         }
//         if (candidates.size() < 1)
//         {
//             // last_armor = armors.at(0);
//             armor = last_armor;
//             // return;
//         }
//         else
//             armor = candidates.at(0);

//         int mode = 1;
//         if (mode)
//         {
//             SerialParam::send_data.shootStatus = 1;
//             cur = Point3d(armor.x, armor.y, armor.z);
//             SerialParam::send_data.yaw = predictor->getYaw(cur, delta_t, speed, imu_data);
//             SerialParam::send_data.pitch = predictor->fitTrajectory(cur, speed);
//         }
//         else
//         { // 变速
//             // predictor->getYaw(armor, delta_t, speed, imu_data);
//             // predictor->fitTrajectory(armor, speed);
//             // const Mat& cur = predictor->getCurState();

//             // if(sentinel.size() >= sentinel_cnt){
//             //     sentinel[sentinel_id].x = armor.x;
//             //     sentinel[sentinel_id].y = armor.y;
//             //     sentinel[sentinel_id].z = armor.z;
//             // } else{
//             //     sentinel.push_back({armor.x, armor.y, armor.z});
//             // }
//             // sentinel_id = ((sentinel_id + 1) % sentinel_cnt + sentinel_cnt) % sentinel_cnt;
//             // DLOG(INFO) << "sentinel id: " << sentinel_id << " size: " << sentinel.size();
//             // x0 = 0; y0 = 0; z0 = 0;
//             // if(sentinel.size() == sentinel_cnt){
//             //     for(int i=0;i<sentinel_cnt;i++){
//             //         x0 += sentinel[i].x; y0 += sentinel[i].y; z0 += sentinel[i].z;
//             //     }
//             //     armor.x = x0/sentinel_cnt; armor.y = y0/sentinel_cnt; armor.z = z0/sentinel_cnt;
//             // }

//             // double distance = sqrt(armor.x*armor.x + armor.z*armor.z), v = 15;
//             // double x = cur.at<float>(0) +(distance/v)*cur.at<float>(2);
//             // double theta = atan(armor.y/distance);
//             // double delta_y;
//             // // R = 42.50 mm, m = 41 g
//             // double k1 = 0.47*1.169*(2*M_PI*0.02125*0.02125)/2/0.041;
//             // for(int i=0;i<100;i++){
//             //     double t = (pow(2.718281828, k1*distance)-1)/(k1*v*cos(theta));
//             //     delta_y = armor.y - v*sin(theta)*t/cos(theta) + 4.9 * t*t/cos(theta)/cos(theta);
//             //     // DLOG(INFO) << "delta_y: " << delta_y;
//             //     if(fabs(delta_y) < 0.001) break;
//             //     theta -= delta_y / (- (v*t) / pow(cos(theta), 2) + 9.8 * t * t / (v * v) * sin(theta) / pow(cos(theta), 3));
//             // }
//             // SerialParam::send_data.shootStatus = fabs(x - armor.x) < 0.02;
//             // SerialParam::send_data.yaw = (atan(armor.x/armor.z)/M_PI*180)*100;
//             // SerialParam::send_data.pitch = (theta/M_PI*180+2)*100;

//             // DLOG(INFO) << "abs: " << fabs(x - armor.x) << " x: " << x;
//             // DLOG(INFO) << "x: " << armor.x << " y: " << armor.y << " z: " << armor.z;
//         }

//         DLOG(INFO) << "                                           angle: " << yaw;
//         DLOG(INFO) << "                                           send yaw: " << SerialParam::send_data.yaw << " send pitch: " << SerialParam::send_data.pitch;
//         DLOG(INFO) << "                                           recv yaw: " << SerialParam::recv_data.yaw << "  recv pitch: " << SerialParam::recv_data.pitch << "  recv roll: " << SerialParam::recv_data.roll;
//         DLOG(INFO) << "                                           x: " << -armor.x << " y: " << -armor.y << " z: " << armor.z;
//         last_armor = armor;
//     }

//     float PoseSolver::calcDiff(const ArmorBlob &a, const ArmorBlob &b)
//     {
//         return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
//     }

//     void PoseSolver::clearCircle()
//     {
//         outpost.clear();
//     }

//     void PoseSolver::clearSentinel()
//     {
//         sentinel.clear();
//     }

//     int PoseSolver::chooseArmor(const vector<ArmorBlob> &armors)
//     {
//         bool has_same_class = false;
//         bool has_hero = false;
//         bool has_sentry = false;
//         bool has_engineer = false;
//         for (const auto &a : armors)
//         {
//             if (a._class == top_pri)
//                 return top_pri;
//             if (a._class == last_armor._class)
//                 has_same_class = true;
//             switch (a._class)
//             {
//             case 1:
//                 has_hero = true;
//                 break;
//             case 2:
//                 has_engineer = true;
//                 break;
//             case 3:
//             case 4:
//             case 5:
//                 has_sentry = true;
//                 break;
//             }
//         }
//         if (has_same_class)
//             return last_armor._class;
//         else if (has_hero)
//             return 1;
//         else if (has_sentry)
//         {
//             double d = 10;
//             int c = 0;
//             for (const auto &a : armors)
//             {
//                 if ((a._class == 3 || a._class == 4 || a._class == 5) && d > (a.x * a.x + a.z * a.z))
//                 {
//                     d = a.x * a.x + a.z * a.z;
//                     c = a._class;
//                 }
//             }
//             return c;
//         }
//         else if (has_engineer)
//             return 2;
//         return 0;
//     }

//     /**
//      * @brief 光束法平差 这一部分的作用，就是更新了装甲板的宽高，并且重新得到了装甲板在相机坐标系下的位姿
//      *
//      * @param a_b 装甲板的  宽/2 高/2
//      * @param points_2d 装甲板四个角点坐标
//      * @param K 相机内参矩阵
//      * @param R solvePnpGeneric得到的旋转矩阵
//      * @param t solvePnPGeneric得到的平移向量
//      * @return Eigen::Isometry3d
//      */
//     Eigen::Isometry3d PoseSolver::bundleAdjustment(Eigen::Vector2d &a_b, Eigen::Matrix<double, 4, 2> &points_2d, const Mat &K, Mat &R, Mat &t)
//     {
//         // 初始化g2o

//         // 位姿的维度为6，路标的维度为2
//         // pose维度为6，landmark维度为2
//         typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 2>> Block;                                               // pose 维度为 6, landmark 维度为 2
//         // linearsolver 使用 Eigen库实现矩阵求逆来解方程
//         std::unique_ptr<Block::LinearSolverType> linearSolver(new g2o::LinearSolverEigen<Block::PoseMatrixType>()); // 方程求解器
//         std::unique_ptr<Block> solver_ptr(new Block(std::move(linearSolver)));                                      // 矩阵块求解器
//         // 初始化一个求解器
//         g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
//         g2o::SparseOptimizer optimizer;
//         optimizer.setAlgorithm(solver);

//         // vertex
//         // 将四个点的坐标分别设置为g2o中的顶点，即为优化的变量，坐标变量为初始值
//         g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap();                // camera pose

//         Eigen::Matrix3d R_mat;
//         // 旋转矩阵
//         R_mat << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
//             R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
//             R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);

//         // pose 初始化
//         pose->setId(0);
//         // 旋转矩阵，平移向量组成一个Isometry3d对象
//         // 初始化相机位姿，旋转矩阵、平移向量
//         pose->setEstimate(g2o::SE3Quat(
//             R_mat,
//             Eigen::Vector3d(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0))));
//         optimizer.addVertex(pose);

//         // 定义右下角坐标 顶点并设置优化变量
//         VertexAB *ab = new VertexAB();
//         ab->setId(1);
//         ab->setEstimate(a_b);
//         ab->setMarginalized(true);
//         optimizer.addVertex(ab);

//         // OrthogonalEdge
//         // 用四个顶点的2D坐标和顶点a,b 在g2o中初始化一套OrthogonalEdge
//         // 用于约束每条射线在相机坐标系下的交点应该都位于四个三角形的内部，即最小化误差项
//         // 构造误差项：计算4个点到光心的射线与装甲板角点构成的三角形内部误差
//         OrthogonalEdge *edge_ = new OrthogonalEdge();
//         edge_->setId(1);
//         edge_->setVertex(0, ab);
//         edge_->setVertex(1, pose);
//         edge_->setMeasurement(points_2d);
//         edge_->setInformation(Eigen::Matrix<double, 8, 8>::Identity());
//         optimizer.addEdge(edge_);

//         // 优化器设置以及运行
//         std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
//         optimizer.setVerbose(true);
//         optimizer.initializeOptimization();
//         // 进行优化
//         optimizer.optimize(5);

//         // 记录优化所需要的时间
//         std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
//         std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
//         cout << "optimization costs time: " << time_used.count() << " seconds." << endl;

//         // 优化后的结果
//         cout << endl
//              << "after optimization:" << endl;
//         cout << "T=" << endl
//              << Eigen::Isometry3d(pose->estimate()).matrix() << endl;
//         cout << "a=" << ab->estimate().matrix()[0] << endl;
//         cout << "b=" << ab->estimate().matrix()[1] << endl;

//         // 优化后的结果： 宽/高 比例
//         float length_rate_height = ab->estimate().matrix()[1] / ab->estimate().matrix()[0];
//         // 优化后的高
//         float height_now = length_of_small * length_rate_height;

//         // 更新装甲板的高度
//         height_of_small = height_now;

//         // ba_points
//         std::vector<cv::Point3f>
//             ba_ponits{{-length_of_small, -height_now, 0}, {length_of_small, -height_now, 0}, {length_of_small, height_now, 0}, {-length_of_small, height_now, 0}};

//         // 将优化后的四点重新写回
//         // 覆盖掉优化后的
//         rewrite_3d_points(ba_ponits);

//         // 返回优化后的 相机 位姿
//         return Eigen::Isometry3d(pose->estimate());
//     }

//     /**
//      * @brief 覆盖掉原来的装甲板坐标系的坐标
//      *
//      * @param points_3d_after_ba 优化后的装甲板坐标系四个角点的坐标
//      */
//     void PoseSolver::rewrite_3d_points(std::vector<cv::Point3f> &points_3d_after_ba)
//     {
//         points_3d = points_3d_after_ba;
//     }
// }
