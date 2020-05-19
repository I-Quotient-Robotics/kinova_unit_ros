# Kinova-Unit机械臂训练平台

## 前言

Kinova-Unit机械臂训练平台，基于Kinova公司的Gen2-6DOF轻质机械臂，Microsoft公司的Azure Kinect DK，以及intel公司的Realsense D435i定制开发完成。可用于物体识别抓取，机械臂规划等相关研究

## 训练平台结构

- 轻质铝型材可移动工作台
- kinova Gen2 6d 2f机械臂
- Kinect DK相机
- Realsense D435i相机
- Intel NUX 迷你计算机
![](https://github.com/I-Quotient-Robotics/kinova_unit_ros/blob/master/IMG/kinova_unit.jpg)

## Kinova-Unit平台软件安装

1. 新建Catkin工程文件夹，并下载相关的ROS包

   ```shell
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src
   git clone https://github.com/I-Quotient-Robotics/kinova_unit_ros.git
   git clone https://github.com/Kinovarobotics/kinova-ros.git
   git clone https://github.com/microsoft/Azure_Kinect_ROS_Driver.git
   git clone https://github.com/IntelRealSense/realsense-ros.git
   git clone https://github.com/IFL-CAMP/easy_handeye.git
   git clone https://github.com/pal-robotics/aruco_ros.git
   ```

2. 安装相关依赖程序

   ```shell
   cd ~/catkin_ws
   rosdep install --from-paths src --ignore-src -r -y
   pip2 install math3d
   ```

   注：Kinect DK，以及realsense需要安装驱动，详细参考官方库教程

3. 增加串口权限配置文件

   ```shell
   cd ~/catkin_ws/src/kinova_ros/kinova_driver/udev/
   sudo cp 10-kinova-arm.rules /etc/udev/rules.d
   sudo service udev reload
   sudo service udev restart
   sudo udevadm trigger
   ```

4. 编译，并索引ROS包

   ```shell
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

## 快速启动

1. 接通平台的电源

2. 启动计算机

3. 将机械臂摆放在一个合适的姿势下，启动机械臂的电源

4. 长按机械臂手柄上的`Home Position`（机械臂图标）按键，直至机械臂恢复到Home位置

5. 松开按钮后，机械臂启动完成

6. 启动机械臂训练单元的驱动

   ```shell
   roslaunch kinova_unit_bringup bringup.launch
   ```

7. 启动MoveIt节点，用于机械臂运动规划和控制

   ```shell
   roslaunch kinova_unit_moveit_config execute.launch
   ```

   **注意**：手腕关节可360度无限制旋转，若规划过程中同方向多次旋转，会拉紧realsense传输线，为避免出现在moveit中加入关节限制，机械臂回归home位置，查看joint_states中关节位置，其中kinova_joint_6应该在1-2，若超出范围，使用手柄旋转到范围内

   ```shell
   rostopic echo /joint_states
   ```

8. 物体模型存储

   ```shell
   rosrun kinova_unit_app object_save
   ```

9. 将物体放置在桌面，启动物体识别、机械臂抓取节点程序

   ```shell
   roslaunch kinova_unit_app kinova_unit_app.launch
   ```

10. 完成抓取后，可以在实验平台上重新放置物体，程序重新执行抓取

## 设备关闭

1. 关闭所有开启的程序
2. 关闭电脑
3. 使用手控器，将机械臂收回至平台投影范围内
4. 手把持住机械臂末端，按下机械臂基座的开关电源，关闭机械臂
5. 缓慢的将机械臂摆放在平台上，确保夹爪未受到挤压
6. 拔掉平台的电源

## 相机标定

测试相机点云信息是否准确，超出误差范围可尝试对相机位置重新标定，关于标定相关细节操作可参考相关教程。标定分为两部分

- 腕部相机（Realsense）和机械臂相对位置的标定
- 顶部固定相机（Kinect DK）和机械臂相对位置的标定

### 手眼相机标定

1. 打印`标定纸-1`，注意打印的时候不要进行缩放

2. 移除平台上的杂物，将标定纸平整的摆放在机械臂前方

3. 启动标定程序

   ```powershell
   roslaunch kinova_unit_app eye_on_hand.launch
   ```

4. 标定采样计算界面

![](https://github.com/I-Quotient-Robotics/kinova_unit_ros/blob/master/IMG/eye-on-hand.png)

5. moveit控制界面，控制机械臂围绕标定纸从各个方向进行拍照

![](https://github.com/I-Quotient-Robotics/kinova_unit_ros/blob/master/IMG/pose_plan.png)

7. 标定软件使用流程：

   （1）将机械臂移动到marker视角，点击**Check starting pose**，显示ready表示机械臂初始姿态可用

   （2）点击**Next Pose**，回到上一个位置，并准备移动到下一个位置

   （3）点击**Plan**，机械臂轨迹规划

   （4）点击**Excute**，执行规划移动到下一个位置

   （5）打开**Plugins/image_view**，选择发布result的topic，观察视角是否能识别到marker，不能识别则移动		  机械臂到合适位置

   （6）每次移动到新的位置后，确认能够识别到marker，点击**Take Simple**，取样多次后进行**compute**，最		  后**save**

   （7）转换坐标的yaml文件保存到了**~/.ros/easy-handeye**文件中，若标定误差较大，可手动调整

8. 完成标定后，在快速启动时，增加如下指令，发布标定后的坐标，替换URDF中的坐标位置

   ```shell
   roslaunch kinova_unit_app publish.launch
   ```

### 固定相机标定

1. 打印`标定纸-2`，注意打印的时候不要进行缩放

2. 移除平台上的杂物，并将标定纸粘贴在机械臂的末端

3. 启动标定程序

   ```powershell
   roslaunch kinova_unit_app eye_on_base.launch
   ```

4. 标定采样计算界面

![](https://github.com/I-Quotient-Robotics/kinova_unit_ros/blob/master/IMG/calibration.png)

5. 使用控制界面，控制机械臂围绕相机进行旋转，相机对二维码从各个方向进行拍照

6. 参考标定软件使用流程，完成标定

7. 完成标定后，在快速启动时，增加如下指令，发布标定后的坐标，替换URDF中的坐标位置

   ```shell
   roslaunch kinova_unit_app publish.launch
   ```

## 多物体位姿估计

使用pcl点云库实现计算物体上表面中心，recognition模块识别物体六自由度位姿估计

### 物体模型文件保存

1. 将功能台清理出空白区域作为点云处理区域，将待抓取物体放置于区域内

2. 启动驱动，运行模型文件保存程序；只支持单个物体的保存，多个物体分多次操作

3. 将待识别模型文件保存至**kinova_unit_app/pcd**目录中，并命名为object1..,4，无顺序要求

   ```shell
   # 使用pcl工具观察模型质量
   pcl_viewer object1.pcd
   ```

### 姿态估计

通过计算物体上表面中心，配准得到旋转姿态，进而转换为物体的位姿，以完成抓取

#### 流程图

![](https://github.com/I-Quotient-Robotics/kinova_unit_ros/blob/master/IMG/object_recognition.png)

#### 功能修改

##### 姿态估计

修改**kinova_unit_app/src/object_recognition_verification.cpp**文件参数

1.  点云处理区域

   修改坐标系下过滤区域，注意坐标系是**depth_camera_link**

   ```c++
     // Create the filtering object
     pcl::PassThrough<pcl::PointXYZRGB> pass_x, pass_y, pass_z;
     pass_x.setInputCloud (xyzCloudPtr);
     pass_x.setFilterFieldName ("x");
     pass_x.setFilterLimits (-0.2, 0.4);
     pass_x.filter (*xyzCloudPtrFiltered);
   
     pass_y.setInputCloud (xyzCloudPtrFiltered);
     pass_y.setFilterFieldName ("y");
     pass_y.setFilterLimits (-0.33, 0);
     pass_y.filter (*xyzCloudPtrFiltered);
   
     pass_z.setInputCloud (xyzCloudPtrFiltered);
     pass_z.setFilterFieldName ("z");
     pass_z.setFilterLimits (0.8, 1.015);
     pass_z.filter (*xyzCloudPtrFiltered);
   ```

2. 地面过滤

   修改地面过滤高度，此处是阀值

   ```c++
     seg1.setDistanceThreshold (0.022);
   ```

3. 聚类查找

   修改可作为聚类划分的点云数量阀值，通常分辨率不变的情况下可用于区分物体大小

   ```c++
     ec.setMinClusterSize (50);
     ec.setMaxClusterSize (25000);
   ```

4. recognition模块以及假设验证部分的整体调参，可参考pcl相关[教程](https://pcl-tutorials.readthedocs.io/en/master/global_hypothesis_verification.html#global-hypothesis-verification)

##### 物体抓取

修改**kinova_unit_app/script/object_detect.py**文件参数

1. 抓取位置的修改

   修改预抓取与抓取位置在z方向的距离

   ```python
   pose_pick_stanby.pose.position.z = pose_objects_detect.poses[x].position.z - 0.08
   pose_pick.pose.position.z = pose_objects_detect.poses[x].position.z + 0.01
   ```

2. 放置位置的修改

   - pose_place、pose_place_stanby为抓取后放置的位置，将机械臂移动到放置位置，查看机械臂姿态

     ```shell
     $ rostopic echo /j2n6s200_driver/out/tool_pose
     ```

   - 根据需要修改pose

     ```c++
     pose_place = geometry_msgs.msg.PoseStamped()
     pose_place.header.frame_id = "j2n6s200_link_base"
     pose_place.pose.position.x = -0.404268950224
     pose_place.pose.position.y = -0.457900762558
     pose_place.pose.position.z = 0.203838279247
     pose_place.pose.orientation.w = 0.0326578058302
     pose_place.pose.orientation.x =  0.98425257206
     pose_place.pose.orientation.y = -0.173656985164
     pose_place.pose.orientation.z = -0.00484579382464
     ```

   - 通常为了优化抓取流程，会加入预抓取位置，在pose_place的z方向预留一段距离

### 功能限制

1. 该功能使用一对一的模型配准，尽量保证物体摆放方式在保存与抓取时一致，可以提高配准概率
2. 采用遍历的方式对物体进行配准，实验中三个物体配准耗时在五秒左右，物体大小会影响时间
3. 配准概率在很大程度上以来参数，对于不同类型、大小的物体最优调参是不同的
4. 抓取位置依赖初始位姿，尽量保证物体模型位姿可以直接在中心点位置抓取
5. 使用异型结构的物体，保证有能明显区分姿态的特征结构
6. 点云存在噪声，桌面不完全在一个平面，物体太小会被作为桌面过滤掉，使用的物体大小应该适中

## 快速处置

| 问题                         | 检查方法                                                     |
| ---------------------------- | ------------------------------------------------------------ |
| 机械臂无法上电               | 检查平台电源是否正常<br />检查机械臂基座背后的电源连接线是否松动 |
| 彩色图像和深度图像有偏差     | Kinect或Realsense没有开启彩色-深度同步，参考官方ROS文档，开启同步 |
| 深度图像和实际物体位置有偏差 | 相机和机械臂之间的固联关系发生了偏移，参考《相机标定》章节，重新标定相机 |
|                              | 检查Kinect和Realsense的接线是否稳固，重新插拔后再尝试        |
| 图像模糊不清，有白斑         | 相机镜头污损，请用镜头纸清洁相机镜头                         |

## 注意事项

1. 机械臂无关节自锁装置，断电后，机械臂会因重力而自由下落。断电时，一定要手扶着机械臂，以免机械臂自由下落砸伤
2. 机械臂的手指比较脆弱，使用时，注意勿将手抓的手指戳在物体或者平台上，避免损坏
3. 相机标定完成后，勿随意拆装相机、相机架以及机械臂底座

## 参考链接

1. ROS tutorial: http://wiki.ros.org/
2. Kinova ROS driver：https://github.com/Kinovarobotics/kinova-ros
3. Kinect DK ROS driver：https://github.com/microsoft/Azure_Kinect_ROS_Driver
5. easy_handeye：https://github.com/IFL-CAMP/easy_handeye.git
5. aruco_ros：https://github.com/pal-robotics/aruco_ros.git
6. RealsenseD435i：https://github.com/IntelRealSense/realsense-ros.git
7. ROS MoveIt tutorial：http://docs.ros.org/kinetic/api/moveit_tutorials/html/
8. PCL documents：http://www.pointclouds.org/documentation/
