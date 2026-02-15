# Gazebo数据采集方法
## 启动gezebo 环境
roslaunch SimDataset run_gazebo.launch 

## 键盘控制移动
 rosrun  SimDataset  teleop_turtle_key_my

按键	对应动作	物理意义
上 / 下箭头	前进 / 后退	沿当前朝向（Yaw）移动
左 / 右箭头	左转 / 右转	改变偏航角 (Yaw)
A / D	左移 / 右移	横向平移 (Strafe)
W / S	仰角 / 俯角	改变俯仰角 (Pitch)
E / Q	升高 / 降低	改变高度 (Z 轴)
空格 (Space)	记录节点	保存当前坐标到内存和文件（命名规则：cam_traj_年 - 月 - 日_时 - 分 - 秒.txt）
C	撤销	删除上一个记录点并返回
回车 (Enter)	演示轨迹	自动回放保存的路径（cam_traj.txt）

## 录制rosbag 
rosbag record /rgb/image_raw /depth_to_rgb/image_raw   -O image.bag 


## 将rostopic转为rgb/depth图片 
roslaunch SimDataset rostopic2picture.launch # 注意在luanch文件中修改根目录, 当前scale为5000




# kinect dk数据采集方法
## 启动kinect dk
roslaunch azure_kinect_ros_driver driver.launch    # Mode: NFOV_2X2BINNED, 720P; Topic: /rgb/image_raw /depth_to_rgb/image_raw

## 录制rosbag  
rosbag record  /rgb/image_raw /depth_to_rgb/image_raw

## 提取depth和rgb:
roslaunch SimDataset rostopic2picture.launch # 注意在luanch文件中修改根目录, 当前scale为5000

## 关联：
python2 associate.py rgb.txt depth.txt  > associations.txt
