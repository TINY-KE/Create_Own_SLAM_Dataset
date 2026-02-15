#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

namespace gazebo {

class RobotPlugin : public ModelPlugin {
 public:
  RobotPlugin() {}

  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override {
    // 获取机器人模型
    model_ = _parent;

    // 初始化ROS节点
    ros::NodeHandle nh;

    // 创建一个ROS发布器，发布机器人的位置信息
    pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/robot_pose", 10);

    // 创建一个Gazebo回调函数，用于在每个仿真步骤中获取机器人的位姿信息
    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&RobotPlugin::OnUpdate, this));
  }

 private:
  void OnUpdate() {
    // 获取机器人的位姿信息
    auto pose = model_->WorldPose();

    // 将位姿信息转换为ROS消息类型
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = "world";
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.pose.position.x = pose.Pos().X();
    pose_msg.pose.position.y = pose.Pos().Y();
    pose_msg.pose.position.z = pose.Pos().Z();
    pose_msg.pose.orientation.x = pose.Rot().X();
    pose_msg.pose.orientation.y = pose.Rot().Y();
    pose_msg.pose.orientation.z = pose.Rot().Z();
    pose_msg.pose.orientation.w = pose.Rot().W();

    // 发布机器人的位姿信息
    pose_pub_.publish(pose_msg);
  }

  // 机器人模型
  physics::ModelPtr model_;

  // ROS发布器，用于发布机器人的位置信息
  ros::Publisher pose_pub_;

  // Gazebo回调函数，用于在每个仿真步骤中获取机器人的位姿信息
  gazebo::event::ConnectionPtr update_connection_;
};

GZ_REGISTER_MODEL_PLUGIN(RobotPlugin)

}  // namespace gazebo

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_publisher");
    
    return(0);
}


// ————————————————

//                             版权声明：本文为博主原创文章，遵循 CC 4.0 BY-SA 版权协议，转载请附上原文出处链接和本声明。
                        
// 原文链接：https://blog.csdn.net/Travis_X/article/details/130431652