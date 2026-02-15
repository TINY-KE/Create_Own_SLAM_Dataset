#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>
#include <math.h>
#include <assert.h>
#include <iostream>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/tf.h>
#include "tf/transform_datatypes.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gazebo_set_states_publisher");
 
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
      
    double pitch = 28.0;
    gazebo_msgs::SetModelState objstate;
    
    double distance = 2.0;
    double height = 1.0;
    double angle = 0;
    double center_x = distance;
    double center_y = 0;
    objstate.request.model_state.model_name = "mobile_base";//"acircles_pattern_0"  mobile_base;
    // objstate.request.model_state.model_name = "chair_2";//"acircles_pattern_0"  mobile_base;
    objstate.request.model_state.pose.position.x = distance*cos(angle) - center_x;
    objstate.request.model_state.pose.position.y = distance*sin(angle) - center_y;
    objstate.request.model_state.pose.position.z = height;
    tf::Quaternion q_init = tf::createQuaternionFromRPY(0, pitch*M_PI/180.0, angle+M_PI);
    objstate.request.model_state.pose.orientation.w = q_init.w();
    objstate.request.model_state.pose.orientation.x = q_init.x();
    objstate.request.model_state.pose.orientation.y = q_init.y();
    objstate.request.model_state.pose.orientation.z = q_init.z();
    objstate.request.model_state.twist.linear.x = 0.0;
    objstate.request.model_state.twist.linear.y = 0.0;
    objstate.request.model_state.twist.linear.z = 0.0;
    objstate.request.model_state.twist.angular.x = 0.0;
    objstate.request.model_state.twist.angular.y = 0.0;
    objstate.request.model_state.twist.angular.z = 0.0;
    objstate.request.model_state.reference_frame = "world";
    std::cout<<"camerabody pose x:"<<objstate.request.model_state.pose.position.x
                <<",y:"<<objstate.request.model_state.pose.position.y
                <<",z:"<<objstate.request.model_state.pose.position.z
                <<",qw:"<<q_init.w()
                <<",qx:"<<q_init.x()
                <<",qy:"<<q_init.y()
                <<",qz:"<<q_init.z()
                <<std::endl;
    client.call(objstate);
    Eigen::Quaterniond q_world_to_camerabody;
    q_world_to_camerabody.w() = q_init.w();
    q_world_to_camerabody.x() = q_init.x();
    q_world_to_camerabody.y() = q_init.y();
    q_world_to_camerabody.z() = q_init.z();
    Eigen::Matrix3d R_world_to_camerabody;//声明一个Eigen类的3*3的旋转矩阵
    R_world_to_camerabody = q_world_to_camerabody.normalized().toRotationMatrix(); //四元数转为旋转矩阵--先归一化再转为旋转矩阵
    Eigen::Matrix3d R_camerabody_to_cam;
    R_camerabody_to_cam<< 0, 0, 1, 
                    -1, 0, 0, 
                    0, -1, 0;
    Eigen::Quaterniond q_world_to_camera = Eigen::Quaterniond ((R_world_to_camerabody * R_camerabody_to_cam )); //.inverse()  R_z_f90 * R_x_f90
    std::cout<<"camera pose"
                <<",qw:"<<q_world_to_camera.w()
                <<",qx:"<<q_world_to_camera.x()
                <<",qy:"<<q_world_to_camera.y()
                <<",qz:"<<q_world_to_camera.z()
                <<std::endl<<std::endl;

    std::cout<<"INIT POSE ----------------------"<<std::endl;
    std::cout<<"按任意键继续 ----------------------"<<std::endl;
    double cout = 0;

    ros::Rate loop_rate(1000);
    std::cin.get();
    while(ros::ok()) {
        angle = cout/180.0*M_PI ;
        double x = distance*cos(angle);
        double y = distance*sin(angle);
        double z = height;
        double mean_x=0, mean_y=0;
        //cv::Mat view = (cv::Mat_<float>(3, 1) << mean_x-x, mean_y-y, 0);
        //double angle = atan( (mean_y-y)/(mean_x-x) );
        //if( (mean_x-x)<0 && (mean_y-y)>0 )
        //    angle = angle +  M_PI;
        //if( (mean_x-x)<0 && (mean_y-y)<0 )
        //    angle = angle -  M_PI;
        //// Eigen::AngleAxisd rotation_vector (angle, Eigen::Vector3d(0,0,1));
        //// Eigen::Matrix3d rotation_matrix = rotation_vector.toRotationMatrix();
        //// cv::Mat rotate_mat = Converter::toCvMat(rotation_matrix);
        tf::Quaternion q = tf::createQuaternionFromRPY(0, pitch/180.0*M_PI, angle+M_PI);

        objstate.request.model_state.pose.position.x = x - center_x;
        objstate.request.model_state.pose.position.y = y - center_y;
        objstate.request.model_state.pose.position.z = z;
        objstate.request.model_state.pose.orientation.w = q.w();
        objstate.request.model_state.pose.orientation.x = q.x();
        objstate.request.model_state.pose.orientation.y = q.y();
        objstate.request.model_state.pose.orientation.z = q.z();

        std::cout<<"pose x:"<<objstate.request.model_state.pose.position.x
                <<",y:"<<objstate.request.model_state.pose.position.y
                <<",z:"<<objstate.request.model_state.pose.position.z
                <<",qw:"<<q.w()
                <<",qx:"<<q.x()
                <<",qy:"<<q.y()
                <<",qz:"<<q.z()
                <<std::endl;
        client.call(objstate);
        ROS_INFO("call service");

        cout += 0.01;
        ros::spinOnce();
        // loop_rate.sleep(); // is stuck on loop rate, reboot roscore
        //ROS_INFO("loop_rate sleep over");
    }
    ROS_INFO("end service");
    return 0;
}
