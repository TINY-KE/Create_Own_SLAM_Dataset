#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
// #include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/sensors/sensors.hh>
// #include <gazebo/math/gzmathc.hh>
// #include <gazebo/gui/gui.hh>
#include <ignition/math/Vector3.hh>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm>
// #include "Object.h"
#include <cmath>


#include <opencv2/core/core.hpp>

// ros::Publisher publisher_object;
using namespace std;
// using namespace ORB_SLAM2;


std::string WORK_SPACE_PATH = "";
std::string yamlfile_object = "";

int mam_loop = 1;
bool MotionIou_flag = true;
bool NoPara_flag = true;
bool ProIou_flag = true;
bool Ttest_flag = true;
bool iforest_flag = true;
bool little_mass_flag = true;
bool ProIou_only30_flag = true;


double angle_tune = 0;


// cv::Mat toCvMat(const Eigen::Matrix4d &m)
// {
//     cv::Mat cvMat(4,4,CV_32F);
//     for(int i=0;i<4;i++)
//         for(int j=0; j<4; j++)
//             cvMat.at<float>(i,j)=m(i,j);

//     return cvMat.clone();
// }

class object{

    public:
        object(){}
        object(std::string name_,
               double x_, double y_, double z_,
               double roll_, double pitch_, double yaw_,
               double l_, double w_, double h_ ):
                name(name_),
                x(x_), y(y_), z(z_),
                yaw(yaw_), pitch(pitch_), roll(roll_),
                length(l_), width(w_), height(h_)
                {
                    //计算pose_mat
                    // 计算旋转矩阵
                    Eigen::Matrix3d rotation_matrix;
                    rotation_matrix = Eigen::AngleAxisd(yaw + angle_tune, Eigen::Vector3d::UnitZ());
                                    // * Eigen::AngleAxisd(ob.pitch, Eigen::Vector3d::UnitY())
                                    // * Eigen::AngleAxisd(ob.roll, Eigen::Vector3d::UnitX());

                    // 坐标系变换矩阵
                    Eigen::Matrix4d T;
                    T   <<  1, 0, 0, x,  // 假设T为平移矩阵，将A坐标系原点(1, 1, 1)平移到B坐标系原点
                            0, 1, 0, y,
                            0, 0, 1, z,
                            0, 0, 0, 1;
                    T.block<3,3>(0,0) = rotation_matrix;

                    // pose_mat = toCvMat(T);
                }

        ~object(){}

    public:
        std::string name;
        int class_id;
        double x,y,z;
        double roll, pitch, yaw;
        double length, width, height;
        // cv::Mat pose_mat;

    public:

};

geometry_msgs::Point corner_to_marker(geometry_msgs::Point& p_o, object& ob){
    geometry_msgs::Point p_w;


    // 计算旋转矩阵
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = Eigen::AngleAxisd(ob.yaw + angle_tune, Eigen::Vector3d::UnitZ());
                    // * Eigen::AngleAxisd(ob.pitch, Eigen::Vector3d::UnitY())
                    // * Eigen::AngleAxisd(ob.roll, Eigen::Vector3d::UnitX());

    // 坐标系变换矩阵
    Eigen::Matrix4d T;
    T << 1, 0, 0, ob.x,  // 假设T为平移矩阵，将A坐标系原点(1, 1, 1)平移到B坐标系原点
        0, 1, 0, ob.y,
        0, 0, 1, ob.z,
        0, 0, 0, 1;
    T.block<3,3>(0,0) = rotation_matrix;


    // 坐标点P在A坐标系下的坐标
    double x_o = p_o.x;
    double y_o = p_o.y;
    double z_o = p_o.z;

    // 将点P从A坐标系变换到B坐标系
    Eigen::Vector4d P_o(x_o, y_o, z_o, 1.0);  // 注意点P_a需要补一个1，才能与矩阵T相乘
    Eigen::Vector4d P_w = T * P_o;

    // 输出结果
    double x_w = P_w(0);
    double y_w = P_w(1);
    double z_w = P_w(2);
    // std::cout << "Point P in A coordinate system: (" << x_a << ", " << y_a << ", " << z_a << ")" << std::endl;
    // std::cout << "Point P in B coordinate system: (" << x_w << ", " << y_w << ", " << z_w << ")" << std::endl;

    p_w.x = x_w ;
    p_w.y = y_w ;
    p_w.z = z_w ;
    return p_w;
}

int main(int argc, char **argv)
{

    std::vector<object> obs;
    ros::init(argc, argv, "gazebo_world_parser");
    ros::NodeHandle nh;

    ros::Publisher publisher_object = nh.advertise<visualization_msgs::Marker>("objectmap_groudtruth", 1000);

    //当前文件路径
    std::string current_file_path = __FILE__;
    std::string current_folder_path = current_file_path.substr(0, current_file_path.find_last_of("/\\"));
    
    cout << "current_file_path = " << current_file_path << endl;

    WORK_SPACE_PATH = current_folder_path + "/" + "../";
    // yamlfile_object = "kinectv1.yaml";

    //gazebo world文件
    string gazebo_file = argv[1];

    if(argc != 2){
        gazebo_file = "/home/lj/Documents/codes/galbot_ws/src/sim_env/world/room_with_objects_on_ground.world";
        // std::cerr << "没有world文件" << std::endl;
    }
    else{
        gazebo_file = argv[1];
    }

    cout << "gazebo_file = " << gazebo_file << endl;


    // Load gazebo
    gazebo::setupServer(argc, argv);

    // Create a world and get the models
    gazebo::physics::WorldPtr world = gazebo::loadWorld(gazebo_file);//worlds/empty.world
    gazebo::physics::Model_V models = world->Models();


    // 真值文件的路径
    std::string groud_truth_path = current_folder_path + "/" + "groudtruth.txt";

    cout << endl << "Saving Objects to " << groud_truth_path << " ..." << endl;

    ofstream file_gt;
    file_gt.open(groud_truth_path.c_str());
    file_gt << fixed;

    // // Loop through each model and get its pose and size
    int h = 0 ;
    cout << models.size() << " models " << endl;

    file_gt << "hij:" << ", name" << ", x " << ", y" << " ,z " \
                                  << ", roll" << " ,pitch" << " ,yaw" \
                                  << ", length" << " ,width" << " ,height=" << std::endl;
    
    for(auto model : models) {
        h++;
        // Get model pose
        ignition::math::Pose3d pose = model->WorldPose();
        double x = pose.Pos().X();
        double y = pose.Pos().Y();
        double z = pose.Pos().Z();

        double roll = pose.Rot().Roll();
        double pitch = pose.Rot().Pitch();
        double yaw = pose.Rot().Yaw();

        double width, length, height;
        gazebo::physics::Link_V links = model->GetLinks();

        vector<std::string> keywordsFilt = {"wall", "box", "ground", "move_base"};

        int i = 0;
        auto link = links[0];  // 据观察，links只需要读取第一个
        // for (auto& link: links)
        {
            i ++;
            gazebo::physics::Collision_V collisions = link->GetCollisions();
            int j = 0;
            auto collision = collisions[0]; //据观察，collisions只包含一个
            // for (auto& collision: collisions)
            {
                j++;

                string model_name = model->GetName();

                bool to_filt = false;

                for (auto &keyword: keywordsFilt){
                    if(model_name.find(keyword)!=std::string::npos){
                        to_filt = true;
                        break;
                    }
                }

                if(to_filt){
                    continue;
                }

                ignition::math::AxisAlignedBox box = collision->BoundingBox();

                width = box.XLength();
                length = box.YLength();
                height = box.ZLength();

                // Print model information
                file_gt << h << i << j << ", ";
                file_gt << model_name << ", ";
                file_gt << x << ", " << y << ", " << z << ", ";
                file_gt << roll << ", " << pitch << ", " << yaw << ", ";
                file_gt << length << ", " << width << ", " << height << std::endl;

                object ob(model_name, x, y, z , roll, pitch, yaw, width, length, height);
                obs.push_back(ob);
            }
        }
        file_gt << std::endl;
    }

    file_gt.close();
    cout << endl << "Saved Objects End " << endl;

    gazebo::shutdown();

    ros::Rate rate(10);
    while (nh.ok()){
        int id = 0;
        for(auto ob: obs) {
            if( ob.name == "ground_plane"){
                // std::cout << "ground"<< std::endl;
                continue;
            }
            id++;
            
            //publish rviz 
            visualization_msgs::Marker marker;
            marker.id = id; //++object_id_init;//object_id_init + i;
            float mObject_Duration=1;
            // marker.lifetime = ros::Duration(mObject_Duration);
            marker.header.frame_id= "map";
            marker.header.stamp=ros::Time::now();
            marker.type = visualization_msgs::Marker::LINE_LIST; //LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.color.r = 255.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 1.0;
            marker.scale.x = 0.01;
            
            //     8------7
            //    /|     /|
            //   / |    / |
            //  5------6  |
            //  |  4---|--3
            //  | /    | /
            //  1------2
            // lenth ：corner_2[0] - corner_1[0]
            // width ：corner_2[1] - corner_3[1]
            // height：corner_2[2] - corner_6[2]

            double length_half = ob.length / 2.0;
            double width_half = ob.width / 2.0;
            double height_half = ob.height / 2.0;



            // double length_half = 0.6;  // y
            // double width_half = 0.3;  // x
            // double height_half = 0.15;  // z

            geometry_msgs::Point p1;   p1.x = -1*width_half;   p1.y = length_half;      p1.z = 0; //-1*height_half; 
            geometry_msgs::Point p2;   p2.x = -1*width_half;   p2.y = -1*length_half;   p2.z = 0; //-1*height_half; 
            geometry_msgs::Point p3;   p3.x = width_half;      p3.y = -1*length_half;   p3.z = 0; //-1*height_half; 
            geometry_msgs::Point p4;   p4.x = width_half;      p4.y = length_half;      p4.z = 0; //-1*height_half; 
            geometry_msgs::Point p5;   p5.x = -1*width_half;   p5.y = length_half;      p5.z = 2*height_half; 
            geometry_msgs::Point p6;   p6.x = -1*width_half;   p6.y = -1*length_half;   p6.z = 2*height_half; 
            geometry_msgs::Point p7;   p7.x = width_half;      p7.y = -1*length_half;   p7.z = 2*height_half; 
            geometry_msgs::Point p8;   p8.x = width_half;      p8.y = length_half;      p8.z = 2*height_half; 

            marker.points.push_back(corner_to_marker(p1, ob));
            marker.points.push_back(corner_to_marker(p2, ob));

            marker.points.push_back(corner_to_marker(p2, ob));
            marker.points.push_back(corner_to_marker(p3, ob));

            marker.points.push_back(corner_to_marker(p3, ob));
            marker.points.push_back(corner_to_marker(p4, ob));

            marker.points.push_back(corner_to_marker(p4, ob));
            marker.points.push_back(corner_to_marker(p1, ob));

            marker.points.push_back(corner_to_marker(p5, ob));
            marker.points.push_back(corner_to_marker(p1, ob));

            marker.points.push_back(corner_to_marker(p6, ob));
            marker.points.push_back(corner_to_marker(p2, ob));

            marker.points.push_back(corner_to_marker(p7, ob));
            marker.points.push_back(corner_to_marker(p3, ob));

            marker.points.push_back(corner_to_marker(p8, ob));
            marker.points.push_back(corner_to_marker(p4, ob));

            marker.points.push_back(corner_to_marker(p5, ob));
            marker.points.push_back(corner_to_marker(p6, ob));

            marker.points.push_back(corner_to_marker(p6, ob));
            marker.points.push_back(corner_to_marker(p7, ob));

            marker.points.push_back(corner_to_marker(p7, ob));
            marker.points.push_back(corner_to_marker(p8, ob));

            marker.points.push_back(corner_to_marker(p8, ob));
            marker.points.push_back(corner_to_marker(p5, ob));

            publisher_object.publish(marker);
            // std::cout << "publish rviz"<< std::endl;

            // publish rviz end
            // rate.sleep();

        }
        
    }

    return 0;

}
