#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <iostream>
#include <chrono>
#include <string> 
using namespace std;

//设置一些保存路径
string Path = "/home/robotlab/dataset/MySimDataset/gazebo_dataset_4/";
string pathRGB = Path + "rgb.txt";
string pathDepth = Path + "depth.txt";
string dataRGB = Path + "/rgb/";
string dataDepth = Path + "/depth/";
//设置图像topic名称
string depth_topic = "/depth_to_rgb/image_raw";
string rgb_topic = "/rgb/image_raw";

//保存图像路径至txt文件下
void savePath(string &path, string &type,double time, bool png=true){
   ofstream of;
   of.open(path, std::ios_base::app);
   if(of.fail()){
       ROS_INFO("Fail to opencv file!!");
   }else{
       of<<endl;
       if(png){
            of<<std::fixed<< time <<" "<< type <<time<<".png"; 
       }
       else{
            of<<std::fixed<< time <<" "<< type <<time<<".tiff"; 
       }
       of.close();
   }
}
//RGB图像回调函数
void GrabRGB(const sensor_msgs::ImageConstPtr& msg)
{
    std::cout<<"get rgb image"<<std::endl;
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
       //保存图像及路径
        cv_ptr = cv_bridge::toCvShare(msg);
        //颜色通道转换
        cv::Mat bgrImage;
        cvtColor(cv_ptr->image, bgrImage, CV_BGR2RGB);
        cv::imshow("RGB", bgrImage);
        cv::waitKey(1);
        double time = cv_ptr->header.stamp.toSec();
        string type = "rgb/";
        savePath(pathRGB, type, time);
        std::ostringstream osf;
        osf<< dataRGB <<std::fixed <<time << ".png";//图像以时间戳命名
        cv::imwrite(osf.str(), bgrImage);
        // cv::imwrite(osf.str(), cv_ptr->image);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
     //cv::waitKey(2);    
}

//深度图像回调函数
void GrabDepth(const sensor_msgs::ImageConstPtr& msg)
{
    std::cout<<"get deth image"<<std::endl;

    cv_bridge::CvImageConstPtr cv_depth_ptr;
    try
    {
        cv_depth_ptr = cv_bridge::toCvShare(msg);

        // 显示
        cv::Mat depth_normalized;
        cv::normalize(cv_depth_ptr->image, depth_normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        cv::imshow("Depth Image", depth_normalized);
        cv::waitKey(1);

        double time = cv_depth_ptr->header.stamp.toSec();
        string type = "depth/";
        

        // 将深度图像消息转换为 OpenCV 格式
        cv_bridge::CvImagePtr cv_ptr;
        
        if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
            savePath(pathDepth, type, time);

            // 对于 uint16 类型的深度图 (常见于 Kinect, RealSense)
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            
            std::ostringstream osf;
            osf<< dataDepth <<std::fixed  <<time << ".png";

            // 保存为 PNG 格式（保持16位精度）
            cv::imwrite(osf.str(), cv_ptr->image);
            ROS_INFO("Saved 16UC1 depth image ");
        }
        else if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            savePath(pathDepth, type, time, false);
            
            // 对于 float32 类型的深度图
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            
            std::ostringstream osf;
            osf<< dataDepth <<std::fixed  <<time << ".tiff";

            // 保存为 TIFF 格式（保持浮点精度）
            cv::imwrite(osf.str(), cv_ptr->image);
            ROS_INFO("Saved 32FC1 depth image ");
        }
        else {
            ROS_WARN("Unsupported depth image encoding", msg->encoding.c_str());
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
     //cv::waitKey(2);
}

//节点主函数
int main(int argc,char **argv)
{
    ros::init(argc,argv,"bagToTUM");
    ros::start();
    ofstream of;
    of.open(pathRGB, std::ios_base::app);
    if(of.fail()){
       ROS_INFO("Fail to opencv file!!");
    }else{
       of<<"#------------start a new dataset-----------------"; 
       of.close();
    }
    of.open(pathDepth, std::ios_base::app);
    if(of.fail()){
       ROS_INFO("Fail to opencv file!!");
    }else{
       of<<"#------------start a new dataset-----------------"; 
       of.close();
    }

    //订阅图像话题
    ROS_INFO("bagToTUM is ready.");
    ros::NodeHandle nodeHandler;
    ros::Subscriber subRGB = nodeHandler.subscribe(rgb_topic, 5, &GrabRGB);
    ros::Subscriber subDepth = nodeHandler.subscribe(depth_topic, 5, &GrabDepth);

    ros::spin();
    return 0;
}

