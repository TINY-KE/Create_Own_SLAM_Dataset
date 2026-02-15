
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"

#include <boost/filesystem.hpp>

#include <deque>
#include <fstream>
#include <string>

using namespace std;

bool create_if_not_exist(const std::string file_path)
{
  if (!boost::filesystem::exists(file_path)) {
    if (!boost::filesystem::create_directories(file_path)) {
      ROS_ERROR("Failed to create directory: %s", file_path.c_str());
      return false;
    }
  }
  return true;
}

class SaveDataNode {
public:
  SaveDataNode() : nh_("~") {
    // Subscribe to RGB and depth image topics
    rgb_sub_ = nh_.subscribe("/rgb/image_raw", 200, &SaveDataNode::rgbCallback, this);
    depth_sub_ = nh_.subscribe("/depth_to_rgb/image_raw", 200, &SaveDataNode::depthCallback, this);
    save_path = "/home/robotlab/ws_3d_vp/src/sim_env/src_save_image/save/";

  }


  // RGB image callback
  void rgbCallback(const sensor_msgs::Image::ConstPtr& msg) {
    // last_rgb_ = msg;
    
    int sec = msg->header.stamp.sec;
    int nsec = msg->header.stamp.nsec;

    std::string img_name = std::to_string(sec) + "." + std::to_string(nsec) + ".png";
    saveRgbImage(save_path + "/rgb/" + img_name, msg);
  }

  // Depth image callback
  void depthCallback(const sensor_msgs::Image::ConstPtr& msg) {
    // last_depth_ = msg;
    int sec = msg->header.stamp.sec;
    int nsec = msg->header.stamp.nsec;

    std::string img_name = std::to_string(sec) + "." + std::to_string(nsec) + ".png";
    saveDepthImage(save_path + "/depth/" + img_name, msg);
  }

  // Save rgb image to file
  void saveRgbImage(const std::string& filename, const sensor_msgs::Image::ConstPtr& img_msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGRA8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::imwrite(filename, cv_ptr->image);
  }

  // Save depth image to file
  void saveDepthImage(const std::string& filename, const sensor_msgs::Image::ConstPtr& img_msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::imwrite(filename, cv_ptr->image);
  }

private:
  ros::NodeHandle nh_;
  // ros::ServiceServer service_;
  ros::Subscriber rgb_sub_;
  ros::Subscriber depth_sub_;
//   ros::Subscriber imu_sub_;
  sensor_msgs::Image::ConstPtr last_rgb_;
  bool isSavingNow = false;
  int img_cnt, save_num;
  std::string save_path;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "get_images_server");
  SaveDataNode save_data_node;
  ros::spin();
  return 0;
}