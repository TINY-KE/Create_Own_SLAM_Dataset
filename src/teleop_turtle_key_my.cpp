/**
 * 创建时间：2024.1.13
 * 功能：使用键盘移动指定名称model，一般用于kinect相机
 * 使用方法：
 *    1. 修改 初始化值 和 model_name
 *    2. 上-前进，下-后退，左-左转，右-右转，w-相机仰，s-相机俯，a-向左平移，d-向右平移
 *    3. 空格-保存当前位置作为节点，Enter-演示目前所有节点对应的轨迹，c-删除当前节点并返回上一节点
*/


// 参考博客：https://www.jianshu.com/p/9c6adc3aeb02

#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

// #include <rclcpp/rclcpp.hpp>

#include <gazebo_msgs/SetModelState.h>
#include <math.h>
#include <assert.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/tf.h>
#include "tf/transform_datatypes.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <iostream>

#include <fstream>

#include <unistd.h>

using namespace std;
// using namespace Eigen;

// a-61, b-62, c-63, d-64, e-65, f-66, g-67
// h-68, i-69, j-6A, k-6B, l-6C, m-6D, n-6E
// o-6F, p-70, q-71, r-72, s-73, t-74
// u-75, v-76, w-77, x-78, y-79, z-7A

#define KEYCODE_RIGHT 0x43 
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
// #define KEYCODE_Esc 0x1B

#define KEYCODE_w 0x77  // w, to speed up linear velocity
#define KEYCODE_s 0x73  // s, to speed down linear velocity
#define KEYCODE_a 0x61  // a, to speed up angular velocity
#define KEYCODE_d 0x64  // d, to speed down angular velocity

#define KEYCODE_e 0x65  // e, to 升高相机高度
#define KEYCODE_q 0x71  // q, to 降低相机高度

#define KEYCODE_SPACE 0x20  // to save traj node
#define KEYCODE_c 0x63  // clear traj node
#define KEYCODE_ENTER 0x0A  // to show traj


// typedef Eigen::Vector<double, 6> Vector6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

class TeleopTurtle  //声明TeleopTurle类
{
public:
  TeleopTurtle();  //构造函数
  void keyLoop();

private:
  void client_call(bool print_state = false);
  void load_traj_node();
  void save_traj_node();
  void clear_current_node();
  void set_state(Vector6d& state);
  void show_traj();
  ros::NodeHandle nh_;
  double v_linear, v_angular, l_scale_, a_scale_;
  double h_linear;
  double x, y, z, roll, pitch, yaw;

  ros::Time start_time;

  string model_name;
  
  string traj_save_path = "cam_traj.txt";
  ros::Publisher vel_pub_;
  ros::ServiceClient client;

  vector<Vector6d, Eigen::aligned_allocator<Vector6d>> traj_node;

//   ros::Rate loop_rate(1000);

};

TeleopTurtle::TeleopTurtle():  //构造函数的定义，传递默认值
  x(1.8), y(-3), z(1), roll(0), pitch(0.), yaw(M_PI/2),
  model_name("mobile_base"),
  v_linear(0.01),
  v_angular(0.01/2),
  l_scale_(2.0),
  a_scale_(2.0),
  h_linear(0.01)
{
    nh_.param("scale_angular", a_scale_, a_scale_);  // param()和getParam()类似，但是允许指定参数的默认值
    // 参考：http://wiki.ros.org/cn/roscpp_tutorials/Tutorials/Parameters 
    nh_.param("scale_linear", l_scale_, l_scale_); // 同上

    client = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");


    Vector6d state;
    state << x,y,z,roll,pitch,yaw;
    traj_node.push_back(state);

    gazebo_msgs::SetModelState objstate;
    objstate.request.model_state.model_name = model_name;//"acircles_pattern_0"  mobile_base;
    // objstate.request.model_state.model_name = "mobile_base";//"acircles_pattern_0"  mobile_base;

    // 文件名称
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::stringstream time_ss;
    time_ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d_%H-%M-%S");// 格式化时间为字符串，例如：2025-09-23_14-30-45
    traj_save_path = "cam_traj_" + time_ss.str() + ".txt";

    client_call(true);

    load_traj_node();

// 注册一个话题，话题名为“turtle1/command_velocity”
//消息类型为：turtlesim::Velocity，消息队列大小为1
}

int kfd = 0;
struct termios cooked, raw;
// 定义两个termios结构体，跟Linux终端I/O相关，结构体中是对终端I/O的控制字段
//参看 https://www.jianshu.com/p/bb128a6b413e

void quit(int sig)  // 定义一个quit函数
{
  tcsetattr(kfd, TCSANOW, &cooked); 
//tcsetattr返回终端属性，中间参数表示可以指定在什么时候新的终端属性才起作用
//TCSANOW：更改立即发生
  ros::shutdown();  //关闭节点
  exit(0); // exit（0）：正常运行程序并退出程序 ,exit（1）：非正常运行导致退出程序；
// 参看：https://www.cnblogs.com/nufangrensheng/archive/2013/03/01/2938508.html
}

void TeleopTurtle::keyLoop()
{
    char c;
    bool dirty=false;

    // get the console in raw mode                                                              
    tcgetattr(kfd, &cooked);  // 参看：https://www.jianshu.com/p/bb128a6b413e 
    memcpy(&raw, &cooked, sizeof(struct termios)); // 将cooked中的内容拷贝到raw中
    raw.c_lflag &=~ (ICANON | ECHO); 
    //将ICANON和ECHO按位或“|” 之后取反，再将raw.c_lflag和其相与，将结果赋值给raw.c_lflag
    // Setting a new line, then end of file                         
    raw.c_cc[VEOL] = 1; 
    raw.c_cc[VEOF] = 2;
    //raw.c_cc是控制字符  https://www.gwduan.com/web/computer/history/unix-prog/terminal.html#special-char
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard"); // puts()函数用来向 标准输出设备 （屏幕）写字符串并换行
    puts("---------------------------");
    puts("Use arrow keys to move the turtle.");


    while(ros::ok())  // 循环一直执行 ，知道遇到退出语句
    {
        if(read(kfd, &c, 1) < 0)  
        {
        perror("read():"); 
        //　perror ( )用 来 将 上 一 个 函 数 发 生 错 误 的 原 因 输 出 到 标 准 设备 (stderr) 
        exit(-1);
        }

        // ROS_DEBUG("value: 0x%02X\n", c);
        // ROS_INFO("value: 0x%02X\n", c);
        
        // cout << "KEYCODE = " << c << endl;

        switch(c)  // 根据键盘的按键值，给相应的参数赋值
        {
        case KEYCODE_LEFT:
            ROS_DEBUG("LEFT");
            yaw += v_angular;
            dirty = true;
            save_traj_node();
            break;
        case KEYCODE_RIGHT:
            ROS_DEBUG("RIGHT");
            yaw -= v_angular;
            dirty = true;
            save_traj_node();
            break;
        case KEYCODE_UP:
            x += v_linear * cos(yaw);
            y += v_linear * sin(yaw);
            dirty = true;
            save_traj_node();
            break;
        case KEYCODE_DOWN:
            x -= v_linear * cos(yaw);
            y -= v_linear * sin(yaw);
            dirty = true;
            save_traj_node();
            break;
        case KEYCODE_a:
            x += v_linear * cos(yaw + M_PI / 2);
            y += v_linear * sin(yaw + M_PI / 2);
            dirty = true;
            save_traj_node();
            break;
        case KEYCODE_d:
            x -= v_linear * cos(yaw + M_PI / 2);
            y -= v_linear * sin(yaw + M_PI / 2);
            dirty = true;
            save_traj_node();
            break;
        case KEYCODE_w:
            ROS_DEBUG("UP");
            printf("front\n");
            pitch -= v_angular;
            dirty = true;
            save_traj_node();
            break;
        case KEYCODE_s:
            ROS_DEBUG("back");
            pitch += v_angular;
            dirty = true;
            save_traj_node();
            break;
        
        case KEYCODE_e:
            ROS_DEBUG("UP");
            printf("front\n");
            z += h_linear;
            dirty = true;
            save_traj_node();
            break;
        case KEYCODE_q:
            ROS_DEBUG("back");
            z -= h_linear;
            dirty = true;
            save_traj_node();
            break;

        case KEYCODE_SPACE:
            save_traj_node();
            break;
        case KEYCODE_c:
            clear_current_node();
            break;
        case KEYCODE_ENTER:
            show_traj();
            break;
        // case 
        
        }


        if(dirty ==true)
        {
            client_call(true);
            dirty=false;
            ros::spinOnce();
        }

    }
    return;
}

void TeleopTurtle::client_call(bool print_state){
    gazebo_msgs::SetModelState objstate;
    tf::Quaternion q = tf::createQuaternionFromRPY(0, pitch, yaw);
    objstate.request.model_state.model_name = model_name;//"acircles_pattern_0"  mobile_base;
    // objstate.request.model_state.model_name = "mobile_base";//"acircles_pattern_0"  mobile_base;
    objstate.request.model_state.pose.position.x = x;
    objstate.request.model_state.pose.position.y = y;
    objstate.request.model_state.pose.position.z = z;

    objstate.request.model_state.pose.orientation.w = q.w();
    objstate.request.model_state.pose.orientation.x = q.x();
    objstate.request.model_state.pose.orientation.y = q.y();
    objstate.request.model_state.pose.orientation.z = q.z();
    
    if (print_state) {
        // ROS_INFO("Camera Pose: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f", x,y,z,roll,pitch,yaw);
        printf("Camera Pose: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, v_linear: %.2f, v_angular: %.2f\n",\
                x,y,z,roll,pitch,yaw,v_linear, v_angular);
    }

    ros::Time current_time = ros::Time::now();
    float relate_time = current_time.toSec() - start_time.toSec();
    // ROS_INFO("Current time: %f", (current_time.toSec() - start_time.toSec()));
    
    printf("Time: %.6f, Camera Pose: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",\
                relate_time,x,y,z,q.x(),q.y(),q.z(),q.w());

    client.call(objstate);
}

void TeleopTurtle::load_traj_node(){
    ifstream f(traj_save_path.c_str());
    if (!f.is_open()) {
        std::cerr << "Cannot open file: " << traj_save_path << std::endl;
        return ;
    }
    traj_node.clear();
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty()) {
            cout << "Empty Line." << endl;
            break; // 跳过空行
        }
        std::istringstream iss(line);
        std::string word;
        std::vector<std::string> words;
        while (iss >> word) {
            words.push_back(word); // 将每个单词加入向量
        }

        if (words.size() != 6) {
            cout << "size != 6" << endl;
            break;
        }

        Vector6d state;
        
        state << std::stod(words[0]),
                 std::stod(words[1]),
                 std::stod(words[2]),
                 std::stod(words[3]),
                 std::stod(words[4]),
                 std::stod(words[5]);
        traj_node.push_back(state);
    }
    f.close();
}

void TeleopTurtle::save_traj_node(){
    Vector6d state;
    state << x,y,z,roll,pitch,yaw;
    traj_node.push_back(state);

    ofstream f;
    f.open(traj_save_path.c_str());
    f << fixed;

    for (auto &node: traj_node) {
        f << node.transpose().matrix() << endl;
    }

    printf("Save traj node\n");
}

void TeleopTurtle::clear_current_node()
{
    if (traj_node.size() == 1) {
        cout << "Only start node is left now." << endl;
        return;
    }

    traj_node.resize(traj_node.size() - 1);
    set_state(traj_node.back());
}

void TeleopTurtle::set_state(Vector6d& state)
{
    x = state(0);
    y = state(1);
    z = state(2);
    roll = state(3);
    pitch = state(4);
    yaw = state(5);

    client_call();
}

void TeleopTurtle::show_traj(){
    // start_time = ros::Time::now();
    // int split_num = 1000;
    // // set_state(traj_node[0]);
    // for(int i_node = 0; i_node < traj_node.size() - 1; i_node++){
    //     auto& state1 = traj_node[i_node], state2 = traj_node[i_node+1];
    //     for(int i_split = 0; i_split < split_num - 1; i_split++) {
    //         double w2 = (double)i_split / split_num;
    //         double w1 = 1.0 - w2;
    //         Vector6d state_mid = w1 * state1 + w2 * state2;
    //         set_state(state_mid);
    //         usleep(1000); // us
    //     }
    // }
    // set_state(traj_node.back());

    std::cout<<" Start read trajectory" <<std::endl;

    traj_node.clear();  // 清空原有数据

    string filename = "cam_traj.txt";
    std::ifstream f(filename.c_str());
    if (!f.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    double x, y, z, roll, pitch, yaw;
    while (f >> x >> y >> z >> roll >> pitch >> yaw) {
        Vector6d state;
        state << x, y, z, roll, pitch, yaw;
        traj_node.push_back(state);
    }

    f.close();
    std::cout << "Loaded " << traj_node.size() << " trajectory nodes from: " << filename << std::endl;

    std::cout<<" Start publish trajectory" <<std::endl;
    std::vector<double> timestamps;
    start_time = ros::Time::now();
    double motion_hz = 75;
    double sleep_time = 1000*1000 / motion_hz; // us, 200Hz
    for(int i_node = 0; i_node < traj_node.size() - 1; i_node++){
        auto& state = traj_node[i_node];
        set_state(state);

        double current_time = ros::Time::now().toSec();
        timestamps.push_back(current_time);
        // std::cout << std::fixed << std::setprecision(6) << current_time << std::endl;

        usleep(sleep_time); // us
    }
    set_state(traj_node.back());

    bool transport = false;
    if(transport){
        string filename_gt_tum = "gt_tum.txt";
        std::ofstream f_gt(filename_gt_tum.c_str());
        if (!f_gt.is_open()) {
            std::cerr << "Failed to open file: " << filename_gt_tum << std::endl;
            return;
        }
        // 将
        for(int i_node = 0; i_node < traj_node.size() - 1; i_node++){
            auto& state = traj_node[i_node];            
            double t_mvoe = timestamps[0]-11050.041000;
            double time = timestamps[i_node]-t_mvoe;
            double x = state(0);
            double y = state(1);
            double z = state(2);
            double roll = state(3);
            double pitch = state(4);
            double yaw = state(5);
            tf::Quaternion q = tf::createQuaternionFromRPY(0, pitch, yaw);

            // 写入到文件，格式：timestamp tx ty tz qx qy qz qw
            f_gt.precision(6); // 设置小数点精度
            f_gt << std::fixed << time << " "
                << x << " " << y << " " << z << " "
                << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
        }
        f_gt.close();
        std::cout << "GT_TUM Trajectory written to " << filename_gt_tum << std::endl;
    }
    std::cout<<" End read trajectory" <<std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle"); // 创建节点的第一步，初始化节点，节点名为："teleop_turtle"
  TeleopTurtle teleop_turtle;  // 定义一个类
  signal(SIGINT,quit);   // siganl 函数设置了某个信号的对于动作，再此是进入quit函数
  teleop_turtle.keyLoop(); // 调用类中的方法
  return(0);
}