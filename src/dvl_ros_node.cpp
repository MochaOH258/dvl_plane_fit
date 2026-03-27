#include "pid.h"
#include "dvl_plane.h"
#include "sub.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <functional>
#include <memory>

class DvlRosNode : public rclcpp::Node
{
public:
    DvlRosNode();

private:
    void beam1_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void beam2_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void beam3_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void beam4_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    void control_step();

private:
    using clock = std::chrono::steady_clock;

    struct BeamData
    {
        double dist = 0.0;
        bool has_msg = false;
    };

    // --------------------------
    // ROS 接口
    // --------------------------
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr beam1_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr beam2_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr beam3_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr beam4_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // --------------------------
    // 输入缓存
    // --------------------------
    std::array<BeamData, 4> beams_{};
    std::mutex mtx_;
    double last_v_cmd_ = 0.0;

    // --------------------------
    // 时序
    // --------------------------
    double freq_ = 200.0;
    clock::time_point last_time_;

    // --------------------------
    // 控制对象（按值拥有）
    // 注意声明顺序不能乱：
    // 先 Beam，再 Plane，再 PID，再 Controller，再 SubInterface
    // --------------------------
    Beam b1_;
    Beam b2_;
    Beam b3_;
    Beam b4_;
    Plane plane_;

    PID yaw_pid_;
    PID dist_pid_;
    PID vel_pid_;

    Controller ctrl_;
    SubInterface sub_if_;
};

DvlRosNode::DvlRosNode()
: Node("dvl_ros_node"),
  last_time_(clock::now()),
  b1_( 0.955316618,  0.785398163),
  b2_( 0.955316618, -0.785398163),
  b3_(-0.955316618,  0.785398163),
  b4_(-0.955316618, -0.785398163),
  plane_(b1_, b2_, b3_, b4_),
  yaw_pid_(1.0, 0.0, 0.0, 1.0 / freq_),
  dist_pid_(1.0, 0.0, 0.0, 1.0 / freq_),
  vel_pid_(1.0, 0.0, 0.0, 1.0 / freq_),
  ctrl_(0.8, 0.0, 0.5, yaw_pid_, dist_pid_, vel_pid_, plane_),
  sub_if_(plane_, b1_, b2_, b3_, b4_, ctrl_)
{
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    beam1_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "beam1", 10, std::bind(&DvlRosNode::beam1_cb, this, std::placeholders::_1));

    beam2_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "beam2", 10, std::bind(&DvlRosNode::beam2_cb, this, std::placeholders::_1));

    beam3_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "beam3", 10, std::bind(&DvlRosNode::beam3_cb, this, std::placeholders::_1));

    beam4_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "beam4", 10, std::bind(&DvlRosNode::beam4_cb, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(5),
        std::bind(&DvlRosNode::control_step, this));
    
    RCLCPP_INFO(this->get_logger(), "dvl_ros_node started");
}

void DvlRosNode::beam1_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    std::scoped_lock lk(mtx_);
    if (!msg->ranges.empty())
    {
        beams_[0].dist = msg->ranges[0];
        beams_[0].has_msg = true;
    }
}

void DvlRosNode::beam2_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    std::scoped_lock lk(mtx_);
    if (!msg->ranges.empty())
    {
        beams_[1].dist = msg->ranges[0];
        beams_[1].has_msg = true;
    }
}

void DvlRosNode::beam3_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    std::scoped_lock lk(mtx_);
    if (!msg->ranges.empty())
    {
        beams_[2].dist = msg->ranges[0];
        beams_[2].has_msg = true;
    }
}

void DvlRosNode::beam4_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    std::scoped_lock lk(mtx_);
    if (!msg->ranges.empty())
    {
        beams_[3].dist = msg->ranges[0];
        beams_[3].has_msg = true;
    }
}

void DvlRosNode::control_step()
{

    auto now = clock::now();
    double dt = std::chrono::duration<double>(now - last_time_).count();
    last_time_ = now;

    if (dt <= 1e-6)
    {
        return;
    }

    yaw_pid_.set_dt(dt);
    dist_pid_.set_dt(dt);
    vel_pid_.set_dt(dt);

    std::array<double, 4> dist{};
    std::array<bool, 4> valid{};

    {
        std::scoped_lock lk(mtx_);
        for (int i = 0; i < 4; ++i)
        {
            dist[i] = beams_[i].dist;
            valid[i] = beams_[i].has_msg && std::isfinite(beams_[i].dist) && beams_[i].dist < 29.9;
        }
    }

    /* 
     RCLCPP_INFO(this->get_logger(),
            "beam: [%.3f %.3f %.3f %.3f] valid:[%d %d %d %d]",
            dist[0], dist[1], dist[2], dist[3],
            valid[0], valid[1], valid[2], valid[3]);
    */
   

    // 如果当前一束都没收到，直接不控制
    bool any_msg = false;
    for (bool v : valid)
    {
        if (v)
        {
            any_msg = true;
            break;
        }
    }
    if (!any_msg)
    {
        return;
    }

    double v_sway_meas = last_v_cmd_;

    std::array<double, 3> ruv_cmd = sub_if_.get_cmd(dist, valid, v_sway_meas);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = ruv_cmd[1];
    cmd.linear.y = ruv_cmd[2];
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = ruv_cmd[0];

    cmd_pub_->publish(cmd);
    last_v_cmd_ = ruv_cmd[2];
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DvlRosNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}