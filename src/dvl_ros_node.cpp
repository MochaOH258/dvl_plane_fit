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
    void declare_and_load_parameters();
    void print_loaded_parameters() const;

    void beam1_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void beam2_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void beam3_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void beam4_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    void control_step();

private:
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
    rclcpp::Time last_time_;

    // --------------------------
    // 参数缓存
    // --------------------------
    bool use_sim_time_ = true;

    double desired_distance_ = 0.8;
    double desired_sway_speed_ = 0.5;
    double freq_ = 10.0;

    double b1_beta_ = 0.955316618;
    double b1_gamma_ = 0.785398163;
    double b2_beta_ = 0.955316618;
    double b2_gamma_ = -0.785398163;
    double b3_beta_ = -0.955316618;
    double b3_gamma_ = 0.785398163;
    double b4_beta_ = -0.955316618;
    double b4_gamma_ = -0.785398163;

    double max_residual_ = 40.0;

    double yaw_kp_ = 1.0;
    double yaw_ki_ = 0.0;
    double yaw_kd_ = 0.0;

    double surge_kp_ = 1.0;
    double surge_ki_ = 0.0;
    double surge_kd_ = 0.0;

    double sway_kp_ = 1.0;
    double sway_ki_ = 0.0;
    double sway_kd_ = 0.0;

    double max_u_ = 0.5;
    double max_v_ = 1.0;
    double max_r_ = 0.5;

    // --------------------------
    // 控制对象（动态创建）
    // --------------------------
    std::unique_ptr<Beam> b1_;
    std::unique_ptr<Beam> b2_;
    std::unique_ptr<Beam> b3_;
    std::unique_ptr<Beam> b4_;
    std::unique_ptr<Plane> plane_;

    std::unique_ptr<PID> yaw_pid_;
    std::unique_ptr<PID> dist_pid_;
    std::unique_ptr<PID> vel_pid_;

    std::unique_ptr<Controller> ctrl_;
    std::unique_ptr<SubInterface> sub_if_;
};

DvlRosNode::DvlRosNode()
: Node("dvl_ros_node")
{
    declare_and_load_parameters();
    print_loaded_parameters();

    if (freq_ <= 0.0)
    {
        RCLCPP_WARN(this->get_logger(), "freq <= 0.0, force set to 10.0");
        freq_ = 10.0;
    }

    // 先根据参数构造 Beam
    b1_ = std::make_unique<Beam>(b1_beta_, b1_gamma_);
    b2_ = std::make_unique<Beam>(b2_beta_, b2_gamma_);
    b3_ = std::make_unique<Beam>(b3_beta_, b3_gamma_);
    b4_ = std::make_unique<Beam>(b4_beta_, b4_gamma_);

    // Plane 构造函数：
    // Plane(Beam& Beam1, Beam& Beam2, Beam& Beam3, Beam& Beam4, double max_r)
    plane_ = std::make_unique<Plane>(*b1_, *b2_, *b3_, *b4_, max_residual_);

    // PID 构造函数：
    // PID(double p, double i, double d, double t, double m)
    yaw_pid_  = std::make_unique<PID>(yaw_kp_,   yaw_ki_,   yaw_kd_,   1.0 / freq_, max_r_);
    dist_pid_ = std::make_unique<PID>(surge_kp_, surge_ki_, surge_kd_, 1.0 / freq_, max_u_);
    vel_pid_  = std::make_unique<PID>(sway_kp_,  sway_ki_,  sway_kd_,  1.0 / freq_, max_v_);

    // 保持你当前 Controller 的参数顺序
    ctrl_ = std::make_unique<Controller>(
        desired_distance_,
        0.0,
        desired_sway_speed_,
        *yaw_pid_,
        *dist_pid_,
        *vel_pid_,
        *plane_);

    sub_if_ = std::make_unique<SubInterface>(
        *plane_, *b1_, *b2_, *b3_, *b4_, *ctrl_);

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    beam1_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "beam1", 10, std::bind(&DvlRosNode::beam1_cb, this, std::placeholders::_1));

    beam2_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "beam2", 10, std::bind(&DvlRosNode::beam2_cb, this, std::placeholders::_1));

    beam3_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "beam3", 10, std::bind(&DvlRosNode::beam3_cb, this, std::placeholders::_1));

    beam4_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "beam4", 10, std::bind(&DvlRosNode::beam4_cb, this, std::placeholders::_1));

    auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / freq_));

    timer_ = this->create_wall_timer(
        period_ns,
        std::bind(&DvlRosNode::control_step, this));

    last_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "dvl_ros_node started");
}

void DvlRosNode::declare_and_load_parameters()
{
    // 通用
    this->declare_parameter<bool>("use_sim_time", true);
    this->declare_parameter<double>("desired_distance", 0.8);
    this->declare_parameter<double>("desired_sway_speed", 0.5);
    this->declare_parameter<double>("freq", 10.0);

    // 波束安装角
    this->declare_parameter<double>("b1_beta", 0.955316618);
    this->declare_parameter<double>("b1_gamma", 0.785398163);
    this->declare_parameter<double>("b2_beta", 0.955316618);
    this->declare_parameter<double>("b2_gamma", -0.785398163);
    this->declare_parameter<double>("b3_beta", -0.955316618);
    this->declare_parameter<double>("b3_gamma", 0.785398163);
    this->declare_parameter<double>("b4_beta", -0.955316618);
    this->declare_parameter<double>("b4_gamma", -0.785398163);

    // 平面拟合阈值
    this->declare_parameter<double>("max_residual", 40.0);

    // PID 参数
    this->declare_parameter<double>("yaw_kp", 1.0);
    this->declare_parameter<double>("yaw_ki", 0.0);
    this->declare_parameter<double>("yaw_kd", 0.0);

    this->declare_parameter<double>("surge_kp", 1.0);
    this->declare_parameter<double>("surge_ki", 0.0);
    this->declare_parameter<double>("surge_kd", 0.0);

    this->declare_parameter<double>("sway_kp", 1.0);
    this->declare_parameter<double>("sway_ki", 0.0);
    this->declare_parameter<double>("sway_kd", 0.0);

    // 输出限幅
    this->declare_parameter<double>("max_u", 0.5);
    this->declare_parameter<double>("max_v", 1.0);
    this->declare_parameter<double>("max_r", 0.5);

    // 读取
    use_sim_time_       = this->get_parameter("use_sim_time").as_bool();

    desired_distance_   = this->get_parameter("desired_distance").as_double();
    desired_sway_speed_ = this->get_parameter("desired_sway_speed").as_double();
    freq_               = this->get_parameter("freq").as_double();

    b1_beta_  = this->get_parameter("b1_beta").as_double();
    b1_gamma_ = this->get_parameter("b1_gamma").as_double();
    b2_beta_  = this->get_parameter("b2_beta").as_double();
    b2_gamma_ = this->get_parameter("b2_gamma").as_double();
    b3_beta_  = this->get_parameter("b3_beta").as_double();
    b3_gamma_ = this->get_parameter("b3_gamma").as_double();
    b4_beta_  = this->get_parameter("b4_beta").as_double();
    b4_gamma_ = this->get_parameter("b4_gamma").as_double();

    max_residual_ = this->get_parameter("max_residual").as_double();

    yaw_kp_ = this->get_parameter("yaw_kp").as_double();
    yaw_ki_ = this->get_parameter("yaw_ki").as_double();
    yaw_kd_ = this->get_parameter("yaw_kd").as_double();

    surge_kp_ = this->get_parameter("surge_kp").as_double();
    surge_ki_ = this->get_parameter("surge_ki").as_double();
    surge_kd_ = this->get_parameter("surge_kd").as_double();

    sway_kp_ = this->get_parameter("sway_kp").as_double();
    sway_ki_ = this->get_parameter("sway_ki").as_double();
    sway_kd_ = this->get_parameter("sway_kd").as_double();

    max_u_ = this->get_parameter("max_u").as_double();
    max_v_ = this->get_parameter("max_v").as_double();
    max_r_ = this->get_parameter("max_r").as_double();
}

void DvlRosNode::print_loaded_parameters() const
{
    RCLCPP_INFO(this->get_logger(), "===== loaded params =====");
    RCLCPP_INFO(this->get_logger(), "use_sim_time = %s", use_sim_time_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "desired_distance = %.6f", desired_distance_);
    RCLCPP_INFO(this->get_logger(), "desired_sway_speed = %.6f", desired_sway_speed_);
    RCLCPP_INFO(this->get_logger(), "freq = %.6f", freq_);
    RCLCPP_INFO(this->get_logger(), "max_residual = %.6f", max_residual_);

    RCLCPP_INFO(this->get_logger(), "b1(beta, gamma) = (%.6f, %.6f)", b1_beta_, b1_gamma_);
    RCLCPP_INFO(this->get_logger(), "b2(beta, gamma) = (%.6f, %.6f)", b2_beta_, b2_gamma_);
    RCLCPP_INFO(this->get_logger(), "b3(beta, gamma) = (%.6f, %.6f)", b3_beta_, b3_gamma_);
    RCLCPP_INFO(this->get_logger(), "b4(beta, gamma) = (%.6f, %.6f)", b4_beta_, b4_gamma_);

    RCLCPP_INFO(this->get_logger(), "yaw pid   = (%.6f, %.6f, %.6f), max_r = %.6f",
                yaw_kp_, yaw_ki_, yaw_kd_, max_r_);
    RCLCPP_INFO(this->get_logger(), "surge pid = (%.6f, %.6f, %.6f), max_u = %.6f",
                surge_kp_, surge_ki_, surge_kd_, max_u_);
    RCLCPP_INFO(this->get_logger(), "sway pid  = (%.6f, %.6f, %.6f), max_v = %.6f",
                sway_kp_, sway_ki_, sway_kd_, max_v_);
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
    auto now = this->now();

    if (last_time_.nanoseconds() == 0)
    {
        last_time_ = now;
        return;
    }

    double dt = (now - last_time_).seconds();
    last_time_ = now;

    if (dt <= 1e-6)
    {
        return;
    }

    yaw_pid_->set_dt(dt);
    dist_pid_->set_dt(dt);
    vel_pid_->set_dt(dt);

    std::array<double, 4> dist{};
    std::array<bool, 4> valid{};

    {
        std::scoped_lock lk(mtx_);
        for (int i = 0; i < 4; ++i)
        {
            dist[i] = beams_[i].dist;
            valid[i] = beams_[i].has_msg &&
                       std::isfinite(beams_[i].dist) &&
                       beams_[i].dist < 29.9;
        }
    }

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

    std::array<double, 3> ruv_cmd = sub_if_->get_cmd(dist, valid, v_sway_meas);

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