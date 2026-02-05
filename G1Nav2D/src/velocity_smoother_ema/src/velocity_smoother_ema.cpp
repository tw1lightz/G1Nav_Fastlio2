#include <velocity_smoother_ema/velocity_smoother_ema.hpp>

VelocitySmootherEma::VelocitySmootherEma(ros::NodeHandle* nh):nh_(*nh)
{
    // 默认不做EMA衰减（alpha=1.0），避免把TEB的最小速度抹小导致机器人迈不开步
    nh_.param<double>("/alpha_v", alpha_v, 1.0);
    nh_.param<double>("/alpha_w", alpha_w, 1.0);
    nh_.param<std::string>("/raw_cmd_topic", raw_cmd_topic, "raw_cmd_vel");
    nh_.param<std::string>("/cmd_topic", cmd_topic, "cmd_vel");
    nh_.param<double>("/cmd_rate", cmd_rate, 10.0);  // 保持10Hz，给g1_control足够的数据点
    nh_.param<int>("/stop_counter", stop_counter_max, 30); // 10Hz下30约等于3秒
    stop_counter = stop_counter_max;

    velocity_sub_ = nh_.subscribe(raw_cmd_topic, 10, &VelocitySmootherEma::twist_callback, this);
    velocity_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic, 10, true);
    timer = nh_.createTimer(ros::Duration(1.0 / cmd_rate), &VelocitySmootherEma::update, this);

    previous_x_vel = 0.0;
    previous_y_vel = 0.0;
    previous_w_vel = 0.0;
    x_vel = 0.0;
    y_vel = 0.0;
    w_vel = 0.0;
    smoothed_x_vel = 0.0;
    smoothed_y_vel = 0.0;
    smoothed_w_vel = 0.0;
}

VelocitySmootherEma::~VelocitySmootherEma()
{
    ros::shutdown();
}

void VelocitySmootherEma::twist_callback(const geometry_msgs::Twist::ConstPtr msg)
{
    // ROS_INFO("I RECEIVED A NEW MESSAGE");
    cmd_vel_msg_ = *msg;
    stop_counter = stop_counter_max;
}

void VelocitySmootherEma::update(const ros::TimerEvent&)
{
    if (stop_counter-- <= 0)
    {
        stop_counter = 0;
        cmd_vel_msg_.linear.x = 0.0;
        cmd_vel_msg_.linear.y = 0.0;
        cmd_vel_msg_.angular.z = 0.0;
        // ROS_INFO("THE CMD IS ZERO");
    }
    // ROS_INFO("THE CMD IS : %d", stop_counter);
    x_vel = cmd_vel_msg_.linear.x;
    y_vel = cmd_vel_msg_.linear.y;
    w_vel = cmd_vel_msg_.angular.z;

    smoothed_x_vel = alpha_v * x_vel + (1 - alpha_v) * previous_x_vel;
    smoothed_y_vel = alpha_v * y_vel + (1 - alpha_v) * previous_y_vel;
    smoothed_w_vel = alpha_w * w_vel + (1 - alpha_w) * previous_w_vel;

    cmd_vel_msg_.linear.x = smoothed_x_vel;
    cmd_vel_msg_.linear.y = smoothed_y_vel;
    cmd_vel_msg_.angular.z = smoothed_w_vel;

    previous_x_vel = smoothed_x_vel;
    previous_y_vel = smoothed_y_vel;
    previous_w_vel = smoothed_w_vel;    

    velocity_pub_.publish(cmd_vel_msg_);
    // ROS_INFO("PUBLISHING TWIST MESSAGE!");
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velocity_smoother_ema");

    ros::NodeHandle nh;

    VelocitySmootherEma vse(&nh);

    ros::spin();

    return 0;
}
