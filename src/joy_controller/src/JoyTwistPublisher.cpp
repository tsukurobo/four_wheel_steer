/**
 * @file JoyTwistPublisher.cpp
 * @brief ジョイコンの入力から速度指令を生成するノード
 */

#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>

class TwistPublisher
{
private:
    constexpr static int XVEHICLE = 0;
    constexpr static int YVEHICLE = 1;
    constexpr static int PARALLEL = 2;
    constexpr static int ROTATE = 3;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher cmd_pub_;
    ros::Publisher enable_signal_pub_;
    ros::Publisher drive_mode_pub_;
    ros::Subscriber joy_sub_;
    ros::Timer timer_;
    sensor_msgs::Joy last_joy_;
    int mode_ = XVEHICLE;
    int assign_x_ = 1;
    int assign_y_ = 0;
    int assign_z_ = 3;
    int assign_xvhecle_mode_ = 7;
    int assign_yvhecle_mode_ = 6;
    int assign_zrotate_mode_ = 5;
    int enable_button_ = 5;
    float max_x = 1.0;
    float max_y = 1.0;
    float max_z = 1.0;

public:
    TwistPublisher() : nh_(), pnh_("~")
    {
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        enable_signal_pub_ = nh_.advertise<std_msgs::Bool>("enable_signal", 1);
        drive_mode_pub_ = nh_.advertise<std_msgs::Int8>("drive_mode", 1);
        joy_sub_ = nh_.subscribe("joy", 10, &TwistPublisher::joyCallback, this);
        timer_ = nh_.createTimer(ros::Duration(0.1), &TwistPublisher::timerCallback, this);
    }

    void joyCallback(const sensor_msgs::Joy &joy_msg)
    {
        last_joy_ = joy_msg;
    }

    void timerCallback(const ros::TimerEvent &e)
    {
        /* ジョイコンの入力割当を設定 */
        setAssign();

        /* 最大速度の設定 */
        setMaxSpeed();

        /* 速度指令の処理 */
        geometry_msgs::Twist cmd_vel;
        calcCmdVel(cmd_vel);
        cmd_pub_.publish(cmd_vel);

        /* 安全ボタンの処理 */
        std_msgs::Bool enable_signal;
        if (0 <= enable_button_ && enable_button_ < last_joy_.buttons.size())
        {
            enable_signal.data = last_joy_.buttons[enable_button_];
        }
        enable_signal_pub_.publish(enable_signal);

        /* 走行モードの処理 */
        std_msgs::Int8 drive_mode;
        if (0 <= assign_xvhecle_mode_ && assign_xvhecle_mode_ < last_joy_.buttons.size())
        {
            if (last_joy_.axes[assign_xvhecle_mode_] == 1)
            {
                drive_mode.data = XVEHICLE;
                ROS_INFO_STREAM("MODE CHANGE: XVEHICLE");
            }
        }
        if (0 <= assign_yvhecle_mode_ && assign_yvhecle_mode_ < last_joy_.buttons.size())
        {
            if (last_joy_.axes[assign_yvhecle_mode_] == 1 || last_joy_.axes[assign_yvhecle_mode_] == -1)
            {
                drive_mode.data = YVEHICLE;
                ROS_INFO_STREAM("MODE CHANGE: YVEHICLE");
            }
        }
        if (0 <= assign_zrotate_mode_ && assign_zrotate_mode_ < last_joy_.buttons.size())
        {
            if (last_joy_.axes[assign_zrotate_mode_] == -1)
            {
                drive_mode.data = ROTATE;
                ROS_INFO_STREAM("MODE CHANGE: ROTATE");
            }
        }
        drive_mode_pub_.publish(drive_mode);
    }

    /**
     * @brief ジョイコンの入力割当を設定する(launchファイルから設定)
     */
    void setAssign()
    {
        // ジョイコン軸の割り当て
        pnh_.setParam("/joy/assign_x", assign_x_);
        pnh_.setParam("/joy/assign_y", assign_y_);
        pnh_.setParam("/joy/assign_z", assign_z_);
        pnh_.setParam("/joy/assign_xvhecle_mode", assign_xvhecle_mode_);
        pnh_.setParam("/joy/assign_yvhecle_mode", assign_yvhecle_mode_);
        pnh_.setParam("/joy/assign_zrotate_mode", assign_zrotate_mode_);
        // // 直進モードの割り当て
        // pnh_.getParam("/joy/assign_x_straight", assign_x_straight);
        // pnh_.getParam("/joy/assign_y_straight", assign_y_straight);

        // ジョイコンボタンの割り当て
        pnh_.getParam("/joy/enable_button", enable_button_);
    }

    /**
     * @brief 最大速度の設定(launchファイルから設定)
     * @param max_x: x(正面：前が正)方向の最大速度(m/sec)
     * @param max_y: y(側面：左が正)方向の最大速度(m/sec)
     * @param max_z: z(回転：左回転が正)方向の最大速度(rad/sec)
     */
    void setMaxSpeed()
    {
        pnh_.setParam("/joy/max_x", max_x);
        pnh_.setParam("/joy/max_y", max_y);
        pnh_.setParam("/joy/max_z", max_z);
    }

    /**
     *  @brief ジョイコンの入力から速度を決定する
     * @param cmd_vel: 速度指令
     */
    void calcCmdVel(geometry_msgs::Twist &cmd_vel)
    {
        // // 直進モードの処理
        // if (0 <= assign_x_straight && assign_x_straight < last_joy_.axes.size())
        // {
        //     cmd_vel.linear.x = max_x * last_joy_.axes[assign_x_straight];
        // }
        // if (0 <= assign_y_straight && assign_y_straight < last_joy_.axes.size())
        // {
        //     cmd_vel.linear.y = max_y * last_joy_.axes[assign_y_straight];
        // }
        if (0 <= assign_x_ && assign_x_ < last_joy_.axes.size())
        {
            cmd_vel.linear.x = max_x * last_joy_.axes[assign_x_];
        }
        if (0 <= assign_y_ && assign_y_ < last_joy_.axes.size())
        {
            cmd_vel.linear.y = max_y * last_joy_.axes[assign_y_];
        }
        if (0 <= assign_z_ && assign_z_ < last_joy_.axes.size())
        {
            cmd_vel.angular.z = max_z * last_joy_.axes[assign_z_];
        }

        // 正規化
        double magnitude = std::hypot(cmd_vel.linear.x, cmd_vel.linear.y);
        if (magnitude > 1.0)
        {
            cmd_vel.linear.x = cmd_vel.linear.x / magnitude;
            cmd_vel.linear.y = cmd_vel.linear.y / magnitude;
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "JoyTwistPublisher");
    TwistPublisher twist_publisher;

    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
