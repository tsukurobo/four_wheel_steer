/**
 * @file Sample.cpp
 * @brief 速度指令を受けてArduinoに各ステアの角度(rad)及びタイヤの角速度(rad/sec)を送信するノード
 */

#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <msgs/FourWheelSteerRad.h>
#include <msgs/FourWheelSteerPIDGain.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "FourWheelSteer.h"

FourWheelSteer steer((80.0 + 0.92) * (21.0 / 49.0) * M_PI, 310.0, 350.0);
msgs::FourWheelSteerRad target;
msgs::FourWheelSteerPIDGain pid_gain;

class FourWheelSteerController
{
private:
    constexpr static int XVEHICLE = 0;
    constexpr static int YVEHICLE = 1;
    constexpr static int PARALLEL = 2;
    constexpr static int ROTATE = 3;
    ros::NodeHandle _nh;
    ros::NodeHandle _pnh;
    ros::Publisher _target_pub;
    ros::Subscriber _vel_sub;
    ros::Subscriber _drive_mode_sub;
    ros::Subscriber _enable_signal_sub;
    ros::Subscriber _rad_sub;
    ros::Timer _timer;
    geometry_msgs::Twist _last_cmd;
    msgs::FourWheelSteerRad target;
    int _mode = XVEHICLE;
    bool _enable_signal = false;

    double vx = 0.0, vy = 0.0, wx = 0.0, wy = 0.0, TurnRadius = 1e6;
    double v_max = 1.0, w_max = M_PI, TurnRadius_min = 0.8, TurnRadius_max = 1e6;

public:
    FourWheelSteerController() : _nh(), _pnh("~")
    {
        _target_pub = _nh.advertise<msgs::FourWheelSteerRad>("target", 1);
        _vel_sub = _nh.subscribe("cmd_vel", 10, &FourWheelSteerController::velCallback, this);
        _drive_mode_sub = _nh.subscribe("drive_mode", 10, &FourWheelSteerController::driveModeCallback, this);
        _enable_signal_sub = _nh.subscribe("enable_signal", 10, &FourWheelSteerController::enableSignalCallback, this);
        _timer = _nh.createTimer(ros::Duration(0.001), &FourWheelSteerController::timerCallback, this);

        steer.setVMax(v_max);
        steer.setWMax(w_max);
        steer.setTurnRadiusMin(TurnRadius_min);
        steer.setTurnRadiusMax(TurnRadius_max);

        target.stop = true;
    }

    void velCallback(const geometry_msgs::Twist &cmd_msg)
    {
        _last_cmd = cmd_msg;
    }

    void driveModeCallback(const std_msgs::Int8 &drive_mode_msg)
    {
        _mode = drive_mode_msg.data;
    }

    void enableSignalCallback(const std_msgs::Bool &enable_signal_msg)
    {
        _enable_signal = enable_signal_msg.data;
    }

    void timerCallback(const ros::TimerEvent &e)
    {
        _pnh.getParam("v_max", v_max);
        if (_pnh.getParam("w_max", w_max))
            w_max *= M_PI;
        _pnh.getParam("TurnRadius_min", TurnRadius_min);
        _pnh.getParam("TurnRadius_max", TurnRadius_max);

        if (_enable_signal)
        {
            target.stop = false;
            vx = _last_cmd.linear.x;
            vy = _last_cmd.linear.y;
            wx = _last_cmd.angular.z;
            wy = -_last_cmd.angular.z;

            if (_mode == XVEHICLE)
            {
                steer.xVehicle(vx, wx);
                ROS_INFO_STREAM("XVEHICLE");
            }
            else if (_mode == YVEHICLE)
            {
                steer.yVehicle(vy, wy);
                ROS_INFO_STREAM("YVEHICLE");
            }
            else if (_mode == ROTATE)
            {
                steer.rotate(wx);
                ROS_INFO_STREAM("ROTATE");
            }
        }
        else
        {
            steer.stop();
            target.stop = true;
            ROS_INFO_STREAM("STOP");
        }

        setTarget();
        _target_pub.publish(target);
    }

    void setTarget()
    {
        for (int i = 0; i < 4; i++)
        {
            target.angle[i] = steer.getAngle(i);
            target.angVel[i] = steer.getAngVel(i);
        }
    }
};

//////////////////

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    FourWheelSteerController controller;

    ros::Rate loop_rate(1000);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}