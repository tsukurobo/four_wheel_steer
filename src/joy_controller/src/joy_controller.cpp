#include <ros/ros.h>
#include <math.h>
#include <msgs/FourWheelSteerRad.h>
#include <msgs/FourWheelSteerPIDGain.h>
#include <sensor_msgs/Joy.h>
#include "FourWheelSteer.h"

using namespace std;

// ros::init(argc, argv, "controller");
// ros::NodeHandle nh;

msgs::FourWheelSteerRad target;
msgs::FourWheelSteerPIDGain pid_gain;

string mode = "VEHICLE";

int freq = 1000;

FourWheelSteer steer(34.286, 31.0, 35.0);
double vx = 0.0, vy = 0.0, w = 0.0, TurnRadius = 1e6;
double v_max = 1.0, w_max = M_PI, TurnRadius_min = 0.8, TurnRadius_max = 1e6;

double Vkp[4], Vki[4], Vkd[4];
double Pkp[4], Pki[4], Pkd[4];

void joyCb(const sensor_msgs::Joy &joy_msg) {
    if (joy_msg.buttons[5]) {
        target.stop = false;
        vy =  joy_msg.axes[0] * v_max;
        vx =  joy_msg.axes[1] * v_max;
        w  = -joy_msg.axes[2] * w_max;
        ROS_INFO_STREAM(mode);

        if (mode == "VEHICLE") {
            steer.vehicle(vx, w);
        }
        else if (mode == "PARALLEL") {
            steer.parallel(vx, vy);
        }
        else if (mode == "ROTATE") {
            steer.rotate(w);
        }
    }
    else {
        steer.stop();
        target.stop = true;
        ROS_INFO_STREAM("STOP");
    }

    if (joy_msg.axes[5] == 1) {
        mode = "VEHICLE";
        ROS_INFO_STREAM("MODE CHANGE: VEHCILE");
    }
    else if (joy_msg.axes[5] == -1) {
        mode = "PARALLEL";
        ROS_INFO_STREAM("MODE CHANGE: PARALLEL");
    }
    else if (joy_msg.axes[4] == 1 || joy_msg.axes[4] == -1) {
        mode = "ROTATE";
        ROS_INFO_STREAM("MODE CHANGE: ROTATE");
    }
}

void radCb(const msgs::FourWheelSteerRad &rad_msg) {
    double angVel[4];
    for (int i = 0; i < 4; i++) {
        angVel[i] = rad_msg.angVel[i];
    }
    if (steer.anomalyDetect(angVel)) {
        ROS_INFO_STREAM("ANOMALY DETECTED");
        target.stop = true;
    }
}

void setTarget() {
    for (int i = 0; i < 4; i++) {
        target.angle[i] = steer.getAngle(i);
        target.angVel[i] = steer.getAngVel(i);
    }
}

void setGain() {
    for (int i = 0; i < 4; i++) {
        pid_gain.Vkp[i] = Vkp[i];
        pid_gain.Vki[i] = Vki[i];
        pid_gain.Vkd[i] = Vkd[i];
        pid_gain.Pkp[i] = Pkp[i];
        pid_gain.Pki[i] = Pki[i];
        pid_gain.Pkd[i] = Pkd[i];
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;

    ros::NodeHandle pnh("~");
    pnh.getParam("v_max", v_max);
    if(pnh.getParam("w_max", w_max)) w_max *= M_PI;
    pnh.getParam("TurnRadius_min", TurnRadius_min);
    pnh.getParam("TurnRadius_max", TurnRadius_max);
    pnh.getParam("Vkp0", Vkp[0]);
    pnh.getParam("Vki0", Vki[0]);
    pnh.getParam("Vkd0", Vkd[0]);
    pnh.getParam("Pkp0", Pkp[0]);
    pnh.getParam("Pki0", Pki[0]);
    pnh.getParam("Pkd0", Pkd[0]);
    pnh.getParam("Vkp1", Vkp[1]);
    pnh.getParam("Vki1", Vki[1]);
    pnh.getParam("Vkd1", Vkd[1]);
    pnh.getParam("Pkp1", Pkp[1]);
    pnh.getParam("Pki1", Pki[1]);
    pnh.getParam("Pkd1", Pkd[1]);
    pnh.getParam("Vkp2", Vkp[2]);
    pnh.getParam("Vki2", Vki[2]);
    pnh.getParam("Vkd2", Vkd[2]);
    pnh.getParam("Pkp2", Pkp[2]);
    pnh.getParam("Pki2", Pki[2]);
    pnh.getParam("Pkd2", Pkd[2]);
    pnh.getParam("Vkp3", Vkp[3]);
    pnh.getParam("Vki3", Vki[3]);
    pnh.getParam("Vkd3", Vkd[3]);
    pnh.getParam("Pkp3", Pkp[3]);
    pnh.getParam("Pki3", Pki[3]);
    pnh.getParam("Pkd3", Pkd[3]);
    pnh.getParam("freq", freq);

    steer.setVMax(v_max);
    steer.setWMax(w_max);
    steer.setTurnRadiusMin(TurnRadius_min);
    steer.setTurnRadiusMax(TurnRadius_max);

    target.stop = true;

    ros::Subscriber joy_sub = nh.subscribe("joy", 1, joyCb);
    ros::Subscriber rad_sub = nh.subscribe("rad", 1, radCb);
    ros::Publisher target_pub = nh.advertise<msgs::FourWheelSteerRad>("target", 1);
    ros::Publisher gain_pub = nh.advertise<msgs::FourWheelSteerPIDGain>("gain", 1);

    sleep(5);
    setGain();
    gain_pub.publish(pid_gain);
    
    ros::Rate loop_rate(freq);
    while (ros::ok()) {
        ros::spinOnce();
        setTarget();
        target_pub.publish(target);
        loop_rate.sleep();
    }
    return 0;
}