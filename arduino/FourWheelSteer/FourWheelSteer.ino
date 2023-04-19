#include <Arduino.h>
#include "cubic_arduino.h"
#include "PID.h"
#include "Cubic.controller.h"

#include <ros.h>
#include "msgs/FourWheelSteerRad.h"
#include "msgs/FourWheelSteerPIDGain.h"
#include <std_msgs/Int16MultiArray.h>

#define TWO_PI 6.283185307179586476925286766559

ros::NodeHandle nh;

const uint8_t driveMotorNum[] = {0, 1, 2, 3}, steerMotorNum[] = {4, 5, 6, 7};
const uint8_t incEncNum[] = {0, 1, 2, 3}, absEncNum[] = {0, 1, 2, 3}; 

float angle[4], angVel[4];
float Vkp[4], Vki[4], Vkd[4], Pkp[4], Pki[4], Pkd[4];
const uint16_t INC_CPR = 2048;
const double capableDuty = 0.5;
bool Stop = false;

std_msgs::Int16MultiArray duty_msg, enc_msg;
msgs::FourWheelSteerRad rad_msg;

void targetCb(const msgs::FourWheelSteerRad &target) {
  for(int i = 0; i < 4; i++) {
    angle[i] = target.angle[i];
    angVel[i] = target.angVel[i];
    Stop = target.stop;
  }
}

void gainCb(const msgs::FourWheelSteerPIDGain &gain) {
  for(int i = 0; i < 4; i++) {
    Vkp[i] = gain.Vkp[i];
    Vki[i] = gain.Vki[i];
    Vkd[i] = gain.Vkd[i];
    Pkp[i] = gain.Pkp[i];
    Pki[i] = gain.Pki[i];
    Pkd[i] = gain.Pkd[i];
  }
}

ros::Subscriber<msgs::FourWheelSteerRad> target_sub("target", targetCb);
ros::Subscriber<msgs::FourWheelSteerPIDGain> gain_sub("gain", gainCb);
ros::Publisher duty_pub("duty", &duty_msg);
ros::Publisher enc_pub("enc", &enc_msg);
ros::Publisher rad_pub("rad", &rad_msg);

void setup()
{
  Cubic::begin();
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();

  duty_msg.data = (int16_t*)malloc(sizeof(int16_t)*8);
  duty_msg.data_length = 8;
  enc_msg.data = (int16_t*)malloc(sizeof(int16_t)*8);
  enc_msg.data_length = 8;

  nh.subscribe(target_sub);
  nh.subscribe(gain_sub);
  nh.advertise(duty_pub);
  nh.advertise(enc_pub);
  nh.advertise(rad_pub);
}

void loop()
{
  nh.spinOnce();

  using namespace Cubic_controller;
  static Velocity_PID drivePID[] = {
    {driveMotorNum[0], incEncNum[0], encoderType::inc, INC_CPR, capableDuty, Vkp[0], Vki[0], Vkd[0], angVel[0], false, false},
    {driveMotorNum[1], incEncNum[1], encoderType::inc, INC_CPR, capableDuty, Vkp[1], Vki[1], Vkd[1], angVel[1], false, false},
    {driveMotorNum[2], incEncNum[2], encoderType::inc, INC_CPR, capableDuty, Vkp[2], Vki[2], Vkd[2], angVel[2], false, false},
    {driveMotorNum[3], incEncNum[3], encoderType::inc, INC_CPR, capableDuty, Vkp[3], Vki[3], Vkd[3], angVel[3], false, false},
  };
  static Position_PID steerPID[] = {
    {steerMotorNum[0], absEncNum[0], encoderType::abs, AMT22_CPR, capableDuty, Pkp[0], Pki[0], Pkd[0], angle[0], true, false},
    {steerMotorNum[1], absEncNum[1], encoderType::abs, AMT22_CPR, capableDuty, Pkp[1], Pki[1], Pkd[1], angle[1], true, false},
    {steerMotorNum[2], absEncNum[2], encoderType::abs, AMT22_CPR, capableDuty, Pkp[2], Pki[2], Pkd[2], angle[2], true, false},
    {steerMotorNum[3], absEncNum[3], encoderType::abs, AMT22_CPR, capableDuty, Pkp[3], Pki[3], Pkd[3], angle[3], true, false},
  };

  if (Stop) {
    for(int i = 0; i < 4; i++) {
      DC_motor::put(driveMotorNum[i], 0);
    }
  }
  else {
    for(int i = 0; i < 4; i++) {
      drivePID[i].setGains(Vkp[i], Vki[i], Vkd[i]);
      drivePID[i].setTarget(angVel[i]);
      drivePID[i].compute();

      steerPID[i].setGains(Pkp[i], Pki[i], Pkd[i]);
      steerPID[i].setTarget(angle[i]);
      steerPID[i].compute();
    }
  }

  for(int i = 0; i < 4; i++) {
    enc_msg.data[i] = Inc_enc::get(incEncNum[i]);
    enc_msg.data[i+4] = Abs_enc::get(absEncNum[i]);
    rad_msg.angle[i] = Cubic_controller::encoderToAngle(0, INC_CPR); // encoderToAngle(Inc_enc::get_diff(incEncNum[i]), INC_CPR);
    rad_msg.angVel[i] = Cubic_controller::encoderToAngle(0, AMT22_CPR); // encoderToAngle(Abs_enc::get(absEncNum[i]), AMT22_CPR);
  }
  //enc_pub.publish(&enc_msg);
  rad_pub.publish(&rad_msg);

  Cubic::update();
}