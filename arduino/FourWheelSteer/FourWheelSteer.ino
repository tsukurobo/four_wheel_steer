#include <Arduino.h>
#include "cubic_arduino.h"
#include "PID.h"
#include "Cubic.controller.h"

#include <ros.h>
#include "msgs/FourWheelSteerRad.h"
#include "msgs/FourWheelSteerPIDGain.h"
#include <std_msgs/Int16MultiArray.h>

ros::NodeHandle nh;

const uint8_t driveMotorNum[] = {4, 2, 0, 6}, steerMotorNum[] = {5, 3, 1, 7};
const uint8_t incEncNum[] = {2, 1, 0, 3}, absEncNum[] = {2, 1, 0, 3}; 

float angle[4], angVel[4];
float Vkp[4], Vki[4], Vkd[4], Pkp[4], Pki[4], Pkd[4];
const uint16_t INC_CPR = 2048;
const double steerCapableDuty = 0.3, driveCapableDuty[] = {0.312, 0.3, 0.325, 0.3};
bool Stop = true;

std_msgs::Int16MultiArray duty_msg, enc_msg;
msgs::FourWheelSteerRad rad_msg;

void targetCb(const msgs::FourWheelSteerRad &target) {
  for(int i = 0; i < 4; i++) {
    angle[i] = target.angle[i];
    angVel[i] = -target.angVel[i];
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
  
  nh.getHardware()->setBaud(2000000);
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
    {driveMotorNum[0], incEncNum[0], encoderType::inc, INC_CPR, driveCapableDuty[0], Vkp[0], Vki[0], Vkd[0], angVel[0], false, false},
    {driveMotorNum[1], incEncNum[1], encoderType::inc, INC_CPR, driveCapableDuty[1], Vkp[1], Vki[1], Vkd[1], angVel[1], false, false},
    {driveMotorNum[2], incEncNum[2], encoderType::inc, INC_CPR, driveCapableDuty[2], Vkp[2], Vki[2], Vkd[2], angVel[2], false, false},
    {driveMotorNum[3], incEncNum[3], encoderType::inc, INC_CPR, driveCapableDuty[3], Vkp[3], Vki[3], Vkd[3], angVel[3], false, false},
  };
  static Position_PID steerPID[] = {
    {steerMotorNum[0], absEncNum[0], encoderType::abs, AMT22_CPR, steerCapableDuty, Pkp[0], Pki[0], Pkd[0], angle[0], false, false},
    {steerMotorNum[1], absEncNum[1], encoderType::abs, AMT22_CPR, steerCapableDuty, Pkp[1], Pki[1], Pkd[1], angle[1], true, false},
    {steerMotorNum[2], absEncNum[2], encoderType::abs, AMT22_CPR, steerCapableDuty, Pkp[2], Pki[2], Pkd[2], angle[2], false, false},
    {steerMotorNum[3], absEncNum[3], encoderType::abs, AMT22_CPR, steerCapableDuty, Pkp[3], Pki[3], Pkd[3], angle[3], false, false},
  };

  if (Stop) {
    for(int i = 0; i < 4; i++) {
      DC_motor::put(driveMotorNum[i], 0);
      DC_motor::put(steerMotorNum[i], 0);
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
      // DC_motor::put(steerMotorNum[i], 200);
    }
  }

  for(int i = 0; i < 4; i++) {
    enc_msg.data[i]   = Inc_enc::get(incEncNum[i]);
    enc_msg.data[i+4] = Abs_enc::get(absEncNum[i]);
    duty_msg.data[i] = DC_motor::get(driveMotorNum[i]);
    duty_msg.data[i+4] = DC_motor::get(steerMotorNum[i]);
    rad_msg.angVel[i] = encoderToAngle(Inc_enc::get_diff(incEncNum[i]), INC_CPR);
    rad_msg.angle[i]  = encoderToAngle(Abs_enc::get(absEncNum[i]), AMT22_CPR);
  }
  // enc_pub.publish(&enc_msg);
  // duty_pub.publish(&duty_msg);
  rad_pub.publish(&rad_msg);

  Cubic::update();
}