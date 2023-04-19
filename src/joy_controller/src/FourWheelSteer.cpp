#include "FourWheelSteer.h"


FourWheelSteer::FourWheelSteer(double DistPerEnc, double DistFBWheel, double DistLRWheel, double v_max, double w_max, double TurnRadius_min, double TurnRadius_max)
    : DistPerEnc(DistPerEnc / 1000.0), DistWheelCenter(hypot(DistFBWheel, DistLRWheel) / 1000.0), DistFBWheel(DistFBWheel / 1000.0), DistLRWheel(DistLRWheel / 1000.0), v_max(abs(v_max)), w_max(abs(w_max)), TurnRadius_min(abs(TurnRadius_min)), TurnRadius_max(abs(TurnRadius_max))
{
    
}

double FourWheelSteer::vLimitter(double &vx, double &vy) {
    // v_maxを越えていた場合、比を保ったまま補正する。
    double v = hypot(vx, vy);
    if (v > v_max) {
        vx = vx/v * v_max;
        vy = vy/v * v_max;
        return v_max;
    }
    return v;
}

void FourWheelSteer::vxLimitter(double &vx) {
    // v_maxを越えていた場合、補正する。
    if (abs(vx) > v_max) vx = copysign(v_max, vx);
}

void FourWheelSteer::wLimitter(double &w) {
    // w_maxを越えていた場合、補正する。
    if (abs(w) > w_max) w = copysign(w_max, w);
}

bool FourWheelSteer::TurnRadiusLimitter(double &TurnRadius) {
    bool straight = false;
    // TurnRadius_minを下回っていた場合、補正する。
    if (abs(TurnRadius) < TurnRadius_min) TurnRadius = TurnRadius_min;
    // TurnRadius_maxを上回っていた場合、補正して直進フラグを立てる
    else if (abs(TurnRadius) + FLT_ZERO > TurnRadius_max) {
        TurnRadius = TurnRadius_max;
        straight = true;
    }
    return straight;
}

void FourWheelSteer::AngleLimitter(double &Angle, double &AngVel) {
    // Angleが±π/2を超えていた場合、補正する。
    if (Angle > M_PI_2) {
        Angle -= M_PI;
        AngVel *= -1.0;
    }
    else if (Angle < -M_PI_2) {
        Angle += M_PI;
        AngVel *= -1.0;
    }
}

void FourWheelSteer::parallel(double vx, double vy) {
    double v = vLimitter(vx, vy);
    if (abs(v) < FLT_ZERO) {
        stop();
        return;
    }
    double angle = atan2(vy, vx);
    double angVel = v / DistPerEnc;
    AngleLimitter(angle, angVel);
    Angle[0] = Angle[1] = Angle[2] = Angle[3] = angle;
    AngVel[0] = AngVel[1] = AngVel[2] = AngVel[3] = angVel;
}

void FourWheelSteer::rotate(double w) {
    wLimitter(w);
    static const double angle = atan2(DistFBWheel, DistLRWheel);
    Angle[0] = Angle[2] = -angle;   // 左前、右後のステア角
    Angle[1] = Angle[3] =  angle;   // 右前、左後のステア角

    double angVel = DistWheelCenter * w / DistPerEnc;
    AngVel[0] = AngVel[1] = -angVel;    // 左前、左後の角速度
    AngVel[2] = AngVel[3] =  angVel;    // 右前、右後の角速度
}

void FourWheelSteer::vehicle(double vx, double TurnRadius, bool TurnLeft) {
    TurnRadius = abs(TurnRadius);
    bool straight = TurnRadiusLimitter(TurnRadius);
    vxLimitter(vx);

    if (straight) {
        parallel(vx, 0.0);
        return;
    }
    static const double HalfDistLRWheel = DistLRWheel / 2.0, HalfDistFBWheel = DistFBWheel / 2.0;
    double angVel = vx / DistPerEnc;

    double inside_angle   = atan2(HalfDistFBWheel, TurnRadius - HalfDistLRWheel);
    double outside_angle  = atan2(HalfDistFBWheel, TurnRadius + HalfDistLRWheel);
    double inside_angVel  = hypot(HalfDistFBWheel, TurnRadius - HalfDistLRWheel) / TurnRadius * angVel;
    double outside_angVel = hypot(HalfDistFBWheel, TurnRadius + HalfDistLRWheel) / TurnRadius * angVel;
    if (TurnLeft) {
        Angle[0] = inside_angle; Angle[1] = -inside_angle;
        Angle[2] = -outside_angle; Angle[3] = outside_angle;
        AngVel[0] = AngVel[1] = inside_angVel;
        AngVel[2] = AngVel[3] = outside_angVel;
    }
    else {
        Angle[0] = -outside_angle; Angle[1] = outside_angle;
        Angle[2] = inside_angle; Angle[3] = -inside_angle;
        AngVel[0] = AngVel[1] = outside_angVel;
        AngVel[2] = AngVel[3] = inside_angVel;
    }
    // 旋回半径が小さすぎない限り、±π/2を超えることはないが、念の為補正する。
    for (int i = 0; i < 4; i++) AngleLimitter(Angle[i], AngVel[i]);
}

bool FourWheelSteer::anomalyDetect(double angVel[4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = i + 1; j < 4; j++) {
            if (abs(angVel[i]) < abs(angVel[j])) {
                if (abs(angVel[i])*ALLOWABLE_ANGVEL_ERROR < abs(angVel[j])) {
                    stop();
                    return true;
                }
            }
            else {
                if (abs(angVel[j])*ALLOWABLE_ANGVEL_ERROR < abs(angVel[i])) {
                    stop();
                    return true;
                }
            }
        }
    }
    return false;
}

// 縦横比が異なる場合に対応できていない
void FourWheelSteer::calcOdom(double angVel[4], double angle[4]) {
    ros::Time now_time = ros::Time::now();
    double dt = now_time.toSec() - prev_time.toSec();
    prev_time = now_time;

    double vel[4];
    for(int i = 0; i < 4; i++) {
        // 速度がマイナスの場合、角度を180度回転させる。
        if (angVel[i] < 0.0) {
            angVel[i] = abs(angVel[i]);
            if (angle[i] < 0.0) angle[i] += M_PI;
            else angle[i] -= M_PI;
        }

        vel[i] = angVel[i] * DistPerEnc;
    }

    x += (vel[0]*cos(theta+angle[0]) + vel[1]*cos(theta+angle[1]) + vel[2]*cos(theta+angle[2]) + vel[3]*cos(theta+angle[3])) / 4.0 * dt;
    y += (vel[0]*sin(theta+angle[0]) + vel[1]*sin(theta+angle[1]) + vel[2]*sin(theta+angle[2]) + vel[3]*sin(theta+angle[3])) / 4.0 * dt;
    theta += ROOT2*(vel[0]*(-cos(angle[0])+sin(angle[0])) + 
                    vel[1]*(-cos(angle[1])-sin(angle[1])) + 
                    vel[2]*( cos(angle[2])-sin(angle[2])) + 
                    vel[3]*( cos(angle[3])+sin(angle[3]))) / 8.0 / DistWheelCenter * dt;
}
