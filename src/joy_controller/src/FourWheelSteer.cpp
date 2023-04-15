#include "FourWheelSteer.h"


FourWheelSteer::FourWheelSteer(double WheelRadius, double DistWheelCenter, double v_max, double w_max, double TurnRadius_min, double TurnRadius_max)
    : VEL_TO_RAD(2*M_PI / (2*abs(WheelRadius)*M_PI / 1000.0)), DistWheelCenter(abs(DistWheelCenter) / 1000.0), v_max(abs(v_max)), w_max(abs(w_max)), TurnRadius_min(abs(TurnRadius_min)), TurnRadius_max(abs(TurnRadius_max))
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
    double angVel = v * VEL_TO_RAD;
    AngleLimitter(angle, angVel);
    Angle[0] = Angle[1] = Angle[2] = Angle[3] = angle;
    AngVel[0] = AngVel[1] = AngVel[2] = AngVel[3] = angVel;
}

void FourWheelSteer::rotate(double w) {
    wLimitter(w);

    Angle[0] = Angle[2] = -M_PI_4;      // 左前、右後のステア角
    Angle[1] = Angle[3] =  M_PI_4;      // 右前、左後のステア角

    double angVel = DistWheelCenter * w * VEL_TO_RAD;
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
    static const double HalfDistWheel = DistWheelCenter / ROOT2;
    double Speed = vx * VEL_TO_RAD;

    double inside_angle = atan2(TurnRadius - HalfDistWheel, HalfDistWheel);
    double outside_angle = atan2(TurnRadius + HalfDistWheel, HalfDistWheel);
    double inside_angVel = hypot(TurnRadius - HalfDistWheel, HalfDistWheel) / TurnRadius * Speed;
    double outside_angVel = hypot(TurnRadius + HalfDistWheel, HalfDistWheel) / TurnRadius * Speed;
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

        vel[i] = angVel[i] / VEL_TO_RAD;
    }

    x += (vel[0]*cos(theta+angle[0]) + vel[1]*cos(theta+angle[1]) + vel[2]*cos(theta+angle[2]) + vel[3]*cos(theta+angle[3])) / 4.0 * dt;
    y += (vel[0]*sin(theta+angle[0]) + vel[1]*sin(theta+angle[1]) + vel[2]*sin(theta+angle[2]) + vel[3]*sin(theta+angle[3])) / 4.0 * dt;
    theta += ROOT2*(vel[0]*(-cos(angle[0])+sin(angle[0])) + 
                    vel[1]*(-cos(angle[1])-sin(angle[1])) + 
                    vel[2]*( cos(angle[2])-sin(angle[2])) + 
                    vel[3]*( cos(angle[3])+sin(angle[3]))) / 8.0 / DistWheelCenter * dt;
}
