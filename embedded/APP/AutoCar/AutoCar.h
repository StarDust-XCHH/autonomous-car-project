#ifndef __AutoCar_H
#define __AutoCar_H



// 定义结构体
typedef struct {
	
		//雷达在头（行驶方向）
	
		float yawCurrent; //小车当前偏航角 （雷达方向为0度，向左转0~-180，向右转0~+180）
		float yawtarget; // 小车目标偏航角
		double motorSpeedRight;   // 右电机参考转速，单位circle/min
		double motorSpeedLeft;		// 左电机参考转速
    double motorSpeedRightRefer;   // 右电机参考转速，单位circle/min
		double motorSpeedLeftRefer;		// 左电机参考转速
	
	
	
		//PID角度环参数
		double AngleError;
		double AngleErrorIteger;
		double AngleErrorDerivate;
		double AngleErrorLast;
		double kp_Angle,ki_Angle,kd_Angle;
		double PID_output;
		double TargetSpeed_output;
		double TargetSpeed_output_reset;
	
} AutoCarInfo;






//--------------set函数--------------

// 获取当前偏航角 yawCurrent（无参数，从传感器读取）
void setYawCurrent(AutoCarInfo* car);

// 设置目标偏航角 yawtarget
void setYawTarget(AutoCarInfo* car, float yaw,double speedRight,double speedLeft);

// 设置右电机转速---速度环，启动！ motorSpeedRight
void setMotorSpeedRight(AutoCarInfo* car, double speed);

// 设置左电机转速---速度环，启动！ motorSpeedLeft
void setMotorSpeedLeft(AutoCarInfo* car, double speed);

// 设置角度误差 AngleError
void setAngleError(AutoCarInfo* car, double error);

// 设置积分误差 AngleErrorIteger
void setAngleErrorIteger(AutoCarInfo* car, double errorIteger);

// 设置微分误差 AngleErrorDerivate
void setAngleErrorDerivate(AutoCarInfo* car, double errorDerivate);

// 设置上一次角度误差 AngleErrorLast
void setAngleErrorLast(AutoCarInfo* car, double errorLast);

// 设置PID角度环参数 kp_Angle
void setKpAngle(AutoCarInfo* car, double kp);

// 设置PID角度环参数 ki_Angle
void setKiAngle(AutoCarInfo* car, double ki);

// 设置PID角度环参数 kd_Angle
void setKdAngle(AutoCarInfo* car, double kd);

// 设置PID输出 PID_output
void setPIDOutput(AutoCarInfo* car, double output);

// 设置目标速度 TargetSpeed_output
void setTargetSpeedOutput(AutoCarInfo* car, double speed);

// 设置重置后的目标速度 TargetSpeed_output_reset
void setTargetSpeedOutputReset(AutoCarInfo* car, double speed);

// 设置PID参数
void setAutoCarInfoPID_para(AutoCarInfo* car, double kp,double ki,double kd) ;

// 设置误差项
void setAngleError_para(AutoCarInfo* car, double error, double errorInteger, double errorDerivate, double errorLast);

// PID初始化
void PID_Angle_init(AutoCarInfo* car);

//PID角度环

void PID_Angle_loop(AutoCarInfo* car);

//PID角度环

void PID_Angle_loop(AutoCarInfo* car);

//最短角度路线计算
double calculateShortestAngleDifference(double current_angle, double target_angle) ;

#endif
