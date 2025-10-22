#include "AutoCar.h"
#include <stdio.h>
#include "math.h"
#include "MPU6050.h"
#include "Motor.h"
#include "PeropheralInit.h"

extern MotorInfo motorDataRight;
extern MotorInfo motorDataLeft;
extern imuInfo imu;
extern float btYawF2H;
extern float btSpeedLast;
extern float yawOdometry;

// 获取当前偏航角 yawCurrent
void setYawCurrent(AutoCarInfo* car) {
    if (car != NULL) {
        car->yawCurrent = imu.iyaw;
				btYawF2H = imu.iyaw;
			  yawOdometry= imu.iyaw;
    }
}

// 设置目标偏航角 yawtarget
void setYawTarget(AutoCarInfo* car, float yaw,double speedRightRef,double speedLeftRed) {
    if (car != NULL) {
        car->yawtarget = yaw;
				car->motorSpeedRightRefer = speedRightRef;
				car->motorSpeedLeftRefer = speedLeftRed;
    }
}

// 设置右电机转速---速度环，启动！ motorSpeedRight
void setMotorSpeedRight(AutoCarInfo* car, double speed) {
    if (car != NULL) {
        car->motorSpeedRight = speed;
				setTargetSpeed_perMin(&motorDataRight,speed);
				PID_loop(&motorDataRight);
				
    }
}

// 设置左电机转速---速度环，启动！ motorSpeedLeft
void setMotorSpeedLeft(AutoCarInfo* car, double speed) {
    if (car != NULL) {
        car->motorSpeedLeft = speed;
				setTargetSpeed_perMin(&motorDataLeft,speed);
				PID_loop(&motorDataLeft);
    }
}




// 设置角度误差 AngleError
void setAngleError(AutoCarInfo* car, double error) {
    if (car != NULL) {
        car->AngleError = error;
    }
}

// 设置积分误差 AngleErrorIteger（注意：原字段名拼写为 Iteger，应为 Integer，但此处保持与结构体一致）
void setAngleErrorIteger(AutoCarInfo* car, double errorIteger) {
    if (car != NULL) {
        car->AngleErrorIteger = errorIteger;
    }
}

// 设置微分误差 AngleErrorDerivate
void setAngleErrorDerivate(AutoCarInfo* car, double errorDerivate) {
    if (car != NULL) {
        car->AngleErrorDerivate = errorDerivate;
    }
}

// 设置上一次角度误差 AngleErrorLast
void setAngleErrorLast(AutoCarInfo* car, double errorLast) {
    if (car != NULL) {
        car->AngleErrorLast = errorLast;
    }
}


// 设置PID角度环参数 kp_Angle
void setKpAngle(AutoCarInfo* car, double kp) {
    if (car != NULL) {
        car->kp_Angle = kp;
    }
}

// 设置PID角度环参数 ki_Angle
void setKiAngle(AutoCarInfo* car, double ki) {
    if (car != NULL) {
        car->ki_Angle = ki;
    }
}

// 设置PID角度环参数 kd_Angle
void setKdAngle(AutoCarInfo* car, double kd) {
    if (car != NULL) {
        car->kd_Angle = kd;
    }
}

// 设置PID输出 PID_output
void setPIDOutput(AutoCarInfo* car, double output) {
    if (car != NULL) {
        car->PID_output = output;
    }
}

// 设置目标速度 TargetSpeed_output
void setTargetSpeedOutput(AutoCarInfo* car, double speed) {
    if (car != NULL) {
        car->TargetSpeed_output = speed;
				car->TargetSpeed_output_reset = 0.0 - speed;
    }
}

// 设置重置后的目标速度 TargetSpeed_output_reset
void setTargetSpeedOutputReset(AutoCarInfo* car, double speed) {
    if (car != NULL) {
        car->TargetSpeed_output_reset = speed;
				car->TargetSpeed_output = 0.0 - speed;

    }
}


// 设置PID参数
void setAutoCarInfoPID_para(AutoCarInfo* car, double kp,double ki,double kd) {
    if (car != NULL) {
        setKpAngle(car,kp);
        setKiAngle(car,ki);
			  setKdAngle(car,kd);

    }
}

// 设置误差项
void setAngleError_para(AutoCarInfo* car, double error, double errorInteger, double errorDerivate, double errorLast){

    if (car != NULL) {
			
			
        setAngleError(car,error);
        setAngleErrorIteger(car,errorInteger);
			  setAngleErrorDerivate(car,errorDerivate);
				setAngleErrorLast(car,errorLast);

    }


}



//--------------PID初始化--------------

// PID初始化
void PID_Angle_init(AutoCarInfo* car){
	


	//设置PID参数
	
	double kp=1.5,ki=0.6,kd=0.0;
	setAutoCarInfoPID_para(car,kp,ki,kd);
	

	
	//设置目标yaw，初始化参考速度
	
	float targetYaw = 0.0;
	setYawTarget(car,targetYaw,0.0,0.0);
	
	
	
	//初始化误差项
	
	double AngleError=0.0;
	double AngleErrorIteger=0.0;
	double AngleErrorDerivate=0.0;
	double AngleErrorLast=0.0;
	
	setAngleError_para(car,AngleError,AngleErrorIteger,AngleErrorDerivate,AngleErrorLast);

}

//PID角度环

void PID_Angle_loop(AutoCarInfo* car){
	
	
	//获取当前yaw
	setYawCurrent(car); 
	
	//计算误差
	

	setAngleError(car,calculateShortestAngleDifference((double)car->yawtarget,(double)car->yawCurrent));
	
	setAngleErrorIteger(car,car->AngleErrorIteger+(car->AngleError)*0.05);
	
	
	setAngleErrorDerivate(car,(car->AngleError-(car->AngleErrorLast))/0.05);
			
	car->PID_output = car->kp_Angle*car->AngleError
									 +car->ki_Angle*car->AngleErrorIteger
									 +car->kd_Angle*car->AngleErrorDerivate;
	
	setTargetSpeedOutput(car,car->PID_output);
	
	//设置左右电机预期转速（参考速度+PID输出），并且执行PID速度环
	setMotorSpeedRight(car,car->motorSpeedRightRefer + car->TargetSpeed_output_reset);
	setMotorSpeedLeft(car,car->motorSpeedLeftRefer + car->TargetSpeed_output);
	
	
	
	

	//打印或更新数据
	
	//printf("%f,%f,%f\n",car->yawCurrent,car->yawtarget,imu.iyaw);
	
	setAngleErrorLast(car,car->AngleError);

	
}

//角度最短路径计算

double calculateShortestAngleDifference(double current_angle, double target_angle) {
    double difference = fmod(target_angle - current_angle, 360.0);
    
    // 如果difference不在[-180, 180]范围内，则调整它
    if (difference > 180.0) {
        difference -= 360.0;
    } else if (difference < -180.0) {
        difference += 360.0;
    }
    
    return difference;
}




