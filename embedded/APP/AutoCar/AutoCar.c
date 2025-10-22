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

// ��ȡ��ǰƫ���� yawCurrent
void setYawCurrent(AutoCarInfo* car) {
    if (car != NULL) {
        car->yawCurrent = imu.iyaw;
				btYawF2H = imu.iyaw;
			  yawOdometry= imu.iyaw;
    }
}

// ����Ŀ��ƫ���� yawtarget
void setYawTarget(AutoCarInfo* car, float yaw,double speedRightRef,double speedLeftRed) {
    if (car != NULL) {
        car->yawtarget = yaw;
				car->motorSpeedRightRefer = speedRightRef;
				car->motorSpeedLeftRefer = speedLeftRed;
    }
}

// �����ҵ��ת��---�ٶȻ��������� motorSpeedRight
void setMotorSpeedRight(AutoCarInfo* car, double speed) {
    if (car != NULL) {
        car->motorSpeedRight = speed;
				setTargetSpeed_perMin(&motorDataRight,speed);
				PID_loop(&motorDataRight);
				
    }
}

// ��������ת��---�ٶȻ��������� motorSpeedLeft
void setMotorSpeedLeft(AutoCarInfo* car, double speed) {
    if (car != NULL) {
        car->motorSpeedLeft = speed;
				setTargetSpeed_perMin(&motorDataLeft,speed);
				PID_loop(&motorDataLeft);
    }
}




// ���ýǶ���� AngleError
void setAngleError(AutoCarInfo* car, double error) {
    if (car != NULL) {
        car->AngleError = error;
    }
}

// ���û������ AngleErrorIteger��ע�⣺ԭ�ֶ���ƴдΪ Iteger��ӦΪ Integer�����˴�������ṹ��һ�£�
void setAngleErrorIteger(AutoCarInfo* car, double errorIteger) {
    if (car != NULL) {
        car->AngleErrorIteger = errorIteger;
    }
}

// ����΢����� AngleErrorDerivate
void setAngleErrorDerivate(AutoCarInfo* car, double errorDerivate) {
    if (car != NULL) {
        car->AngleErrorDerivate = errorDerivate;
    }
}

// ������һ�νǶ���� AngleErrorLast
void setAngleErrorLast(AutoCarInfo* car, double errorLast) {
    if (car != NULL) {
        car->AngleErrorLast = errorLast;
    }
}


// ����PID�ǶȻ����� kp_Angle
void setKpAngle(AutoCarInfo* car, double kp) {
    if (car != NULL) {
        car->kp_Angle = kp;
    }
}

// ����PID�ǶȻ����� ki_Angle
void setKiAngle(AutoCarInfo* car, double ki) {
    if (car != NULL) {
        car->ki_Angle = ki;
    }
}

// ����PID�ǶȻ����� kd_Angle
void setKdAngle(AutoCarInfo* car, double kd) {
    if (car != NULL) {
        car->kd_Angle = kd;
    }
}

// ����PID��� PID_output
void setPIDOutput(AutoCarInfo* car, double output) {
    if (car != NULL) {
        car->PID_output = output;
    }
}

// ����Ŀ���ٶ� TargetSpeed_output
void setTargetSpeedOutput(AutoCarInfo* car, double speed) {
    if (car != NULL) {
        car->TargetSpeed_output = speed;
				car->TargetSpeed_output_reset = 0.0 - speed;
    }
}

// �������ú��Ŀ���ٶ� TargetSpeed_output_reset
void setTargetSpeedOutputReset(AutoCarInfo* car, double speed) {
    if (car != NULL) {
        car->TargetSpeed_output_reset = speed;
				car->TargetSpeed_output = 0.0 - speed;

    }
}


// ����PID����
void setAutoCarInfoPID_para(AutoCarInfo* car, double kp,double ki,double kd) {
    if (car != NULL) {
        setKpAngle(car,kp);
        setKiAngle(car,ki);
			  setKdAngle(car,kd);

    }
}

// ���������
void setAngleError_para(AutoCarInfo* car, double error, double errorInteger, double errorDerivate, double errorLast){

    if (car != NULL) {
			
			
        setAngleError(car,error);
        setAngleErrorIteger(car,errorInteger);
			  setAngleErrorDerivate(car,errorDerivate);
				setAngleErrorLast(car,errorLast);

    }


}



//--------------PID��ʼ��--------------

// PID��ʼ��
void PID_Angle_init(AutoCarInfo* car){
	


	//����PID����
	
	double kp=1.5,ki=0.6,kd=0.0;
	setAutoCarInfoPID_para(car,kp,ki,kd);
	

	
	//����Ŀ��yaw����ʼ���ο��ٶ�
	
	float targetYaw = 0.0;
	setYawTarget(car,targetYaw,0.0,0.0);
	
	
	
	//��ʼ�������
	
	double AngleError=0.0;
	double AngleErrorIteger=0.0;
	double AngleErrorDerivate=0.0;
	double AngleErrorLast=0.0;
	
	setAngleError_para(car,AngleError,AngleErrorIteger,AngleErrorDerivate,AngleErrorLast);

}

//PID�ǶȻ�

void PID_Angle_loop(AutoCarInfo* car){
	
	
	//��ȡ��ǰyaw
	setYawCurrent(car); 
	
	//�������
	

	setAngleError(car,calculateShortestAngleDifference((double)car->yawtarget,(double)car->yawCurrent));
	
	setAngleErrorIteger(car,car->AngleErrorIteger+(car->AngleError)*0.05);
	
	
	setAngleErrorDerivate(car,(car->AngleError-(car->AngleErrorLast))/0.05);
			
	car->PID_output = car->kp_Angle*car->AngleError
									 +car->ki_Angle*car->AngleErrorIteger
									 +car->kd_Angle*car->AngleErrorDerivate;
	
	setTargetSpeedOutput(car,car->PID_output);
	
	//�������ҵ��Ԥ��ת�٣��ο��ٶ�+PID�����������ִ��PID�ٶȻ�
	setMotorSpeedRight(car,car->motorSpeedRightRefer + car->TargetSpeed_output_reset);
	setMotorSpeedLeft(car,car->motorSpeedLeftRefer + car->TargetSpeed_output);
	
	
	
	

	//��ӡ���������
	
	//printf("%f,%f,%f\n",car->yawCurrent,car->yawtarget,imu.iyaw);
	
	setAngleErrorLast(car,car->AngleError);

	
}

//�Ƕ����·������

double calculateShortestAngleDifference(double current_angle, double target_angle) {
    double difference = fmod(target_angle - current_angle, 360.0);
    
    // ���difference����[-180, 180]��Χ�ڣ��������
    if (difference > 180.0) {
        difference -= 360.0;
    } else if (difference < -180.0) {
        difference += 360.0;
    }
    
    return difference;
}




