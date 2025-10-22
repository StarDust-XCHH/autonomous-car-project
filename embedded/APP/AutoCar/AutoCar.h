#ifndef __AutoCar_H
#define __AutoCar_H



// ����ṹ��
typedef struct {
	
		//�״���ͷ����ʻ����
	
		float yawCurrent; //С����ǰƫ���� ���״﷽��Ϊ0�ȣ�����ת0~-180������ת0~+180��
		float yawtarget; // С��Ŀ��ƫ����
		double motorSpeedRight;   // �ҵ���ο�ת�٣���λcircle/min
		double motorSpeedLeft;		// �����ο�ת��
    double motorSpeedRightRefer;   // �ҵ���ο�ת�٣���λcircle/min
		double motorSpeedLeftRefer;		// �����ο�ת��
	
	
	
		//PID�ǶȻ�����
		double AngleError;
		double AngleErrorIteger;
		double AngleErrorDerivate;
		double AngleErrorLast;
		double kp_Angle,ki_Angle,kd_Angle;
		double PID_output;
		double TargetSpeed_output;
		double TargetSpeed_output_reset;
	
} AutoCarInfo;






//--------------set����--------------

// ��ȡ��ǰƫ���� yawCurrent���޲������Ӵ�������ȡ��
void setYawCurrent(AutoCarInfo* car);

// ����Ŀ��ƫ���� yawtarget
void setYawTarget(AutoCarInfo* car, float yaw,double speedRight,double speedLeft);

// �����ҵ��ת��---�ٶȻ��������� motorSpeedRight
void setMotorSpeedRight(AutoCarInfo* car, double speed);

// ��������ת��---�ٶȻ��������� motorSpeedLeft
void setMotorSpeedLeft(AutoCarInfo* car, double speed);

// ���ýǶ���� AngleError
void setAngleError(AutoCarInfo* car, double error);

// ���û������ AngleErrorIteger
void setAngleErrorIteger(AutoCarInfo* car, double errorIteger);

// ����΢����� AngleErrorDerivate
void setAngleErrorDerivate(AutoCarInfo* car, double errorDerivate);

// ������һ�νǶ���� AngleErrorLast
void setAngleErrorLast(AutoCarInfo* car, double errorLast);

// ����PID�ǶȻ����� kp_Angle
void setKpAngle(AutoCarInfo* car, double kp);

// ����PID�ǶȻ����� ki_Angle
void setKiAngle(AutoCarInfo* car, double ki);

// ����PID�ǶȻ����� kd_Angle
void setKdAngle(AutoCarInfo* car, double kd);

// ����PID��� PID_output
void setPIDOutput(AutoCarInfo* car, double output);

// ����Ŀ���ٶ� TargetSpeed_output
void setTargetSpeedOutput(AutoCarInfo* car, double speed);

// �������ú��Ŀ���ٶ� TargetSpeed_output_reset
void setTargetSpeedOutputReset(AutoCarInfo* car, double speed);

// ����PID����
void setAutoCarInfoPID_para(AutoCarInfo* car, double kp,double ki,double kd) ;

// ���������
void setAngleError_para(AutoCarInfo* car, double error, double errorInteger, double errorDerivate, double errorLast);

// PID��ʼ��
void PID_Angle_init(AutoCarInfo* car);

//PID�ǶȻ�

void PID_Angle_loop(AutoCarInfo* car);

//PID�ǶȻ�

void PID_Angle_loop(AutoCarInfo* car);

//��̽Ƕ�·�߼���
double calculateShortestAngleDifference(double current_angle, double target_angle) ;

#endif
