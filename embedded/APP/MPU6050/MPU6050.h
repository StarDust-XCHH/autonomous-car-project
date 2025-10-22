#ifndef __MPU6050_H
#define __MPU6050_H


int MPU6050_DMP_Init(void);
int MPU6050_DMP_Get_Data(float *pitch,float *roll,float *yaw);
float normalize_angle_0_360(float angle);

#endif
