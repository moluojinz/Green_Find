//
// Created by ShiF on 2023/8/4.
//

#include "servo.h"
#include "A_headfile.h"

/**
 * PD13-->TIM4_CH2  upper
 * PD12-->TIM4_CH1  base
 */

PID_t servo_go;
data servo_tar;     //存储目标值
/**
 * 控制，输入参数   (X,Y)  单位：mm
 * X            --->base        横向偏移
 * Y            --->upper       纵向偏移
 * dis_default  --->初始距离，激光灯距目标点的距离
 * L_default    --->舵机臂长
 */
#define dis_default 1000
#define L_default   50
float initial_x = 1350;
float initial_y = 1100;
#define ServoUpper_max  (initial_x+300)
#define ServoUpper_min  (initial_x-300)
#define ServoBase_max   (initial_y+300)
#define ServoBase_min   (initial_y-300)
double long_angle, tran_angle;    //纵向角度，横向角度

void Red_Servo(float x, float y) {
    //计算纵向角度
//    double b_parm = (-dis_default * L_default +
//                     sqrt(pow(dis_default, 2) * pow(L_default, 2) + y * pow(L_default, 2) * (2 * L_default + y))) /
//                    (2 * L_default + y);
    long_angle = atan2(y, dis_default);
    long_angle = (180.0 / M_PI) * long_angle;
    //计算横向角度
    tran_angle = acos((2 * pow(dis_default, 2) - pow(x, 2)) / (2 * pow(dis_default, 2)));
    tran_angle = (180.0 / M_PI) * tran_angle;

    static double long_pram = 2500.0 / 180.0;
    static double tran_pram = 2500.0 / 270.0;

    long_angle += -long_pram * long_angle ;
    if (x < 0) tran_angle = (-1) * tran_angle;
    tran_angle += -tran_pram * tran_angle ;

    if (long_angle >= ServoUpper_max) long_angle = ServoUpper_max;
    if (long_angle <= ServoUpper_min) long_angle = ServoUpper_min;
    if (tran_angle >= ServoBase_max) tran_angle = ServoBase_max;
    if (tran_angle <= ServoBase_min) tran_angle = ServoBase_min;

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, long_angle);//Red upper
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, tran_angle);//Red base
}
/**
 *  舵机追迹函数
 * @param now   当前servo坐标值
 * @param tar   目标servo坐标值
 * @param path  距离步进  单位  cm per path
 * @return  是否达到目标值，达到则返回OK，没达到则返回NOTYET
 */
void servo_control( float path) {
    static float distance, x_offset, y_offset, y_path, x_path, cnt;
    distance = servo_go.PID_Out;
    cnt = fabsf(distance / path);    //计算步进次数
    y_path = servo_tar.asis_y  / cnt;           //计算每次步进
    x_path = servo_tar.asis_x / cnt;
    x_offset =  x_path;
    y_offset =  y_path;


    Red_Servo(x_offset, y_offset);

}

void servo_Init(void) {
    PID_SpeedParamInit(&servo_go);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, initial_y);//Red upper
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, initial_x);//Red base
}

void servo_PID(void) {
    servo_go.Kp1 = 0.0;
    servo_go.Ki1 = 0.0;
    servo_go.Kd1 = 0.0;
    uint8_t distance = sqrtf(powf(servo_tar.asis_y, 2) + powf(servo_tar.asis_x, 2));
    servo_go.PID_Target = distance;

    PID_Update(&servo_go, (float) 0);
    PID_GetPositionPID(&servo_go);
}

