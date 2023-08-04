//
// Created by ShiF on 2023/8/4.
//

#ifndef GREEN_FIND_SERVO_H
#define GREEN_FIND_SERVO_H
typedef struct {
    float asis_x;
    float asis_y;
} data;
extern data servo_tar;

void servo_PID(void);
void servo_Init(void);

#endif //GREEN_FIND_SERVO_H
