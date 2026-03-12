#include "MKL25Z4.h"
#include "servo.h"
#include "ultrasonic.h"
#include "delay.h"
#include <stdio.h>

#define SIDE_THRESH_CM   10
#define BACK_THRESH_CM   10

int main(void)
{
    int16_t angle = 0;
    int8_t direction = 1;

    uint32_t dist_s1 = 0, dist_s2 = 0, dist_s3 = 0;
    uint8_t priority_sensor = 0;

    Ultrasonic_InitAll();
    Servo_Init();

    printf("System start\r\n");
    printf("S1=PTD2/PTD3  S2=PTD4/PTD5  S3=BACK\r\n");

    while (1)
    {
        /* ---------- SERVO MOTION ---------- */
        angle += 2 * direction;

        if (angle >= 180) direction = -1;
        if (angle <= 0)   direction =  1;

        Servo1_SetAngle(angle);
        Servo2_SetAngle(180 - angle);

        /* ---------- SENSOR SAMPLING ---------- */
        static uint8_t sample_div = 0;
        sample_div++;

        if (sample_div >= 10)
        {
            sample_div = 0;

            dist_s1 = Ultrasonic_MeasureCm_Left();
            //Delay(1);

            dist_s2 = Ultrasonic_MeasureCm_Right();
            //Delay(1);

            dist_s3 = Ultrasonic_MeasureCm_Back();

            /* ---------- PRIORITY LOGIC ---------- */
            /* 3 = back sensor highest */

            if (dist_s3 > 0 && dist_s3 < BACK_THRESH_CM)
            {
                priority_sensor = 3;
            }
            else if ((dist_s1 > 0 && dist_s1 < SIDE_THRESH_CM) &&
                     (dist_s2 > 0 && dist_s2 < SIDE_THRESH_CM))
            {
                if (dist_s1 < dist_s2)
                    priority_sensor = 1;
                else
                    priority_sensor = 2;
            }
            else if (dist_s1 > 0 && dist_s1 < SIDE_THRESH_CM)
            {
                priority_sensor = 1;
            }
            else if (dist_s2 > 0 && dist_s2 < SIDE_THRESH_CM)
            {
                priority_sensor = 2;
            }
            else
            {
                priority_sensor = 0;
            }

            /* ---------- TERMINAL OUTPUT ---------- */
            printf("S1:%3lu cm  S2:%3lu cm  S3:%3lu cm  Priority:%u\r\n",
                   (unsigned long)dist_s1,
                   (unsigned long)dist_s2,
                   (unsigned long)dist_s3,
                   priority_sensor);
        }

        Delay(2);
    }
}
