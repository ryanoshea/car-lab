#include <device.h>
#include <stdio.h>
#include <stdlib.h>

/* Variables and Constants for Speed Control */
uint32 MAX_TIME = 65535; //Counter period
uint32 FREQ = 100; //Hz
uint32 lastTimeSeen[5]; // timestamp of each wheel magnet's last pass over Hall effect sensor
uint16 FORWARD = 999;
uint16 BACKWARD = 0;
uint8  dir_flag;
float CIRCUMFERENCE = 0.66; // feet, of each wheel
float SETPOINT = 3.5; // feet per second
int currentMagnet;
float speed;

/* PID Controller constants and variables */
float epsilon = 0.01;
float Kp = 65;
float Ki = 10;
float Kd = .07;
float MAX_OUT = 700;
float MIN_OUT = 0;
float preError;
float integral;
float openloop;
float speed_eps = 0.05;
float battery_scale = 0.95;

// Carries out PID calculations and updates motor control output
float PIDCtrl(float wantedSpeed, float actualSpeed)/*, float dt)*/
{
    float error = wantedSpeed - actualSpeed;
    float derivative;
    float output;

    // Integrate error, only if large enough
    float abserror;
    if (error >= 0) abserror = error;
    else abserror = -1*error;
    if (abserror > epsilon)
    {
        integral += error;
    }

    // calculate derivative component
    derivative = error - preError;

    // calculate output level via PID formula
    output = openloop + Kp*error + Ki*integral + Kd*derivative;

    // Saturation filter
    if (output > MAX_OUT)
    {
        output = MAX_OUT;
    }
    if (output < MIN_OUT)
    {
        output = MIN_OUT;
    }

    preError = error; // save error for next round's derivative component

    return output;
}

// Service a wheel magnet passing over the Hall effect sensor
CY_ISR(magnet)
{
    uint32 nextTime = SpeedTimer_ReadCounter(); // Counter value of this pass over the sensor
    uint32 prevTime = lastTimeSeen[currentMagnet]; // Counter value from this magnet's last pass over the sensor (1 wheel revolution ago)

    float dt = (prevTime - nextTime)/(float)FREQ; // Time elapsed in the last revolution

    lastTimeSeen[currentMagnet] = nextTime;

    // Correct for timer overflow
    if (nextTime > prevTime)
    {
        nextTime = nextTime - MAX_TIME;
    }

    speed = (CIRCUMFERENCE / dt); // Calculate car's current speed

    // Update which magnet will next trigger this interrupt
    currentMagnet++;
    if (currentMagnet == 5)
    {
        currentMagnet = 0;
    }
}

// Detect when the car has stalled and act accordingly
CY_ISR(stall)
{
    uint32 currentTime = SpeedTimer_ReadCounter();
    uint32 idx;
    uint32 oldTime;
    float pidOutput;

    if (currentMagnet == 0) idx = 5;
    else idx = currentMagnet;
    oldTime = lastTimeSeen[idx - 1];

    if ((oldTime - currentTime)/FREQ > 0.25) // Car has stalled; mark speed as 0
    {
        pidOutput = PIDCtrl(SETPOINT, (float)0);
        Motor_PWM_WriteCompare1((uint16) pidOutput);
    }
    else // Normal PWM update
    {
        pidOutput = PIDCtrl(SETPOINT, speed);
        Motor_PWM_WriteCompare1((uint16) pidOutput);
    }
}

/* Variables for Ping sonar sensors */
float  SOUND_SPEED_IN_AIR = 346.5;
float  DIST_FREQ = 1000000.;
uint32 MAX_INT = 4294967295;
uint16 MAX_16 = 65535;
uint16 W_OFFSET = 1300;
uint16 Y_OFFSET = 19300;
uint16 CIRCLE = 360;
float  bins[20];
uint16 DIV_NUM = 20;
uint16 NUM_BINS = 18;
uint16 current_y_bin;
uint16 current_w_bin;
uint16 prev_y_bin;
uint16 prev_w_bin;
uint16 y_counts;
uint16 w_counts;
float  y_accum_dist;
float  w_accum_dist;

// Service rangefinder reading from sonar 1
CY_ISR(ping1) /* White Sensor */
{
    uint16 platform = Platform_Timer_ReadCounter();
    uint32 junk = Ping1_Timer_ReadCapture();
    uint32 junk2 = Ping1_Timer_ReadCapture();
    uint32 frontEdgeTime = Ping1_Timer_ReadCapture();
    uint32 endEdgeTime = Ping1_Timer_ReadCapture();
    uint32 timeDiff = frontEdgeTime - endEdgeTime;
    float distance;
    char strbuffer[15];
    uint16 degree;

    distance = (SOUND_SPEED_IN_AIR * ((float) timeDiff / DIST_FREQ))/(float) 2;
    degree = MAX_16-platform+W_OFFSET;
    degree /= 100;
    if (degree > CIRCLE)
        degree -= CIRCLE;

    current_w_bin = degree/DIV_NUM;
    if (current_w_bin != prev_w_bin)
    {
        bins[prev_w_bin] = w_accum_dist/w_counts;
        prev_w_bin = current_w_bin;
        w_accum_dist = distance;
        w_counts = 1;
    }
    else
    {
        w_accum_dist += distance;
        w_counts++;
    }

    if (degree <= CIRCLE)
        bins[current_w_bin] = distance;
}

// Service rangefinder reading from sonar 2
CY_ISR(ping2) /* Yellow Sensor */
{
    uint16 platform = Platform_Timer_ReadCounter();
    uint32 junk = Ping2_Timer_ReadCapture();
    uint32 junk2 = Ping2_Timer_ReadCapture();
    uint32 frontEdgeTime = Ping2_Timer_ReadCapture();
    uint32 endEdgeTime = Ping2_Timer_ReadCapture();
    uint32 timeDiff = frontEdgeTime - endEdgeTime;
    float distance;
    char strbuffer[15];
    uint16 degree;

    distance = (SOUND_SPEED_IN_AIR * ((float) timeDiff / DIST_FREQ))/(float) 2;
    degree = MAX_16-platform+Y_OFFSET;
    degree /= 100;
    if (degree > CIRCLE)
        degree -= CIRCLE;

    current_y_bin = degree/DIV_NUM;
    if (current_y_bin != prev_y_bin)
    {
        bins[prev_y_bin] = y_accum_dist/y_counts;
        prev_y_bin = current_y_bin;
        y_accum_dist = distance;
        y_counts = 1;
    }
    else
    {
        y_accum_dist += distance;
        y_counts++;
    }

    if (degree <= CIRCLE)
        bins[degree/DIV_NUM] = distance;
}

uint16 RIGHT = 190;
uint16 SLIGHT_RIGHT = 170;
uint16 SLIGHT_LEFT  = 130;
uint16 LEFT = 110;
uint16 MIDDLE = 153;
float  MIN_DISTANCE = 1.0;
float  MED_DISTANCE = 0.65;
float  TURN_DIST = 0.4;
float  ABSMIN_DISTANCE  = 0.4;
uint8  any_seen;

// Detect obstacles, map them in space, prioritize based on distance,
// and update steering and motor controls based on detected obstacles
CY_ISR(platform)
{
    float closest_dist  = 10.0;
    float closest_dist2 = 10.0;
    uint16 closest_bin, closest_bin2;
    int i;
    char strbuffer[15];

    for (i = 0; i < NUM_BINS; i++)
    {
        if (bins[i] < closest_dist)
        {
            if ((i != closest_bin-1) && (i != closest_bin+1))
            {
                closest_dist2 = closest_dist;
                closest_bin2 = closest_bin;
            }
            closest_dist = bins[i];
            closest_bin  = i;
        }
    }

    /* Forward/Backward adjustment */
    if (closest_dist < MIN_DISTANCE)
    {
        uint16 effective_bin;
        any_seen = 1;

        if (closest_dist2 < MIN_DISTANCE)
        {
            float dist_sum = closest_dist + closest_dist2;
            if ((closest_bin >= 14) && (closest_bin <= 17)) // One point in left forward quadrant
            {
                if ((closest_bin2 >= 0) && (closest_bin2 <= (closest_bin - 9))) // Second point in (generally) front right
                {
                    effective_bin = ((uint16)(((closest_dist2/dist_sum)*(float)closest_bin + (closest_dist/dist_sum)*(float)(closest_bin2 + 18)))) % 18;
                }
                else
                    effective_bin = (closest_bin + closest_bin2)/2;
            }
            else if ((closest_bin2 >= 14) && (closest_bin2 <= 17)) // One point in left forward quadrant
            {
                if ((closest_bin >= 0) && (closest_bin <= (closest_bin2 - 9))) // Second point in (generally) front right
                {
                    effective_bin = ((uint16)(((closest_dist/dist_sum)*(float)closest_bin2 + (closest_dist2/dist_sum)*(float)(closest_bin + 18)))) % 18;
                }
                else
                    effective_bin = (closest_bin + closest_bin2)/2;
            }
            else if ((closest_bin >= 0) && (closest_bin <= 4)) // One point in right forward quadrant
            {
                if (closest_bin2 >= (closest_bin+9)) // Second point in (generally) front left
                {
                    effective_bin = ((uint16)(((closest_dist2/dist_sum)*(float)(closest_bin+18) + (closest_dist/dist_sum)*(float)closest_bin2))) % 18;
                }
                else
                    effective_bin = (closest_bin + closest_bin2)/2;
            }
            else if ((closest_bin2 >= 0) && (closest_bin2 <= 4)) // One point in right forward quadrant
            {
                if (closest_bin >= (closest_bin2+9)) // Second point in (generally) front left
                {
                    effective_bin = ((uint16)(((closest_dist/dist_sum)*(float)(closest_bin2+18) + (closest_dist2/dist_sum)*(float)closest_bin))) % 18;
                }
                else
                    effective_bin = (uint16)((closest_dist2/dist_sum)*(float)closest_bin + (closest_dist/dist_sum)*(float)closest_bin2);
            }
            else
                effective_bin = (uint16)((closest_dist2/dist_sum)*(float)closest_bin + (closest_dist/dist_sum)*(float)closest_bin2);
        }
        else
            effective_bin = closest_bin;
        if (effective_bin >= NUM_BINS)
            effective_bin -= NUM_BINS;

        if (closest_dist < ABSMIN_DISTANCE)
            SETPOINT = 3.5*battery_scale;
        else if (closest_dist < MED_DISTANCE)
            SETPOINT = 3.0*battery_scale;
        else
            SETPOINT = 2.5*battery_scale;
        openloop = 40*SETPOINT;

        switch(effective_bin)
        {
            case 0:
                Steering_PWM_WriteCompare(RIGHT);
                Motor_PWM_WriteCompare2(BACKWARD);
                if (dir_flag == 0)
                {
                    SETPOINT = 3.0*battery_scale;
                    openloop = 50*SETPOINT;
                    preError = 0.0;
                    integral = 2*SETPOINT;
                    dir_flag = 1;
                }
                break;
            case 1:
                Steering_PWM_WriteCompare(RIGHT);
                Motor_PWM_WriteCompare2(BACKWARD);
                if (dir_flag == 0)
                {
                    SETPOINT = 3.0*battery_scale;
                    openloop = 50*SETPOINT;
                    preError = 0.0;
                    integral = 2*SETPOINT;
                    dir_flag = 1;
                }
                break;
            case 2:
                if (closest_dist < TURN_DIST)
                {
                    Steering_PWM_WriteCompare(RIGHT);
                    Motor_PWM_WriteCompare2(BACKWARD);
                    if (dir_flag == 0)
                    {
                        SETPOINT = 3.0*battery_scale;
                        openloop = 50*SETPOINT;
                        preError = 0.0;
                        integral = 2*SETPOINT;
                        dir_flag = 1;
                    }
                }
                else
                {
                    Steering_PWM_WriteCompare(SLIGHT_LEFT);
                    Motor_PWM_WriteCompare2(FORWARD);
                    if (dir_flag)
                    {
                        SETPOINT = 3.0*battery_scale;
                        openloop = 50*SETPOINT;
                        preError = 0.0;
                        integral = 2*SETPOINT;
                        dir_flag = 0;
                    }
                }
                break;
            case 3:
                if (closest_dist < TURN_DIST)
                {
                    Steering_PWM_WriteCompare(RIGHT);
                    Motor_PWM_WriteCompare2(BACKWARD);
                    if (dir_flag == 0)
                    {
                        SETPOINT = 3.0*battery_scale;
                        openloop = 50*SETPOINT;
                        preError = 0.0;
                        integral = 2*SETPOINT;
                        dir_flag = 1;
                    }
                }
                else
                {
                    Steering_PWM_WriteCompare(SLIGHT_LEFT);
                    Motor_PWM_WriteCompare2(FORWARD);
                    if (dir_flag)
                    {
                        SETPOINT = 3.0*battery_scale;
                        openloop = 50*SETPOINT;
                        preError = 0.0;
                        integral = 2*SETPOINT;
                        dir_flag = 0;
                    }
                }
                break;
            case 4:
                Steering_PWM_WriteCompare(SLIGHT_LEFT);
                Motor_PWM_WriteCompare2(FORWARD);
                if (dir_flag)
                {
                    SETPOINT = 3.0*battery_scale;
                    openloop = 50*SETPOINT;
                    preError = 0.0;
                    integral = 2*SETPOINT;
                    dir_flag = 0;
                }
                break;
            case 5:
                Steering_PWM_WriteCompare(MIDDLE);
                Motor_PWM_WriteCompare2(FORWARD);
                if (dir_flag)
                {
                    SETPOINT = 3.0*battery_scale;
                    openloop = 50*SETPOINT;
                    preError = 0.0;
                    integral = 2*SETPOINT;
                    dir_flag = 0;
                }
                break;
            case 6:
                Steering_PWM_WriteCompare(MIDDLE);
                Motor_PWM_WriteCompare2(FORWARD);
                if (dir_flag)
                {
                    SETPOINT = 3.0*battery_scale;
                    openloop = 50*SETPOINT;
                    preError = 0.0;
                    integral = 2*SETPOINT;
                    dir_flag = 0;
                }
                break;
            case 7:
                Steering_PWM_WriteCompare(MIDDLE);
                Motor_PWM_WriteCompare2(FORWARD);
                if (dir_flag)
                {
                    SETPOINT = 3.0*battery_scale;
                    openloop = 50*SETPOINT;
                    preError = 0.0;
                    integral = 2*SETPOINT;
                    dir_flag = 0;
                }
                break;
            case 8:
                Steering_PWM_WriteCompare(MIDDLE);
                Motor_PWM_WriteCompare2(FORWARD);
                if (dir_flag)
                {
                    SETPOINT = 3.0*battery_scale;
                    openloop = 50*SETPOINT;
                    preError = 0.0;
                    integral = 2*SETPOINT;
                    dir_flag = 0;
                }
                break;
            case 9:
                Steering_PWM_WriteCompare(MIDDLE);
                Motor_PWM_WriteCompare2(FORWARD);
                if (dir_flag)
                {
                    SETPOINT = 3.0*battery_scale;
                    openloop = 50*SETPOINT;
                    preError = 0.0;
                    integral = 2*SETPOINT;
                    dir_flag = 0;
                }
                break;
            case 10:
                Steering_PWM_WriteCompare(MIDDLE);
                Motor_PWM_WriteCompare2(FORWARD);
                if (dir_flag)
                {
                    SETPOINT = 3.0*battery_scale;
                    openloop = 50*SETPOINT;
                    preError = 0.0;
                    integral = 2*SETPOINT;
                    dir_flag = 0;
                }
                break;
            case 11:
                Steering_PWM_WriteCompare(MIDDLE);
                Motor_PWM_WriteCompare2(FORWARD);
                if (dir_flag)
                {
                    SETPOINT = 3.0*battery_scale;
                    openloop = 50*SETPOINT;
                    preError = 0.0;
                    integral = 2*SETPOINT;
                    dir_flag = 0;
                }
                break;
            case 12:
                Steering_PWM_WriteCompare(MIDDLE);
                Motor_PWM_WriteCompare2(FORWARD);
                if (dir_flag)
                {
                    SETPOINT = 3.0*battery_scale;
                    openloop = 50*SETPOINT;
                    preError = 0.0;
                    integral = 2*SETPOINT;
                    dir_flag = 0;
                }
                break;
            case 13:
                Steering_PWM_WriteCompare(SLIGHT_RIGHT);
                Motor_PWM_WriteCompare2(FORWARD);
                if (dir_flag)
                {
                    SETPOINT = 3.0*battery_scale;
                    openloop = 50*SETPOINT;
                    preError = 0.0;
                    integral = 2*SETPOINT;
                    dir_flag = 0;
                }
                break;
            case 14:
                if (closest_dist < TURN_DIST)
                {
                    Steering_PWM_WriteCompare(LEFT);
                    Motor_PWM_WriteCompare2(BACKWARD);
                    if (dir_flag == 0)
                    {
                        SETPOINT = 3.0*battery_scale;
                        openloop = 50*SETPOINT;
                        preError = 0.0;
                        integral = 2*SETPOINT;
                        dir_flag = 1;
                    }
                }
                else
                {
                    Steering_PWM_WriteCompare(SLIGHT_RIGHT);
                    Motor_PWM_WriteCompare2(FORWARD);
                    if (dir_flag)
                    {
                        SETPOINT = 3.0*battery_scale;
                        openloop = 50*SETPOINT;
                        preError = 0.0;
                        integral = 2*SETPOINT;
                        dir_flag = 0;
                    }
                }
                break;
            case 15:
                if (closest_dist < TURN_DIST)
                {
                    Steering_PWM_WriteCompare(LEFT);
                    Motor_PWM_WriteCompare2(BACKWARD);
                    if (dir_flag == 0)
                    {
                        SETPOINT = 3.0*battery_scale;
                        openloop = 50*SETPOINT;
                        preError = 0.0;
                        integral = 2*SETPOINT;
                        dir_flag = 1;
                    }
                }
                else
                {
                    Steering_PWM_WriteCompare(SLIGHT_RIGHT);
                    Motor_PWM_WriteCompare2(FORWARD);
                    if (dir_flag)
                    {
                        SETPOINT = 3.0*battery_scale;
                        openloop = 50*SETPOINT;
                        preError = 0.0;
                        integral = 2*SETPOINT;
                        dir_flag = 0;
                    }
                }
                break;
            case 16:
                Steering_PWM_WriteCompare(LEFT);
                Motor_PWM_WriteCompare2(BACKWARD);
                if (dir_flag == 0)
                {
                    SETPOINT = 3.0*battery_scale;
                    openloop = 50*SETPOINT;
                    preError = 0.0;
                    integral = 2*SETPOINT;
                    dir_flag = 1;
                }
                break;
            case 17:
                Steering_PWM_WriteCompare(RIGHT);
                Motor_PWM_WriteCompare2(BACKWARD);
                if (dir_flag == 0)
                {
                    SETPOINT = 3.0*battery_scale;
                    openloop = 50*SETPOINT;
                    preError = 0.0;
                    integral = 2*SETPOINT;
                    dir_flag = 1;
                }
                break;
        }
    }

    else if (any_seen)
    {
        SETPOINT = 2.5*battery_scale;
        openloop = 50*SETPOINT;
        Steering_PWM_WriteCompare(MIDDLE);
        if (dir_flag)
        {
            preError = 0.0;
            integral = 2*SETPOINT;
            Motor_PWM_WriteCompare2(FORWARD);
            dir_flag = 0;
        }
        any_seen = 0;
    }
}

// Send surroundings data over UART and xBee to a computer for plotting
CY_ISR(tx)
{
    int i;
    char strbuffer[15];
    for (i = 0; i < NUM_BINS; i++)
    {
        sprintf(strbuffer, "%d,%04.3f=", i, bins[i]);
        UART_PutString(strbuffer);
    }
    sprintf(strbuffer, "\n");
    UART_PutString(strbuffer);
}

void main()
{
    int i;
    float pidOutput;

    LCD_Start();

    /* Motor initializing */
    Motor_PWM_Start();
    SpeedTimer_Start();
    StallTimer_Start();
    CyGlobalIntEnable;
    inter_hall_Start();
    inter_hall_SetVector(magnet);
    inter_stall_Start();
    inter_stall_SetVector(stall);

    /* Steering initializing */
    Steering_PWM_Start();

    /* Obstacle detection initialization */
    Ping_PWM_Start();
    Ping1_Timer_Start();
    Ping2_Timer_Start();
    inter_ping1_Start();
    inter_ping1_SetVector(ping1);
    inter_ping2_Start();
    inter_ping2_SetVector(ping2);
    Ping_Motor_PWM_Start();
    Platform_Timer_Start();
    PlatInter_Counter_Start();
    inter_platform_Start();
    inter_platform_SetVector(platform);

    /* RS232 initialization */
    UART_Start();
    TX_Counter_Start();
    inter_tx_Start();
    inter_tx_SetVector(tx);

    speed = 0;
    currentMagnet = 0;
    preError = 0.0;
    integral = 0.0;
    openloop = 50*SETPOINT;

    /* initialize the array */
    for (i = 0; i < 5; i++)
    {
        lastTimeSeen[i] = MAX_TIME;
    }

    // Calculate the initial motor output
    pidOutput = PIDCtrl(SETPOINT, 0);

    // Change the PWM to be the pid output
    Motor_PWM_WriteCompare1((uint16) pidOutput);
    Motor_PWM_WriteCompare2(FORWARD);
    dir_flag = 0;

    for(;;)
    {
        ;
    }
}
