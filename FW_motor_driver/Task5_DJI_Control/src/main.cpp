#include <Arduino.h>
#include "dji.h"
#include <Encoder.h>
// control loop limit for safe
// DO NOT MODIFY UNTIL YOU ARE TOLD TO DO SO !
#define MAX_VEL 5000        // maximum velocity +- 2000 rpm
#define MAX_CUR 2048        // maximum current +-2.5 A
#define MAX_CUR_CHANGE 1024 // limit the change in current
#define MAX_POS 500000      // maximum position 100000 count
#define MIN_POS -500000     // maximum position 100000 count

// For testing max speed achieved by m3508 for pendulum
// int max_speed = 0;
// int32_t cur_enc;

// control loop parameters
#define P_KP 0.8
#define P_KD 1024
#define V_KP 512

// Pendulum's PID
// #define kP 4.5
// #define kD 8.5
// #define multiplier 4

// Create a encoder object
// It will be used to get the position of the rod when operating pendulum
// Encoder myEnc(3, 4);

enum Control_Mode
{
    MODE_CUR, // value = 0
    MODE_VEL, // value = 1
    MODE_POS  // value = 2
};

double t_current = 0;

Control_Mode ctrl_mode;
double ctrl_target;
double i_out;
double t_final;
double p_init;
long double v_init;
double p_exp = 0;
double v_exp = 0;
double vt;
double p_diff;
// bool enable_balance = 0;
// This function will print out the feedback on the serial monitor
void print_feedback()
{
    // if (t_current < t_final)
    // {
    Serial.print(dji_fb.enc);
    Serial.print(" ");
    Serial.print(dji_fb.rpm);
    Serial.print(" ");
    Serial.print(dji_fb.cur);
    Serial.print(" ");
    Serial.print((long)p_exp);
    Serial.print(" ");
    Serial.print((long)v_exp);
    Serial.print(" ");
    Serial.print((long)v_init);
    Serial.print(" ");
    Serial.print((long)p_diff);
    Serial.print(" ");
    Serial.println(millis());

    // }
}
// This function will get the command from the user and update corresbonding configurations
// General tasks of this function:
// 1. Read a line from serial monitor
// 2. Update ctrl_mode, ctrl_target, enable_feedback
void get_command()
{
    static char cmd_buf[32];
    static uint8_t cmd_buf_next = 0;
    char cmd;
    int32_t val;
    int32_t val2;

    if (Serial.available() == 0)
        return;
    char received = Serial.read();
    cmd_buf[cmd_buf_next++] = received;

    if (received == '\b')
    {
        cmd_buf_next = max(0, cmd_buf_next - 2);
        return;
    }
    else if (received == '\n')
    {
        cmd_buf[cmd_buf_next - 2] = '\0';
        cmd_buf_next = 0;
    }
    else
    {
        return;
    }

    int noOfField = sscanf(cmd_buf, "%c %ld %ld", &cmd, &val, &val2);
    if (!((noOfField == 3 && (cmd == 'p')) || (noOfField == 2 && (cmd == 'i' || cmd == 'v')) || (cmd == 'f')))
    {
        Serial.write("returned");
        return;
    }

    switch (cmd)
    {
    case 'i':
        t_current = 0;
        ctrl_mode = MODE_CUR;
        ctrl_target = constrain(val, -MAX_CUR, MAX_CUR);

        break;
    case 'v':
        t_current = 0;
        ctrl_mode = MODE_VEL;
        ctrl_target = constrain(val, -MAX_VEL, MAX_VEL);

        break;
    case 'p':
        ctrl_mode = MODE_POS;
        t_current = 0;
        ctrl_target = constrain(val, MIN_POS, MAX_POS);
        t_final = val2;
        p_init = dji_fb.enc;
        v_init = dji_fb.rpm;
        v_init = (v_init * 8191.0)/(1000.0*60.0);
        p_diff = ctrl_target - p_init;
        vt = (v_init/p_diff)*t_final;

        break;
    case 'f':

        break;
    default:
        break;
    }
    print_feedback();
}

// This part is for control the motor(eg. position, speed, current)
int32_t control()
{
    // desired output, error of output, change in output error, expected output
    double p_des, p_err;
    // desired output, error of output, expected velocity
    double v_des, v_err;
    // desired current output
    double i_des;
    static double prev_i_out; // current output the previous loop cycle

    // DO IT YOURSELF
    // use dji_fb.enc or dji_fb.rpm to calculate required current
    // such that the motor can move in the desired way given by ctrl_mode and ctrl_target

    if (ctrl_mode == MODE_CUR)
    {
        // MODE_CUR
        i_des = ctrl_target;
    }
    else
    {
        if (ctrl_mode == MODE_VEL)
        {
            // MODE_VEL
            v_des = ctrl_target;
        }
        else
        {

            p_des = ctrl_target;
            if (t_current <= t_final)
            {
                // formular * p_diff + p_init -> getting the correst s-t graph
                p_exp = (((v_init*t_current)/p_diff)+(((3.0-2.0*vt) / (t_final * t_final)) * t_current * t_current )- (((2.0 - 1.0*vt) / (t_final * t_final * t_final)) * t_current * t_current * t_current)) * (p_diff) + p_init;
                v_exp = ((v_init/p_diff)+(((6.0-4.0*vt)/(t_final*t_final))*t_current)-(((6.0-3.0*vt)/(t_final*t_final*t_final))*(t_current * t_current)))*p_diff;
                v_exp = (v_exp * 1000 * 60) / 8191;
                p_err = p_exp - dji_fb.enc;
                v_des = P_KP * p_err + v_exp;
                v_des = constrain(v_des, -MAX_VEL, MAX_VEL);
                t_current++;
            }

            //<-------old code--------> (for debug only)
            // p_err = p_des - dji_fb.enc;
            // dp_err = p_err - prev_p_err;
            // prev_p_err = p_err;
            // v_des = P_KP * p_err + P_KD * dp_err;
            // v_des /= 128;
            // v_des = constrain(v_des, -MAX_VEL, MAX_VEL);
        }

        // MODE_VEL and MODE_POS will go through this:
        // Task 5b.2 - velocity control loop
        // 1. find v_err, the error between desired velocity (v_des) and actual velocity (dji_fb.rpm)
        // 2. calculate i_des with V_KP * v_err / 128
        // 3. limit i_des with MAX_CUR using constrain()
        // TYPE YOUR CODE HERE:
        v_err = v_des - dji_fb.rpm;
        i_des = V_KP * v_err;
        i_des /= 128;
        i_des = constrain(i_des, -MAX_CUR, MAX_CUR);
    }

    // Task 5b.1 - limit the change in current (do it first)
    // 1. find the difference between the desired current output(i_des) and previous current output (prev_i_out)
    // 2. limit the difference with MAX_CUR_CHANGE using constrain()
    // 3. apply the limited change to the previous current output (prev_i_out)
    // 4. prev_i_out is now the calculated current, assign it to i_out
    // TYPE YOUR CODE HERE:
    prev_i_out = constrain(i_des - prev_i_out, -MAX_CUR_CHANGE, MAX_CUR_CHANGE);
    i_out = prev_i_out;
    // if t_current > t_final, set output = 0
    if (t_current > t_final)
    {
        i_out = 0;
    }

    return i_out; // return the calculated current output
}

void setup()
{
    while (!Serial)
        ; // wait serial ready
    Serial.begin(1000000);
    Serial.flush();
    dji_init();
    // Serial.println("DJI Init Done");
}

void loop()
{
    // Do not change anything here
    get_command();

    if (dji_get_feedback())
    {
        dji_set_current(control());
    }
}
