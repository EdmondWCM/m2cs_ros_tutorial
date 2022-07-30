#include <Arduino.h>
#include "dji.h"
#include <Encoder.h>
// control loop limit for safe
// DO NOT MODIFY UNTIL YOU ARE TOLD TO DO SO !
#define MAX_VEL 5000       // maximum velocity +- 2000 rpm
#define MAX_CUR 1024       // maximum current +-2.5 A
#define MAX_CUR_CHANGE 1024 // limit the change in current
#define MAX_POS 500000      // maximum position 100000 count
#define MIN_POS -500000     // maximum position 100000 count

// For testing max speed achieved by m3508 for pendulum
// int max_speed = 0;
// int32_t cur_enc;

// control loop parameters
#define P_KP 128
#define P_KD 1024
#define V_KP 256

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

long long time = 0;

Control_Mode ctrl_mode;
int32_t ctrl_target;
int32_t iout;
int32_t exp_time;
int32_t pstart;
// bool enable_balance = 0;
// This function will print out the feedback on the serial monitor
void print_feedback()
{
    // if (time < exp_time)
    // {
        Serial.print(dji_fb.enc);
        Serial.print(" ");
        Serial.print(dji_fb.rpm);
        Serial.print(" ");
        Serial.println(dji_fb.cur);
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
        ctrl_mode = MODE_CUR;
        ctrl_target = constrain(val, -MAX_CUR, MAX_CUR);

        break;
    case 'v':
        ctrl_mode = MODE_VEL;
        ctrl_target = constrain(val, -MAX_VEL, MAX_VEL);

        break;
    case 'p':
        ctrl_mode = MODE_POS;
        time = 0;
        ctrl_target = constrain(val, MIN_POS, MAX_POS);
        exp_time = val2;
        pstart = dji_fb.enc;
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
    float pdes, perr, dperr, pexp;
    // desired output, error of output, expected velocity
    float vdes, verr, vexp;
    // desired current output
    float ides;
    // with 'static' keyword, these values will retent after this function returns
    static float prev_perr; // error of position loop in the previous loop cycle
    static float prev_iout; // current output the previous loop cycle

    // DO IT YOURSELF
    // use dji_fb.enc or dji_fb.rpm to calculate required current
    // such that the motor can move in the desired way given by ctrl_mode and ctrl_target

    if (ctrl_mode == MODE_CUR)
    {
        // MODE_CUR
        ides = ctrl_target;
    }
    else
    {
        if (ctrl_mode == MODE_VEL)
        {
            // MODE_VEL
            vdes = ctrl_target;
        }
        else
        {


            pdes = ctrl_target;
            const float pdiff = pdes - pstart;
            if (time <= exp_time)
            {  
                // formular * pdiff + pstart -> getting the correst s-t graph
                pexp = ((3 / (exp_time * exp_time)) * time * time + (-2 / (exp_time * exp_time * exp_time)) * time * time * time) * (pdiff) + pstart;
                vexp = ((6 * (pdiff)) / (exp_time * exp_time)) * time + ((-6 * (pdiff)) / (exp_time * exp_time * exp_time)) * time * time;
                // hz of the program = 1000
                // turning the unit from pos/ms to rpm
                vexp = (vexp * 1000 * 60)/8191;
                perr = pexp - dji_fb.enc;
                vdes = P_KP * perr + vexp;
                vdes = constrain(vdes, -MAX_VEL, MAX_VEL);
                time++;
            }

            // if time > exp_time, set output = 0
            if (time > exp_time)
            {
                vdes = dji_fb.rpm;
            }

            //<-------old code--------> (for debug only)
            // perr = pdes - dji_fb.enc;
            // dperr = perr - prev_perr;
            // prev_perr = perr;
            // vdes = P_KP * perr + P_KD * dperr;
            // vdes /= 128;
            // vdes = constrain(vdes, -MAX_VEL, MAX_VEL);
        }

        // MODE_VEL and MODE_POS will go through this:
        // Task 5b.2 - velocity control loop
        // 1. find verr, the error between desired velocity (vdes) and actual velocity (dji_fb.rpm)
        // 2. calculate ides with V_KP * verr / 128
        // 3. limit ides with MAX_CUR using constrain()
        // TYPE YOUR CODE HERE:
        verr = vdes - dji_fb.rpm;
        ides = V_KP * verr;
        ides /= 128;
        ides = constrain(ides, -MAX_CUR, MAX_CUR);
    }

    // Task 5b.1 - limit the change in current (do it first)
    // 1. find the difference between the desired current output(ides) and previous current output (prev_iout)
    // 2. limit the difference with MAX_CUR_CHANGE using constrain()
    // 3. apply the limited change to the previous current output (prev_iout)
    // 4. prev_iout is now the calculated current, assign it to iout
    // TYPE YOUR CODE HERE:
    prev_iout = constrain(ides - prev_iout, -MAX_CUR_CHANGE, MAX_CUR_CHANGE);
    iout = prev_iout;

    return iout; // return the calculated current output
}

void setup()
{
    while (!Serial); // wait serial ready
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
