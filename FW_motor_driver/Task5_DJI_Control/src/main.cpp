#include <Arduino.h>
#include "dji.h"
#include <Encoder.h>
// control loop limit for safe
// DO NOT MODIFY UNTIL YOU ARE TOLD TO DO SO !
#define MAX_VEL 2000        // maximum velocity +- 2000 rpm
#define MAX_CUR 512         // maximum current +-2.5 A
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

int time = 0;

Control_Mode ctrl_mode;
int32_t ctrl_target;
int16_t iout;
int32_t exp_time;
int32_t pstart;
// bool enable_balance = 0;
// This function will print out the feedback on the serial monitor
void print_feedback()
{
        Serial.print(dji_fb.enc);
        // Serial.print("Pendulum ENC: ");
        // Serial.print(cur_enc);
        Serial.print(" ");
        Serial.print(dji_fb.rpm);
        Serial.print(" ");
        Serial.println(dji_fb.cur);
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

    // Task 5a.1
    // refer to Lab 4 simple calculator, read in a line of command
    // [optional] try to handle the backspace '\b' character this time
    // you can do it by decreasing the 'next' variable by 1
    // and use the max() function to ensure the 'next' is >= 0
    // this will allow you to delete a typo
    // TYPE YOUR CODE HERE:
    if (Serial.available() == 0)
        return;
    char received = Serial.read();
    // Serial.print(received);
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

    // Task 5a.2
    // - parse the line of command and extract the cmd word e.g. i/v/p/f
    //   and the value after it using sscanf() with format "%c %ld"
    // - continue only if the return value of sscanf() is 2
    //   this means two fields are successfully converted and assigned
    // - echo the command if it is a valid command
    // TYPE YOUR CODE HERE:
    int noOfField = sscanf(cmd_buf, "%c %ld %ld", &cmd, &val, &val2);
    // if (noOfField == 3)
    // {
    //     Serial.print("num of field = " + noOfField);
    //     Serial.print("cmd = " + cmd);
    //     Serial.print("pos = " + val);
    //     Serial.print("time = " + val2);
    // }

    if (!((noOfField == 3 && (cmd == 'p')) || (noOfField == 2 && (cmd == 'i' || cmd == 'v')) || (cmd == 'f')))
    {
        Serial.write("returned");
        return;
    }

    // Task 5a.3
    // limit the input value defined at the top using constrain()
    // update ctrl_mode, ctrl_target, enable_feedback accordingly
    // TYPE YOUR CODE HERE:
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
    // desired output, error of output, change in output error, expectied output
    long long pdes, perr, dperr, pexp;
    // desired output, error of output, expected velocity
    int32_t vdes, verr, vexp;
    // desired current output
    int32_t ides;
    int32_t exp_pos;
    // with 'static' keyword, these values will retent after this function returns
    static int32_t prev_perr; // error of position loop in the previous loop cycle
    static int32_t prev_iout; // current output the previous loop cycle

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
        
            pdes=ctrl_target;
            // MODE_POS
            // Task 5b.3 - position control loop (do it the last)
            // Steps to follow:
            // 1. Find perr, the error between desired position (pdes) and actual position (dji_fb.enc)
            // 2. Find dperr, the difference between the error calculated in current loop cycle (perr)
            //    and the previous loop cycle (prev_perr)
            // 3. Calculate vdes using this formula: (P_KP*perr + P_KD*dperr)/128
            // 4. Update prev_perr with perr
            // 5. Limit vdes with MAX_VEL using constrain()
            // TYPE YOUR CODE HERE:
            const int32_t pdiff = pdes - pstart;
            if (time <= exp_time)
            {
                pexp = ((3 / (exp_time * exp_time)) * time * time + (-2 / (exp_time * exp_time * exp_time)) * time * time * time )*(pdiff)+pstart;
                vexp = ((6 * (pdiff))/ (exp_time * exp_time)) * time + ((-6 *(pdiff))/ (exp_time * exp_time * exp_time)) * time * time ;
                vexp = (vexp*100*60)/(8191*19);
                perr = pexp - dji_fb.enc;
                vdes = P_KP * perr + vexp;
                vdes = constrain(vdes, -MAX_VEL, MAX_VEL);
                time++;
            }

            //<-------old code-------->
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
        if (time > exp_time)
        {
            ides = 0;
        }
        ides /= 128;
        ides = constrain(ides, -MAX_CUR, MAX_CUR);
    }

    // Task 5b.1 - limit the change in current (do it first)
    // 1. find the difference between the desired current output(ides) and previous current output (prev_iout)
    // 2. limit the difference with MAX_CUR_CHANGE using constrain()
    // 3. apply the limited change to the previous current output (prev_iout)
    // 4. prev_iout is now the calculated current, assign it to iout
    // TYPE YOUR CODE HERE:
    prev_iout += constrain(ides - prev_iout, -MAX_CUR_CHANGE, MAX_CUR_CHANGE);
    iout = prev_iout;

    return iout; // return the calculated current output
}

void setup()
{
    while (!Serial)
        ; // wait serial ready
    Serial.begin(1000000);
    Serial.flush();
    dji_init();
    // Serial.println("DJI Init Done");

    // Uncomment the following if you have not yet complete get_command() part
    // for Task 5b.1
    // ctrl_mode = MODE_CUR;
    // ctrl_target = 512;
    // for Task 5b.2
    // ctrl_mode = MODE_VEL;
    // ctrl_target = 800;
    // for Task 5b.3
    // ctrl_mode = MODE_POS;
    // ctrl_target = 10000;
}

// void balance(){
//     // Target velocity of dji motor
//     int32_t target_v;
//     // Store encoder reading in a variable which will be
//     // compare to the current encoder reading
//     static int32_t prev_enc;

//     // Task 1: Set ctrl_mode to MODE_VEL
//     ctrl_mode = MODE_VEL;

//     // Get relative reading of encoder with respect to one cycle
//     cur_enc = myEnc.read();
//     cur_enc %= 4096;

//     // Reading over a half then change side to find the closest path
//     if (cur_enc < -2048)
//         cur_enc += 4096;
//     if (cur_enc > 2047)
//         cur_enc -= 4096;

//     // Task3: Stay still if the rod reaches deadzone
//     // (if the rod is within a range, pendulum will not move).
//     int deadzone_lower_bound = -10;
//     int deadzone_upper_bound = 10;
//     // Task 4:
//     // Calculate target velocity of motor by using PID formula
//     // Hints: v = enc_reading * kP + (change in enc_reading)* kD

//     target_v = cur_enc * kP + (cur_enc - prev_enc) * kD;

//     // Update history variable prev_enc
//     prev_enc = cur_enc;

//     // Task 5: Update ctrl_target with the calculated velocity from PID
//     if (cur_enc < deadzone_lower_bound || cur_enc > deadzone_upper_bound){
//         ctrl_target = target_v;
//     }
// }

void loop()
{
    // Do not change anything here
    get_command();

    if (dji_get_feedback())
    {
        // Serial.print("Speed: ");
        // Serial.println(max_speed);
        // Uncomment this when you want to play with pendulum
        // PLEASE NOTIFIY US WHEN YOU WANT TO PLAY WITH PENDULUM !!!
        // balance();
        dji_set_current(control());
    }
}
