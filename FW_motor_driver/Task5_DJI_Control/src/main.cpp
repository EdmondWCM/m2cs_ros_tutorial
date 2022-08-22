
#include <Arduino.h>
#include "dji.h"
#include <Encoder.h>

// control loop limit for safe

#define MAX_VEL 5000        // maximum velocity +- 5000 rpm
#define MAX_CUR 2048        // maximum current +-2.5 A
#define MAX_CUR_CHANGE 1024 // limit the change in current
#define MAX_POS 500000      // maximum position +-500000 count

// control loop parameters
#define P_KP 0.8
#define P_KD 1024
#define V_KP 512

enum Control_Mode
{
  MODE_CUR,    // value = 0
  MODE_VEL,    // value = 1
  MODE_POS,    // value = 2
  MODE_POS_VEL // value =3

};

// variable
Control_Mode ctrl_mode;
double ctrl_target;
double t_current = 0; // current time
double t_target = 0;  // target time
double p_init = 0;    // initial position
double p_exp = 0;     // expected position
double p_diff = 0;    // diff in initial and target position
double v_init = 0;    // initial velocity
double v_current = 0; // current velocity
double v_max = 0;     // max velocity
double v_exp = 0;     // expected velocity
double pre_v_exp = 0; // previous expected velocity
double vt = 0;        // (v_init / p_diff) * t_target
double a_exp = 0;     // expected accleration
double pre_a_exp = 0; // previous expected accleration
double jerk_exp = 0;  // expected jerk

// This function will print out the feedback on the serial monitor
void print_feedback()
{
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
  Serial.print((long)a_exp);
  Serial.print(" ");
  Serial.print((long)jerk_exp);
  Serial.print(" ");
  Serial.println(millis());
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

  int noOfField = sscanf(cmd_buf, "%s %ld %ld", &cmd, &val, &val2);
  if (!((noOfField == 3 && (cmd == 's')) || (noOfField == 3 && (cmd == 'p')) || (noOfField == 2 && (cmd == 'i' || cmd == 'v')) || (cmd == 'f')))
  {
    Serial.write("returned");
    return;
  }

  switch (cmd)
  {
  case 'i':
    ctrl_mode = MODE_CUR;
    ctrl_target = constrain(val, -MAX_CUR, MAX_CUR);
    t_current = 0;

    break;
  case 'v':
    ctrl_mode = MODE_VEL;
    ctrl_target = constrain(val, -MAX_VEL, MAX_VEL);
    t_current = 0;

    break;
  case 'p':
    ctrl_mode = MODE_POS;
    ctrl_target = constrain(val, -MAX_POS, MAX_POS);
    t_current = 0;
    t_target = val2;
    p_init = dji_fb.enc;
    p_diff = ctrl_target - p_init;
    v_init = dji_fb.rpm;
    v_init = (v_init * 8191.0) / (1000.0 * 60.0);
    vt = (v_init / p_diff) * t_target;

    break;
  case 's':
    ctrl_mode = MODE_POS_VEL;
    ctrl_target = constrain(val, -MAX_POS, MAX_POS);
    t_current = 0;
    t_target = 1000;
    p_init = dji_fb.enc;
    p_diff = ctrl_target - p_init;
    v_init = dji_fb.rpm;
    v_current = 0;
    v_max = val2;
    v_max = (v_max * 8191.0) / (1000.0 * 60.0);
    t_target = 15.0 / (8.0 * v_max);
    v_init = (v_init * 8191.0) / (1000.0 * 60.0);
    vt = (v_init / p_diff) * t_target;

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
  // error of output
  double p_err;
  // desired output, error of output
  double v_des, v_err;
  // desired current output, real current output
  double i_des, i_out;
  static double prev_i_out; // current output the previous loop cycle
  double A , B , C, D, E;

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
      // MODE_POS and MODE_POS_VEL
      if (ctrl_mode == MODE_POS_VEL)
      { 
        A = (6.0 - 4.0 * vt);
        B = (6.0 - 3.0 * vt);
        for (int i = 0; i < 10; i++)
        {
          t_current = (A * t_target) / (2*B);
          v_exp = ((v_init / p_diff) + ((A / (t_target * t_target)) * t_current) - ((B / (t_target * t_target * t_target)) * (t_current * t_current))) * p_diff;
          if (v_exp < 0) {
            v_exp = -v_exp;
          }
          if (v_exp < v_max)
          {
            t_target = t_target * 0.5;
          }
          else if (v_exp > v_max)
          {
            t_target = t_target * 1.5;
          }
          else
          {
            break;
          }
        }
      }
      // MODE_POS
      {
        if (t_current <= t_target && t_target != 0)
        {
          // formular * p_diff + p_init -> getting the correst s-t graph
          p_exp = (((v_init * t_current) / p_diff) + (((A / 2 )/ (t_target * t_target)) * t_current * t_current) - (((B / 3) / (t_target * t_target * t_target)) * t_current * t_current * t_current)) * (p_diff) + p_init; // unit = encoder count
          v_exp = ((v_init / p_diff) + ((A / (t_target * t_target)) * t_current) - ((B / (t_target * t_target * t_target)) * (t_current * t_current))) * p_diff;
          v_exp = (v_exp * 1000 * 60) / 8191; // unit = rpm
          a_exp = (v_exp - pre_v_exp) * 1000; // unit = rpm/s, if want to find rpm/sp, remove "* 1000"
          pre_v_exp = v_exp;
          jerk_exp = (a_exp - pre_a_exp) * 1000; // unit rpm/s^-2, if want to find rpm/sp^-2, remove "* 1000" here and in a_exp
          pre_a_exp = a_exp;
          t_current++;
        }
      }
      // else
      // {
      //     if (t_current <= t_target && t_target != 0)
      //     {
      //         C = 10.0 - 6.0 * vt;
      //         D = 8 * vt - 15;
      //         E = 6 - 3 * vt;
      //         p_exp = (v_init * t_current + (C * t_current * t_current * t_current) / (t_target * t_target * t_target) + (D * t_current * t_current * t_current * t_current) / (t_target * t_target * t_target * t_target) + (E * t_current * t_current * t_current * t_current * t_current) / (t_target * t_target * t_target * t_target * t_target)) * p_diff + p_init;
      //         v_exp = (v_init + (3 * C * t_current * t_current) / (t_target * t_target * t_target) + (4 * D * t_current * t_current * t_current) / (t_target * t_target * t_target * t_target) + (5 * E * t_current * t_current * t_current * t_current) / (t_target * t_target * t_target * t_target * t_target)) * p_diff + p_init;
      //         a_exp = (6 * C * t_current) / (t_target * t_target * t_target) + (12 * D * t_current * t_current) / (t_target * t_target * t_target * t_target) + (20 * E * t_current * t_current * t_current) / (t_target * t_target * t_target * t_target * t_target);
      //         v_exp = (v_exp * 1000 * 60) / 8191; // unit = rpm
      //         pre_v_exp = v_exp;
      //         jerk_exp = (a_exp - pre_a_exp) * 1000; // unit rpm/s^-2, if want to find rpm/sp^-2, remove "* 1000" here and in a_exp
      //         pre_a_exp = a_exp;
      //         t_current++;
      //     }
      // }
      p_err = p_exp - dji_fb.enc;
      v_des = P_KP * p_err + v_exp;
      v_des = constrain(v_des, -MAX_VEL, MAX_VEL);

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
  return i_out; // return the calculated current output
}

void setup()
{
  while (!Serial)
    ; // wait serial ready
  Serial.begin(115200);
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