/*
Library for controlling DJI C620/C610 with MCP2515 
Jeff Lai 20211104
*/

#include <Arduino.h>

typedef struct{
  int32_t enc; 
  int16_t rpm;
  int16_t cur;
  uint8_t tmp;
} DJI_Feedback;

extern DJI_Feedback dji_fb;

// initialize MCP2515, setup interrupt
void dji_init(); 
// receive feedback, and update dji_fb
// return 0 if feedback not available
bool dji_get_feedback(); 
// set motor current
// 16384 = 20A
// -16383 = -20A
void dji_set_current(int32_t cur);
