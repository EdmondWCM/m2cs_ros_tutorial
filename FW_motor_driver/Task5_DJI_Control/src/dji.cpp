/*
Library for controlling DJI C620/C610 with MCP2515 
Jeff Lai 20211104
*/
#include "dji.h"
#include <SPI.h>
#include <mcp2515.h>

#define CS_PIN 10
#define INT_PIN 2

#ifdef MAX_CUR
#define CURRENT_LIMIT MAX_CUR
#endif

#ifndef MAX_CUR
#define CURRENT_LIMIT 1024
#endif

volatile bool intterupt_received = false;
struct can_frame rx_msg;
struct can_frame tx_msg;
 
typedef struct{
  uint16_t enc; // rotor encoder count
  int16_t rpm; // rotor rpm
  int16_t cur; // actual current output
  uint16_t tmp; // temperature
} DJI_Raw_Feedback;

MCP2515 mcp2515(10);
DJI_Raw_Feedback dji_raw;
DJI_Feedback dji_fb;
 
void on_mcp2515_cb()
{
  intterupt_received = true;
}
 
bool get_rx_msg(){
  uint8_t irq = mcp2515.getInterrupts();
  mcp2515.clearInterrupts();
  bool msg_received = false;
  if (irq & MCP2515::CANINTF_RX0IF)
  {
    if (mcp2515.readMessage(MCP2515::RXB0, &rx_msg) == MCP2515::ERROR_OK)
    {
      // frame contains received from RXB0 message
      // Serial.println("RXB0");
      msg_received = true;
    }
  }
 
  if (irq & MCP2515::CANINTF_RX1IF)
  {
    if (mcp2515.readMessage(MCP2515::RXB1, &rx_msg) == MCP2515::ERROR_OK)
    {
      // frame contains received from RXB1 message
      // Serial.println("RXB1");
      msg_received = true;
    }
  }
  return msg_received;
}
 
void dji_init(){
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  pinMode(INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), on_mcp2515_cb, FALLING);
}

/*
Get DJI feedback from MCP2515 and put them in dji_fb
return 1 if success
return 0 if failed
*/
bool dji_get_feedback(){
  if(!intterupt_received | !get_rx_msg())
    return 0;

  static uint16_t prev_enc = 0;
  memcpy(&dji_raw, rx_msg.data, sizeof(dji_raw));
  ///////////////////////////////////////////////////////////////////////
  // handle encoder multiturn
  uint16_t raw_enc = (dji_raw.enc >> 8) | ((dji_raw.enc & 0xFF) << 8);
  int16_t dpos = (8192 + raw_enc - prev_enc) % 8192; // 8192+0-8191
  if (dpos > 4096)
    dpos = dpos - 8192;
  dji_fb.enc += dpos;
  prev_enc = raw_enc;
  ///////////////////////////////////////////////////////////////////////
 
  ///////////////////////////////////////////////////////////////////////
  // handle rpm, current endian
  ///////////////////////////////////////////////////////////////////////
    dji_fb.rpm   = (uint16_t)rx_msg.data[2] << 8 | rx_msg.data[3];
    dji_fb.cur = (uint16_t)rx_msg.data[4] << 8 | rx_msg.data[5];
  return 1;
}
 
void dji_set_current(int32_t cur){
  // limit current
  cur = min(CURRENT_LIMIT, cur);
  cur = max(-CURRENT_LIMIT, cur);
  cur = cur & 0xFFFF;
  // assemble control message
  tx_msg.can_id = 0x200;
  tx_msg.can_dlc = 8;
  tx_msg.data[0] = cur >> 8;
  tx_msg.data[1] = cur & 0xFF;
  mcp2515.sendMessage(&tx_msg);
}