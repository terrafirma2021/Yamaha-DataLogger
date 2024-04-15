#pragma once
#include <U8g2lib.h>

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

extern uint32_t Time;
const uint32_t centreOffset = 12;             
const uint32_t lowerBound = centreOffset + 5; 
const uint32_t upperBound = centreOffset - 4; 
uint16_t frameIntervalMs = 33;                // Frame rate
const uint16_t frameDelay = 10000;            // Frame movement speed

static uint32_t lastDisplayUpdate = 0;
static uint32_t lastMovementUpdate = 0; 
static uint32_t offset = centreOffset; 
static bool movingDown = true;
static bool atCenter = true; 

void displayData()
{

  if (Time - lastDisplayUpdate >= frameIntervalMs)
  {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_sirclivethebold_tr);

    u8g2.drawStr(0, offset, "Yamaha");
    u8g2.drawStr(0, offset + 16, "RPM");
    u8g2.drawStr(0, offset + 26, "Speed");
    u8g2.drawStr(0, offset + 36, "Temp");
    u8g2.drawStr(0, offset + 46, "Gear");

    char rpmText[6], speedText[6], coolantText[4], gearText[2];
    snprintf(rpmText, sizeof(rpmText), "%d", RPM_PID);
    u8g2.drawStr(75, offset + 16, rpmText);
    snprintf(speedText, sizeof(speedText), "%d", Speed_PID);
    u8g2.drawStr(75, offset + 26, speedText);
    snprintf(coolantText, sizeof(coolantText), "%d", Coolant_PID);
    u8g2.drawStr(75, offset + 36, coolantText);
    snprintf(gearText, sizeof(gearText), "%d", Gear_PID);
    u8g2.drawStr(75, offset + 46, gearText);

    u8g2.sendBuffer();
    
    lastDisplayUpdate = Time;
  }


  if (Time - lastMovementUpdate >= frameDelay)
  {
    if (atCenter)
    {
      if (movingDown)
      {
        offset++;
        if (offset >= lowerBound)
        {
          movingDown = false;
        }
      }
      else
      {
        offset--;
        if (offset <= upperBound)
        {
          movingDown = true; 
          atCenter = false;  
        }
      }
    }
    else
    {
      offset--;
      if (offset <= centreOffset)
      {
        atCenter = true;
      }
    }

    lastMovementUpdate = Time;
  }
}
