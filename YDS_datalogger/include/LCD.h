#ifndef LCD_H
#define LCD_H

#include <U8g2lib.h>

uint32_t lastByteTime = 0;

// Define your display model and connection type
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

// Define the frame interval in milliseconds
const unsigned long frameIntervalMs = 33; // 1000 milliseconds / 30 frames = 33 milliseconds per frame

// Function to display data on the OLED display
void displayData()
{
    static unsigned long lastDisplayUpdate = 0;

    unsigned long currentTime = esp_timer_get_time() / 1000; // Convert microseconds to milliseconds

    // Check if it's time to update the display
    if (currentTime - lastDisplayUpdate >= frameIntervalMs)
    {
        // Clear the display buffer
        u8g2.clearBuffer();

        // Set the font to courR08 for all text
        u8g2.setFont(u8g2_font_sirclivethebold_tr);

        // Define the text to display
        const char *headerText = "Yamaha";

        // Get the display width
        int displayWidth = u8g2.getDisplayWidth();

        // Calculate the width of the header text
        int textWidth = u8g2.getStrWidth(headerText);

        // Calculate the number of spaces needed on each side of the header text for centering
        int padding = (displayWidth - textWidth) / 2;

        // Draw the header text centered on the display
        u8g2.drawStr(padding, 10, headerText);

        // Draw static labels for each parameter
        u8g2.drawStr(0, 26, "RPM:");
        u8g2.drawStr(0, 36, "Speed:");
        u8g2.drawStr(0, 46, "Temp:");
        u8g2.drawStr(0, 56, "Gear:");

        // Prepare text strings for dynamic PID values
        char rpmText[6];
        char speedText[6];
        char coolantText[4];
        char gearText[2];

        // Print and update dynamic PID values
        snprintf(rpmText, sizeof(rpmText), "%d", RPM_PID);
        u8g2.drawStr(75, 26, rpmText); // Draw RPM value at a fixed position

        snprintf(speedText, sizeof(speedText), "%d", Speed_PID);
        u8g2.drawStr(75, 36, speedText); // Draw speed value at a fixed position

        snprintf(coolantText, sizeof(coolantText), "%d", Coolant_PID);
        u8g2.drawStr(75, 46, coolantText); // Draw coolant temperature value at a fixed position

        if (Gear_PID == 0)
        {
            // Set gearText to "-" ( NO Cap with Font)
            strcpy(gearText, "n");
        }
        else
        {
            // Format the gear value as usual
            snprintf(gearText, sizeof(gearText), "%d", Gear_PID);
        }

        // Draw the gear value at a fixed position
        u8g2.drawStr(75, 56, gearText);

        // Update the display with the changes
        u8g2.sendBuffer();

        // Update the timestamp of the last display update
        lastDisplayUpdate = currentTime;
    }
}

#endif // LCD_H
