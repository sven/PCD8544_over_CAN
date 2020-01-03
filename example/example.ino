/**
 * @brief Counterpart to Test the "Nokia 5110 Display over CAN"
 *
 * This sketch fills the whole display (about 8 seconds) than clears it and
 * starts again. After each run it also flashes the display backlight and
 * requests the current values for temperature and voltage which can be seen in
 * the serial monitor.
 *
 * The sketch is losely based on the CAN_write.ino example of the MCP2515
 * library.
 *
 * Thanks to:
 *   * MCP2515 CAN: https://github.com/autowp/arduino-mcp2515
 *
 * Copyright (c) 2019, Sven Bachmann <dev@mcbachmann.de>
 *
 * Licensed under the MIT license, see LICENSE for details - or stricter if
 * used components require it.
 */

#include <SPI.h>
#include <mcp2515.h>

/*****************************************************************************/
/* Pin Mappings */
/*****************************************************************************/

// CAN
static const int PIN_CAN_CS = 7;


/*****************************************************************************/
/* Local variables */
/*****************************************************************************/
static MCP2515 mcp2515(PIN_CAN_CS);
static struct can_frame canTx;
static struct can_frame canRx;


/*****************************************************************************/
/** Initialization
 */
void setup() {
    canTx.can_id  = 0xF6;

    while (!Serial);
    Serial.begin(115200);
    SPI.begin();

    if(MCP2515::ERROR_OK != mcp2515.reset())
    {
        Serial.println("error: reset");
    }

    if(MCP2515::ERROR_OK != mcp2515.setBitrate(CAN_125KBPS))
    {
        Serial.println("error: setBitrate");
    }

    if(MCP2515::ERROR_OK != mcp2515.setNormalMode())
    {
        Serial.println("error: setNormalMode");
    }

    Serial.println("Example: Write to CAN");
}


/*****************************************************************************/
/** Processing
 */
void loop()
{
    static uint8_t pos_x = 0;
    static uint8_t pos_y = 0;
    static uint32_t lastWrap = 0;
    uint32_t now;

    // sleep 5 seconds after each screen fill
    if(!lastWrap) {

        // clear display if first position is reached
        if(!pos_x && !pos_y) {
            canTx.can_dlc = 1;
            canTx.data[0] = 0x04;
            mcp2515.sendMessage(&canTx);
            delay(100);
        }

        // set pixel
        canTx.can_dlc = 3;
        canTx.data[0] = 0x05;
        canTx.data[1] = pos_x;
        canTx.data[2] = pos_y;
        mcp2515.sendMessage(&canTx);
        delay(2);

        // go from left to right and top to bottom
        pos_x++;
        if(pos_x >= 84) {
            pos_x = 0;
            pos_y++;
            if(pos_y >= 48) {
                pos_y = 0;
                lastWrap = millis();

                // switch off backlight
                canTx.can_dlc = 3;
                canTx.data[0] = 0x03;
                canTx.data[1] = 255;
                mcp2515.sendMessage(&canTx);
                delay(100);

                // read temperature and voltage
                canTx.can_dlc = 1;
                canTx.data[0] = 0x02;
                mcp2515.sendMessage(&canTx);
                delay(500);

                // switch on backlight
                canTx.can_dlc = 3;
                canTx.data[0] = 0x03;
                canTx.data[1] = 127;
                mcp2515.sendMessage(&canTx);
                delay(100);
            }
        }
    } else {
        if((millis() - lastWrap) >= 4300) {
            lastWrap = 0;
        }

    }

    // read messages
    if(MCP2515::ERROR_OK != mcp2515.readMessage(&canRx))
    {
        return;
    }

    Serial.print(canRx.can_id, HEX); // print ID
    Serial.print(" ");
    Serial.print(canRx.can_dlc, HEX); // print DLC
    Serial.print(" ");

    for (int i = 0; i<canRx.can_dlc; i++)  {  // print the data
        Serial.print(canRx.data[i],HEX);
        Serial.print(" ");
    }

    Serial.println();
}
