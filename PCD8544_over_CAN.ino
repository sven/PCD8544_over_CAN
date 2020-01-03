/**
 * @brief Nokia 5110 Display over CAN
 *
 * Thanks to:
 *   * ATmega Temperature: http://www.avrfreaks.net/forum/328p-internal-temperature
 *   * ATmega Voltage: https://code.google.com/archive/p/tinkerit/wikis/SecretVoltmeter.wiki
 *   * Button Debounce: https://www.instructables.com/id/Arduino-Dual-Function-Button-Long-PressShort-Press/
 *   * Rotary Encoder: http://www.mathertel.de/Arduino/RotaryEncoderLibrary.aspx
 *   * MCP2515 CAN: https://github.com/autowp/arduino-mcp2515
 *   * PCD8544 Display: https://www.e-tinkers.com/2017/11/how-to-use-lcd-5110-pcd-8544-with-arduino/
 *   * Display Wiring: https://eldontronics.wordpress.com/2018/03/01/driving-the-nokia-5110-display-with-arduino-no-external-library/
 *
 * Not implemented:
 *   * sleep mode
 *   * interrupts for CAN and rotary encoder push button (hw wiring is already done)
 *   * CAN filter
 *
 * Information:
 *   * PCD8544 can't handle 10 MHz SPI speed that is used by the MCP2515
 *     library, so it is adapted before each transfer
 *
 * Copyright (c) 2019, Sven Bachmann <dev@mcbachmann.de>
 *
 * Licensed under the MIT license, see LICENSE for details - or stricter if
 * used components require it.
 */

#include <SPI.h>
#include <mcp2515.h>
#include <RotaryEncoder.h>


/*****************************************************************************/
/* Pin Mappings */
/*****************************************************************************/

// display
static const int RD_PIN_DISPLAY_CLK = 13;
static const int RD_PIN_DISPLAY_DIN = 11;
static const int RD_PIN_DISPLAY_DC = 10;
static const int RD_PIN_DISPLAY_RST = 9;
static const int RD_PIN_DISPLAY_SCE = 8;
static const int RD_PIN_DISPLAY_LED = 6;

// CAN
static const int RD_PIN_CAN_CS = 7;

// rotary encoder
static const int RD_PIN_ROT_SW = 3;
static const int RD_PIN_ROT_DT = 4;
static const int RD_PIN_ROT_CLK = 5;


/*****************************************************************************/
/* Constants */
/*****************************************************************************/

// CAN messages
enum
{
    RD_CAN_MSG_STATUS = 0x01,
    RD_CAN_MSG_TEMP_VOLT = 0x02,
    RD_CAN_MSG_BACKLIGHT = 0x03,
    RD_CAN_MSG_CLEAR = 0x04,
    RD_CAN_MSG_PIXEL = 0x05,
};

// CAN ids
static const uint8_t RD_CAN_LOCAL_ID = 0xf6;
static const uint8_t RD_CAN_REMOTE_ID = 0xf5;

// device
static const uint8_t RD_ID = 0x01;
static const uint8_t RD_VERSION = 0x01;

// display
static const uint32_t RD_DISPLAY_SPI_CLOCK = 4000000;
static const uint8_t RD_DISPLAY_WIDTH = 84;
static const uint8_t RD_DISPLAY_HEIGHT = 48;
static const uint8_t RD_DISPLAY_CMD = LOW;
static const uint8_t RD_DISPLAY_DATA = HIGH;
static const uint8_t RD_DISPLAY_BACKLIGHT_ON = 0;
static const uint8_t RD_DISPLAY_BACKLIGHT_HALF = 127;
static const uint8_t RD_DISPLAY_BACKLIGHT_OFF = 255;

// temperature and voltage correction
static const int16_t RD_ATMEGA_TEMP_CORR = -117;
static const uint16_t RD_ATMEGA_VOLT_CORR = 1023L;

// button
static const uint8_t RD_CAN_BUTTON_PRESS = 0x01;
static const uint8_t RD_CAN_BUTTON_LONG_PRESS = 0x02;


/*****************************************************************************/
/* Local variables */
/*****************************************************************************/
static struct can_frame rdCanTxBuf;
static struct can_frame rdCanRxBuf;
static MCP2515 rdMcp2515(RD_PIN_CAN_CS);
static RotaryEncoder rdEncoder(RD_PIN_ROT_DT, RD_PIN_ROT_CLK);
static uint8_t rdPixelBuf[(RD_DISPLAY_WIDTH * RD_DISPLAY_HEIGHT) / 8];
static boolean rdButtonActive = false;
static boolean rdButtonLongPressActive = false;


/*****************************************************************************/
/* Prototypes */
/*****************************************************************************/
void rdDisplayClear();
void rdDisplayBacklight(uint8_t value);
void rdDisplaySetPixel(uint8_t x, uint8_t y);
void rdDisplayWrite(unsigned int mode, unsigned char data);
void rdButtonProcess();
uint16_t rdAtmegaReadTempOrVoltage(bool flagTemp);


/*****************************************************************************/
/** Initialization
 */
void setup()
{
    // initialize UART
    Serial.begin(115200);

    // configure IOs
    pinMode(RD_PIN_DISPLAY_CLK, OUTPUT);
    pinMode(RD_PIN_DISPLAY_DIN, OUTPUT);
    pinMode(RD_PIN_DISPLAY_DC, OUTPUT);
    pinMode(RD_PIN_DISPLAY_SCE, OUTPUT);
    pinMode(RD_PIN_DISPLAY_RST, OUTPUT);
    pinMode(RD_PIN_DISPLAY_LED, OUTPUT);
    pinMode(RD_PIN_CAN_CS, OUTPUT);
    pinMode(RD_PIN_ROT_SW, INPUT);

    // enable backlight on bootup
    rdDisplayBacklight(RD_DISPLAY_BACKLIGHT_ON);

    // wake up display
    digitalWrite(RD_PIN_DISPLAY_RST, HIGH);

    // disable display and CAN SPI
    digitalWrite(RD_PIN_DISPLAY_SCE, HIGH);
    digitalWrite(RD_PIN_CAN_CS, HIGH);

    // initialize SPI
    SPI.begin();

    // initialize CAN
    if (MCP2515::ERROR_OK != rdMcp2515.reset())
    {
        Serial.println("MCP2515: reset error");
    }

    if (MCP2515::ERROR_OK != rdMcp2515.setBitrate(CAN_125KBPS))
    {
        Serial.println("MCP2515: setBitrate error");
    }

    if (MCP2515::ERROR_OK != rdMcp2515.setNormalMode())
    {
        Serial.println("MCP2515: setNormalMode error");
    }

    // initialize display
    rdDisplayWrite(RD_DISPLAY_CMD, 0x21);  // Set Extended Command set
    rdDisplayWrite(RD_DISPLAY_CMD, 0xb2);  // Set Vlcd to 6v (LCD Contrast)
    rdDisplayWrite(RD_DISPLAY_CMD, 0x13);  // Set voltage bias system 1:48 (Viewing Angle)
    rdDisplayWrite(RD_DISPLAY_CMD, 0x20);  // Set Normal Command set
    rdDisplayClear(); // Clear all display memory and set cursor to 1,1
    rdDisplayWrite(RD_DISPLAY_CMD, 0x09);  // Set all pixels ON
    rdDisplayWrite(RD_DISPLAY_CMD, 0x0c);  // Set display mode to Normal

    // prepare CAN TX message buffer
    rdCanTxBuf.can_id = RD_CAN_REMOTE_ID;
}


/*****************************************************************************/
/** Processing
 */
void loop()
{
    static int lastPos = 0;
    static bool lastButtonActive = false;
    static bool lastLongPressActive = false;
    uint8_t button = 0;
    int8_t rotation;
    int newPos;
    bool statusUpdate;

    // process button
    rdButtonProcess();

    // process rotary encoder
    rdEncoder.tick();

    // detect rotation
    newPos = rdEncoder.getPosition();
    rotation = (lastPos != newPos) ? lastPos - newPos : 0;
    statusUpdate = !!rotation;
    lastPos = newPos;

    // detect button
    if (rdButtonActive != lastButtonActive) {
        button |= (rdButtonActive) ? RD_CAN_BUTTON_PRESS : 0;
        lastButtonActive = rdButtonActive;
        statusUpdate = true;
    }

    if (rdButtonLongPressActive != lastLongPressActive)
    {
        button |= (rdButtonLongPressActive) ? RD_CAN_BUTTON_LONG_PRESS : 0;
        lastLongPressActive = rdButtonLongPressActive;
        statusUpdate = true;
    }

    // send status update
    if (statusUpdate)
    {
        rdCanTxBuf.can_dlc = 3;
        rdCanTxBuf.data[0] = RD_CAN_MSG_STATUS;
        rdCanTxBuf.data[1] = rotation;
        rdCanTxBuf.data[2] = button;

        if (MCP2515::ERROR_OK != rdMcp2515.sendMessage(&rdCanTxBuf))
        {
            Serial.println("MCP2515: sendMessage error");
        }
    }

    // read requests
    if (MCP2515::ERROR_OK != rdMcp2515.readMessage(&rdCanRxBuf))
    {
        return;
    }

    if (RD_CAN_LOCAL_ID != rdCanRxBuf.can_id)
    {
        return;
    }

    // handle messages
    switch (rdCanRxBuf.data[0])
    {
        case RD_CAN_MSG_TEMP_VOLT:
            uint16_t val;
            rdCanTxBuf.can_dlc = 5;
            rdCanTxBuf.data[0] = RD_CAN_MSG_TEMP_VOLT;
            val = rdAtmegaReadTempOrVoltage(true);
            rdCanTxBuf.data[1] = val >> 8;
            rdCanTxBuf.data[2] = val & 0xff;
            val = rdAtmegaReadTempOrVoltage(false);
            rdCanTxBuf.data[3] = val >> 8;
            rdCanTxBuf.data[4] = val & 0xff;

            if (MCP2515::ERROR_OK != rdMcp2515.sendMessage(&rdCanTxBuf))
            {
                Serial.println("MCP2515: sendMessage error");
            }
            break;

        case RD_CAN_MSG_BACKLIGHT:
            rdDisplayBacklight(rdCanRxBuf.data[1]);
            break;

        case RD_CAN_MSG_CLEAR:
            rdDisplayClear();
            break;

        case RD_CAN_MSG_PIXEL:
            rdDisplaySetPixel(rdCanRxBuf.data[1], rdCanRxBuf.data[2]);
            break;

        default:
            Serial.println("MCP2515: unknown message");
    }
}


/*****************************************************************************/
/** Button Detection and Debouncing
 */
void rdButtonProcess()
{
    static long buttonTimer = 0;
    static long longPressTime = 250;

    if (digitalRead(RD_PIN_ROT_SW) == LOW) {
        if (rdButtonActive == false) {
            rdButtonActive = true;
            buttonTimer = millis();
        }

        if ((millis() - buttonTimer > longPressTime) && (rdButtonLongPressActive == false)) {
            rdButtonLongPressActive = true;
        }
    } else {
        if (rdButtonActive == true) {
            rdButtonLongPressActive = false;
            rdButtonActive = false;
        }
    }
}


/*****************************************************************************/
/** Toggle Display Pixel at a given Position
 */
void rdDisplaySetPixel(uint8_t x, uint8_t y)
{
    unsigned pixelPos = x + ((y / 8) * RD_DISPLAY_WIDTH);
    unsigned pixelBit = 1 << (y % 8);
    rdPixelBuf[pixelPos] |= pixelBit;

    // x
    rdDisplayWrite(RD_DISPLAY_CMD, 0x80 | x);

    // y
    rdDisplayWrite(RD_DISPLAY_CMD, 0x40 | (y / 8));

    // pixel
    rdDisplayWrite(RD_DISPLAY_DATA, rdPixelBuf[pixelPos]);
}


/*****************************************************************************/
/** Control Display Backlight
 */
void rdDisplayBacklight(uint8_t value)
{
    analogWrite(RD_PIN_DISPLAY_LED, value);
}


/*****************************************************************************/
/** Clear Display Content
 */
void rdDisplayClear(void)
{
    memset(rdPixelBuf, 0, sizeof(rdPixelBuf) / sizeof(rdPixelBuf[0]));
    rdDisplayWrite(RD_DISPLAY_CMD, 0x80);
    rdDisplayWrite(RD_DISPLAY_CMD, 0x40);
    for(int pixel = (RD_DISPLAY_WIDTH * RD_DISPLAY_HEIGHT / 8); pixel > 0; pixel--) {
        rdDisplayWrite(RD_DISPLAY_DATA, 0x00);
    }
}


/*****************************************************************************/
/** Send Command or Data to Display
 */
void rdDisplayWrite(unsigned int mode, unsigned char data)
{
    SPI.beginTransaction(SPISettings(RD_DISPLAY_SPI_CLOCK, MSBFIRST, SPI_MODE0));
    digitalWrite(RD_PIN_DISPLAY_SCE, LOW);
    digitalWrite(RD_PIN_DISPLAY_DC, mode);
    SPI.transfer(data);
    digitalWrite(RD_PIN_DISPLAY_SCE, HIGH);
}


/*****************************************************************************/
/** Read ATmega Temperature or Voltage
 */
uint16_t rdAtmegaReadTempOrVoltage(bool flagTemp)
{
    static const unsigned sampleCount = 5;
    uint32_t res = 0;

    /* select internal 1.1V voltage reference and enable channel ADC8 */
    ADMUX = (flagTemp) ? ((1 << REFS1) | (1 << REFS0) | (1 << MUX3)) : ((1 << REFS0) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1));

    /* enable ADC and set the prescaler to div factor 16 */
    ADCSRA = (1 << ADEN) | (0x06 << ADPS0);

    /* wait for ADC initialization */
    _delay_ms(10);

    /* sample multiple times */
    for (unsigned cnt = 0; cnt < sampleCount; cnt++)
    {
        /* start ADC */
        ADCSRA |= (1 << ADSC);

        /* wait until ADC conversion is done */
        while (ADCSRA & (1 << ADSC));

        res += ADCW;
    }

    res /= sampleCount;

    /* fetch the result in mV and use the given 25 °C as reference */
    if (flagTemp) {
        // temperature correction
        return res + RD_ATMEGA_TEMP_CORR;
    }

    // voltage correction
    return  (1100L * RD_ATMEGA_VOLT_CORR) / res;
}
