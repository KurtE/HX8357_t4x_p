#ifndef _ILI948X_T4X_P_H_
#define _ILI948X_T4X_P_H_

// uncomment below the line corresponding to your screen:

// #define ILI9481_1
// #define ILI9481_2
// #define ILI9486
// #define ILI9488
// #define R61529

#include "Arduino.h"
#include "DMAChannel.h"
#include "FlexIO_t4.h"

#include "Teensy_Parallel_GFX.h"

#define SHIFTNUM 4            // number of shifters used (must be 1, 2, 4, or 8)
#define SHIFTER_DMA_REQUEST (_write_shifter + SHIFTNUM - 1) // only 0, 1, 2, 3 expected to work
#define SHIFTER_IRQ (_write_shifter + SHIFTNUM - 1)

#define FLEXIO_ISR_PRIORITY 64 // interrupt is timing sensitive, so use relatively high priority (supersedes USB)

#define _TFTWIDTH 320  // ILI9488 TFT width in default rotation
#define _TFTHEIGHT 480 // ILI9488 TFT height in default rotation

#define HX8357_NOP     0x00
#define HX8357_SWRESET 0x01
#define HX8357_RDDID   0x04
#define HX8357_RDDST   0x09

#define HX8357_SLPIN   0x10
#define HX8357_SLPOUT  0x11
#define HX8357B_PTLON   0x12
#define HX8357B_NORON   0x13

#define HX8357_RDMODE  0x0A
#define HX8357_RDMADCTL  0x0B
#define HX8357_RDPIXFMT  0x0C
#define HX8357_RDIMGFMT  0x0D
#define HX8357_RDSELFDIAG  0x0F

#define HX8357_INVOFF  0x20
#define HX8357_INVON   0x21
#define HX8357_GAMMASET 0x26
#define HX8357_DISPOFF 0x28
#define HX8357_DISPON  0x29

#define HX8357_CASET   0x2A
#define HX8357_PASET   0x2B
#define HX8357_RAMWR   0x2C
#define HX8357_RAMRD   0x2E

#define HX8357B_PTLAR    0x30
#define HX8357_TEON  0x35
#define HX8357_TEARLINE  0x44
#define HX8357_MADCTL   0x36
#define HX8357_VSCRSADD 0x37
#define HX8357_COLMOD  0x3A

#define HX8357_SETOSC 0xB0
#define HX8357_SETPWR1 0xB1
#define HX8357B_SETDISPLAY 0xB2
#define HX8357_SETRGB 0xB3
#define HX8357D_SETCOM  0xB6

#define HX8357B_SETDISPMODE  0xB4
#define HX8357D_SETCYC  0xB4
#define HX8357B_SETOTP 0xB7
#define HX8357D_SETC 0xB9

#define HX8357B_SET_PANEL_DRIVING 0xC0
#define HX8357D_SETSTBA 0xC0
#define HX8357B_SETDGC  0xC1
#define HX8357B_SETID  0xC3
#define HX8357B_SETDDB  0xC4
#define HX8357B_SETDISPLAYFRAME 0xC5
#define HX8357B_GAMMASET 0xC8
#define HX8357B_SETCABC  0xC9
#define HX8357_SETPANEL  0xCC

#define HX8357B_SETPOWER 0xD0
#define HX8357B_SETVCOM 0xD1
#define HX8357B_SETPWRNORMAL 0xD2

#define HX8357B_RDID1   0xDA
#define HX8357B_RDID2   0xDB
#define HX8357B_RDID3   0xDC
#define HX8357B_RDID4   0xDD

#define HX8357D_SETGAMMA 0xE0
#define HX8357B_SETGAMMA 0xC8
#define MADCTL_MY 0x80  // Bottom to top
#define MADCTL_MX 0x40  // Right to left
#define MADCTL_MV 0x20  // Row/Column exchange
#define MADCTL_ML 0x10  // LCD refresh Bottom to top
#define MADCTL_RGB 0x00 // Red-Green-Blue pixel order
#define MADCTL_BGR 0x08 // Blue-Green-Red pixel order
#define MADCTL_MH 0x04  // LCD refresh right to left

/****************************************************************************************/
// #define HX8357_CLOCK_READ 30   //equates to 8mhz
#define HX8357_CLOCK_READ 60 // equates to 4mhz
//#define HX8357_CLOCK_READ 120   //equates to 2mhz

enum {
    HX8357D = 0
};

#ifdef __cplusplus
class HX8357_t4x_p : public Teensy_Parallel_GFX {
  public:
    HX8357_t4x_p(int8_t dc, int8_t cs = -1, int8_t rst = -1);
    void begin(uint8_t display_name = HX8357D, uint8_t baud_speed_mhz = 20);
    uint8_t getBusSpd();

    // If used this must be called before begin
    // Set the FlexIO pins.  The first version you can specify just the wr, and read and optionsl first Data.
    // it will use information in the Flexio library to fill in d1-d7
    bool setFlexIOPins(uint8_t write_pin, uint8_t rd_pin, uint8_t tft_d0 = 0xff);

    // Set the FlexIO pins.  Specify all of the pins for 8 bit mode. Must be called before begin
    bool setFlexIOPins(uint8_t write_pin, uint8_t rd_pin, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3,
                       uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7);

    uint8_t setBitDepth(uint8_t bitDepth);
    uint8_t getBitDepth();

    void setFrameRate(uint8_t frRate);
    uint8_t getFrameRate();

    void setTearingEffect(bool tearingOn);
    bool getTearingEffect();

    void setTearingScanLine(uint16_t scanLine);
    uint16_t getTearingScanLine();

    void setRotation(uint8_t r);
    void invertDisplay(bool invert);
    void displayInfo();

    void pushPixels16bit(const uint16_t *pcolors, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
    void pushPixels16bitDMA(const uint16_t *pcolors, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

    uint8_t readCommand(uint8_t const cmd);
    uint32_t readCommandN(uint8_t const cmd, uint8_t count_bytes);

    // Added functions to read pixel data...
    // uint16_t readPixel(int16_t x, int16_t y);
    void readRectFlexIO(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t *pcolors);
    
    // Called by GFX to do updateScreenAsync and new writeRectAsync(;
    bool writeRectAsyncFlexIO(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t *pcolors);
    bool writeRectAsyncActiveFlexIO();

    // void pushPixels16bitTearing(uint16_t * pcolors, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2 );
    // void pushPixels24bitTearing(uint16_t * pcolors, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2 );
    void DMAerror();

    /**************************************************************/
    void setScroll(uint16_t offset);

    uint16_t _previous_addr_x0 = 0xffff;
    uint16_t _previous_addr_x1 = 0xffff;
    uint16_t _previous_addr_y0 = 0xffff;
    uint16_t _previous_addr_y1 = 0xffff;

    uint16_t generate_output_word(uint8_t data) __attribute__((always_inline)) {
        #if !defined(ARDUINO_TEENSY40)
        return data;
        #else
        if (_bus_width == 8) return data;
        return (uint16_t)(data & 0x0F) | (uint16_t)((data & 0xF0) << 2);
        #endif
    }

    uint8_t read_shiftbuf_byte() __attribute__((always_inline)) {
        #if !defined(ARDUINO_TEENSY40)
        return p->SHIFTBUFBYS[_read_shifter];
        #else
        if (_bus_width == 8) return p->SHIFTBUFBYS[_read_shifter];
        uint16_t data = p->SHIFTBUF[_read_shifter] >> 16; // 10 bits but shifter does 16
        return ((data >> 2) & 0xf0) | (data & 0xf);
        #endif
    }


    void setAddr(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
        __attribute__((always_inline)) {

        uint8_t Command;
        uint8_t CommandValue[4];
        if ((x0 != _previous_addr_x0) || (x1 != _previous_addr_x1)) {
            Command = 0x2A;
            CommandValue[0U] = x0 >> 8U;
            CommandValue[1U] = x0 & 0xFF;
            CommandValue[2U] = x1 >> 8U;
            CommandValue[3U] = x1 & 0xFF;
            SglBeatWR_nPrm_8(Command, CommandValue, 4U);
            _previous_addr_x0 = x0;
            _previous_addr_x1 = x1;
        }
        if ((y0 != _previous_addr_y0) || (y1 != _previous_addr_y1)) {
            Command = 0x2B;
            CommandValue[0U] = y0 >> 8U;
            CommandValue[1U] = y0 & 0xFF;
            CommandValue[2U] = y1 >> 8U;
            CommandValue[3U] = y1 & 0xFF;
            SglBeatWR_nPrm_8(Command, CommandValue, 4U);
            _previous_addr_y0 = y0;
            _previous_addr_y1 = y1;
        }
    }
    enum { WRITE_SHIFT_TO = 20,
           READ_SHIFT_TO = 20,
           WRITE_TIMER_TO = 20 };
    void waitWriteShiftStat(int error_identifier = 0) __attribute__((always_inline)) {
        elapsedMillis em = 0;
        while (0 == (p->SHIFTSTAT & _write_shifter_mask)) {
            if (em > WRITE_SHIFT_TO) {
                Serial.printf(">>>waitWriteShiftStat(%d) TO\n", error_identifier);
                if (Serial.available()) {
                    while (Serial.read() != -1) {
                    }
                    Serial.println("*** Paused ***");
                    while (Serial.read() == -1) {
                    }
                    while (Serial.read() != -1) {
                    }
                }
                return; // bail
            }
        }
    }

    void waitReadShiftStat(int error_identifier = 0) __attribute__((always_inline)) {
        elapsedMillis em = 0;
        while (0 == (p->SHIFTSTAT & _read_shifter_mask)) {
            if (em > READ_SHIFT_TO) {
                Serial.printf(">>>waitReadShiftStat(%d) TO\n", error_identifier);
                if (Serial.available()) {
                    while (Serial.read() != -1) {
                    }
                    Serial.println("*** Paused ***");
                    while (Serial.read() == -1) {
                    }
                    while (Serial.read() != -1) {
                    }
                }
                return; // bail
            }
        }
    }

    void waitTimStat(int error_identifier = 0) __attribute__((always_inline)) {
        elapsedMillis em = 0;
        while (0 == (p->TIMSTAT & _flexio_timer_mask)) {
            if (em > WRITE_SHIFT_TO) {
                Serial.printf(">>>waitWriteShiftStat(%d) TO\n", error_identifier);
                if (Serial.available()) {
                    while (Serial.read() != -1) {
                    }
                    Serial.println("*** Paused ***");
                    while (Serial.read() == -1) {
                    }
                    while (Serial.read() != -1) {
                    }
                }
                return; // bail
            }
        }
    }

    void beginWrite16BitColors();
    void write16BitColor(uint16_t color);
    void endWrite16BitColors();
//    void write16BitColor(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, const uint16_t *pcolors, uint16_t count);
    void writeRectFlexIO(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t *pcolors);
    void fillRectFlexIO(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);

    typedef void (*CBF)();
    CBF _callback;
    void onCompleteCB(CBF callback);

  protected:
  private:
    uint8_t _display_name = 0;
    FlexIOHandler *pFlex;
    IMXRT_FLEXIO_t *p;
    const FlexIOHandler::FLEXIO_Hardware_t *hw;
    static DMAChannel flexDma;

    uint8_t _baud_div = 20;

    uint8_t _bitDepth = 16;
    uint8_t _rotation = 0;
    uint8_t MADCTL[5];

    uint8_t _frameRate = 60;

    bool _bTearingOn = false;
    uint16_t _tearingScanLine = 0;

    // int16_t _width, _height;
    int8_t _dc, _cs, _rst;

    // The Teensy IO pins used for data and Read and Write
    uint8_t _data_pins[8], _wr_pin, _rd_pin;

    uint8_t _flexio_D0, _flexio_WR, _flexio_RD; // which flexio pins do they map to
    uint8_t _write_shifter = 0;
    uint8_t _write_shifter_mask = (1 << 0);
    uint8_t _read_shifter = 3;
    uint8_t _bus_width = 8;
    uint8_t _read_shifter_mask = (1 << 3);
    uint8_t _flexio_timer = 0;
    uint8_t _flexio_timer_mask = 1 << 0;

    uint8_t _dummy;
    uint8_t _curMADCTL;

    volatile bool WR_AsyncTransferDone = true;
    uint32_t MulBeatCountRemain;
    uint16_t *MulBeatDataRemain;
    uint32_t TotalSize;

    void displayInit(uint8_t display_name);
    void CSLow();
    void CSHigh();
    void DCLow();
    void DCHigh();
    void gpioWrite();
    void gpioRead();

    void FlexIO_Init();
    typedef enum { CONFIG_CLEAR = 0,
                   CONFIG_SNGLBEAT,
                   CONFIG_MULTIBEAT,
                   CONFIG_SNGLREAD } Flexio_config_state_t;
    Flexio_config_state_t flex_config = CONFIG_CLEAR;
    void FlexIO_Config_SnglBeat();
    void FlexIO_Clear_Config_SnglBeat();
    void FlexIO_Config_MultiBeat();
    void FlexIO_Config_SnglBeat_Read();

    void SglBeatWR_nPrm_8(uint32_t const cmd, uint8_t const *value, uint32_t const length);
    void SglBeatWR_nPrm_16(uint32_t const cmd, const uint16_t *value, uint32_t const length);
    // Works on FlexIO1 and FlexIO2 but not 3 and only on Shifters 0-3
    void MulBeatWR_nPrm_DMA(uint32_t const cmd, const void *value, uint32_t const length);

    // Works on FlexIO3 and others as well
    void MulBeatWR_nPrm_IRQ(uint32_t const cmd,  const void *value, uint32_t const length);
    static void flexio_ISR();
    void flexIRQ_Callback();

    void microSecondDelay();

    static void dmaISR();
    void flexDma_Callback();
    static HX8357_t4x_p *IRQcallback;

    bool isCB = false;
    void _onCompleteCB();

    static HX8357_t4x_p *dmaCallback;

    /* variables used by ISR */
    volatile uint32_t _irq_bytes_remaining;
    volatile unsigned int _irq_bursts_to_complete;
    volatile uint32_t *_irq_readPtr;
    uint8_t  _irq_bytes_per_shifter;
    uint16_t _irq_bytes_per_burst;
    uint32_t finalBurstBuffer[SHIFTNUM];

};
#endif //__cplusplus
#endif //_IHX8357_t4x_p.h_