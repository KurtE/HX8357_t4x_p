//-------------------------------------------------------------------
// Kurt's Frame buffer and clip tests
//
// This test program is a set of random tests that have been done
// over time to test out the different functions to make sure they
// are working with the clipping functions as well as the frame
// buffer.  So you can for example test to see if you get the
// same results with the frame buffer turned on or off
//
// this sketch is in the public domain.
//
// This sketch depends on the fonts that are contained in the library
//     https://github.com/mjs513/ILI9341_fonts
//-------------------------------------------------------------------
// Set which Display we are using and at what speed
// Currently I have options for both MICROMOD and T42 to make it
// easier for testing

#define HX8357X HX8357D
#define HX8357X_SPEED_MHX 20


#include <MemoryHexDump.h>

#include <HX8357_t4x_p.h>
#include <Teensy_Parallel_GFX.h>
#include <Adafruit_GFX.h>  // Core graphics library
#include "font_Arial.h"
#include "font_ArialBold.h"
#include <Fonts/FreeMonoBoldOblique12pt7b.h>
#include <Fonts/FreeSerif12pt7b.h>

#define ROTATION 3

#define KURTS_MICROMOD

#include "SPI.h"

//Adafruit_GFX_Button button;

// Let's allocate the frame buffer ourself.
//DMAMEM uint16_t tft_frame_buffer[HX8357_TFTWIDTH * HX8357_TFTHEIGHT];

uint8_t use_dma = 0;
uint8_t use_clip_rect = 0;
uint8_t use_set_origin = 0;
uint8_t use_fb = 0;

#define ORIGIN_TEST_X 50
#define ORIGIN_TEST_Y 50

#ifdef ARDUINO_TEENSY41
HX8357_t4x_p tft = HX8357_t4x_p(10, 8, 9);  //(dc, cs, rst)
#elif ARDUINO_TEENSY40
HX8357_t4x_p tft = HX8357_t4x_p(0, 1, 2);  //(dc, cs, rst)
#elif defined(ARDUINO_TEENSY_DEVBRD4)  || defined(ARDUINO_TEENSY_DEVBRD5)
HX8357_t4x_p tft = HX8357_t4x_p(10, 11, 12);  //(dc, cs, rst)
#else
HX8357_t4x_p tft = HX8357_t4x_p(4, 5, 3);  //(dc, cs, rst)
#endif

void setup() {
    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    while (!Serial && (millis() < 4000))
        ;
    Serial.begin(115200);

    if (CrashReport) {
        Serial.print(CrashReport);
        WaitForUserInput();
    }
    //Serial.printf("Begin: CS:%d, DC:%d, MOSI:%d, MISO: %d, SCK: %d, RST: %d\n", TFT_CS, TFT_DC, TFT_MOSI, TFT_MISO, TFT_SCK, TFT_RST);
    Serial.println("\n*** Sketch Startup ***");
#ifdef TFT_TOUCH_CS
    pinMode(TFT_TOUCH_CS, OUTPUT);
    digitalWrite(TFT_TOUCH_CS, HIGH);
#endif

/*
   * begin(Dispalay type, baud)
   * Display type is associated with the the diplay
   * init configurations:
   * ILI9588, ILI9486, ILI9481_1, ILI9481_2, R61529
   * Baud can be as defined in the readme.
   * begin defaults to ILI9488 and 20Mhz:
   *     lcd.begin();
  */
// Begin optionally change FlexIO pins.
//    WRITE, READ, D0, [D1 - D7]
//    tft.setFlexIOPins(7, 8);
//    tft.setFlexIOPins(7, 8, 40);
//    tft.setFlexIOPins(7, 8, 40, 41, 42, 43, 44, 45, 6, 9);
//tft.setFlexIOPins(7, 8);
#if defined(ARDUINO_TEENSY_DEVBRD4)
    Serial.print("DEVBRD4 - ");
#elif defined(ARDUINO_TEENSY_DEVBRD5)
    Serial.print("DEVBRD5 - ");

#elif defined(ARDUINO_TEENSY_MICROMOD)
    Serial.print("Micromod - ");
#elif defined(ARDUINO_TEENSY41)
    Serial.print("Teensy4.1 - ");
#endif
    Serial.println(HX8357X_SPEED_MHX);
    tft.begin(HX8357X, HX8357X_SPEED_MHX);

    tft.setBitDepth(16);

    tft.displayInfo();
    //  tft.setFrameBuffer(tft_frame_buffer);

    tft.setRotation(ROTATION);
    tft.fillScreen(HX8357_BLACK);
    Serial.printf("Screen width:%u height:%u\n", tft.width(), tft.height());
 
    delay(500);
    tft.fillScreen(HX8357_RED);
    delay(500);
    tft.fillScreen(HX8357_GREEN);
    delay(500);
    tft.fillScreen(HX8357_BLUE);

    //
    //  button.initButton(&tft, 200, 125, 100, 40, HX8357_GREEN, HX8357_YELLOW, HX8357_RED, "UP", 1, 1);
    tft.onCompleteCB(&frame_complete_callback);

    drawTestScreen();
}

void frame_complete_callback() {
    Serial.println("\n*** Frame Complete Callback ***");
}

void SetupOrClearClipRectAndOffsets() {
    if (use_clip_rect) {
        tft.setClipRect();  // make sure we clear the whole screen
        tft.setOrigin();    // make sure none are set yet

        tft.fillScreen(HX8357_LIGHTGREY);

        // Now lets set origin.
        if (use_set_origin)
            tft.setOrigin(ORIGIN_TEST_X, ORIGIN_TEST_Y);
        int x = tft.width() / 4;
        int y = tft.height() / 4;
        int w = tft.width() / 2;
        int h = tft.height() / 2;
        tft.drawRect(x, y, w, h, HX8357_ORANGE);
        tft.updateScreen();
        tft.setClipRect(x + 1, y + 1, w - 2, h - 2);
        delay(250);

    } else {
        tft.setClipRect();
        if (use_set_origin)
            tft.setOrigin(ORIGIN_TEST_X, ORIGIN_TEST_Y);
        else
            tft.setOrigin();
    }
}


uint16_t palette[256];  // Should probably be 256, but I don't use many colors...
uint16_t pixel_data[2500];
const uint8_t pict1bpp[] = { 0xff, 0xff, 0xc0, 0x03, 0xa0, 0x05, 0x90, 0x9, 0x88, 0x11, 0x84, 0x21, 0x82, 0x41, 0x81, 0x81,
                             0x81, 0x81, 0x82, 0x41, 0x84, 0x21, 0x88, 0x11, 0x90, 0x09, 0xa0, 0x05, 0xc0, 0x03, 0xff, 0xff };
const uint8_t pict2bpp[] = {
    0x00,
    0x00,
    0x55,
    0x55,
    0xaa,
    0xaa,
    0xff,
    0xff,
    0x00,
    0x00,
    0x55,
    0x55,
    0xaa,
    0xaa,
    0xff,
    0xff,
    0x55,
    0x55,
    0xaa,
    0xaa,
    0xff,
    0xff,
    0x00,
    0x00,
    0x55,
    0x55,
    0xaa,
    0xaa,
    0xff,
    0xff,
    0x00,
    0x00,
    0xaa,
    0xaa,
    0xff,
    0xff,
    0x00,
    0x00,
    0x55,
    0x55,
    0xaa,
    0xaa,
    0xff,
    0xff,
    0x00,
    0x00,
    0x55,
    0x55,
    0xff,
    0xff,
    0x00,
    0x00,
    0x55,
    0x55,
    0xaa,
    0xaa,
    0xff,
    0xff,
    0x00,
    0x00,
    0x55,
    0x55,
    0xaa,
    0xaa,
    0x00,
    0x00,
    0x55,
    0x55,
    0xaa,
    0xaa,
    0xff,
    0xff,
    0x00,
    0x00,
    0x55,
    0x55,
    0xaa,
    0xaa,
    0xff,
    0xff,
    0x55,
    0x55,
    0xaa,
    0xaa,
    0xff,
    0xff,
    0x00,
    0x00,
    0x55,
    0x55,
    0xaa,
    0xaa,
    0xff,
    0xff,
    0x00,
    0x00,
    0xaa,
    0xaa,
    0xff,
    0xff,
    0x00,
    0x00,
    0x55,
    0x55,
    0xaa,
    0xaa,
    0xff,
    0xff,
    0x00,
    0x00,
    0x55,
    0x55,
    0xff,
    0xff,
    0x00,
    0x00,
    0x55,
    0x55,
    0xaa,
    0xaa,
    0xff,
    0xff,
    0x00,
    0x00,
    0x55,
    0x55,
    0xaa,
    0xaa,
};
const uint8_t pict4bpp[] = {
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x11,
    0x11,
    0x11,
    0x11,
    0x11,
    0x11,
    0x00,
    0x00,
    0x11,
    0x11,
    0x11,
    0x11,
    0x11,
    0x11,
    0x00,
    0x00,
    0x11,
    0x22,
    0x22,
    0x22,
    0x22,
    0x11,
    0x00,
    0x00,
    0x11,
    0x22,
    0x22,
    0x22,
    0x22,
    0x11,
    0x00,
    0x00,
    0x11,
    0x22,
    0x33,
    0x33,
    0x22,
    0x11,
    0x00,
    0x00,
    0x11,
    0x22,
    0x33,
    0x33,
    0x22,
    0x11,
    0x00,
    0x00,
    0x11,
    0x22,
    0x33,
    0x33,
    0x22,
    0x11,
    0x00,
    0x00,
    0x11,
    0x22,
    0x33,
    0x33,
    0x22,
    0x11,
    0x00,
    0x00,
    0x11,
    0x22,
    0x22,
    0x22,
    0x22,
    0x11,
    0x00,
    0x00,
    0x11,
    0x22,
    0x22,
    0x22,
    0x22,
    0x11,
    0x00,
    0x00,
    0x11,
    0x11,
    0x11,
    0x11,
    0x11,
    0x11,
    0x00,
    0x00,
    0x11,
    0x11,
    0x11,
    0x11,
    0x11,
    0x11,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
};



void drawTestScreen() {
    Serial.printf("Use FB: %d ", use_fb);
    Serial.flush();
    tft.useFrameBuffer(use_fb);
    SetupOrClearClipRectAndOffsets();
    uint32_t start_time = millis();
    tft.fillScreen(use_fb ? HX8357_RED : HX8357_BLACK);
    //tft.setFont(Inconsolata_60);
    tft.setFont(Arial_24_Bold);
    tft.setTextColor(HX8357_WHITE);
    tft.setCursor(0, 0);
    tft.println("Test");
    tft.setTextColor(HX8357_WHITE, HX8357_RED);
    tft.println("text");
    tft.setCursor(85, 65);
    tft.print("XYZ");
    tft.setFontAdafruit();
    tft.setTextSize(2);
    tft.setTextColor(HX8357_WHITE);
    tft.println("01234");
    tft.setTextColor(HX8357_WHITE, HX8357_GREEN);
    tft.println("56789!@#$%");

    tft.drawRect(0, 150, 100, 50, HX8357_WHITE);
    tft.drawLine(0, 150, 100, 50, HX8357_GREEN);
    tft.fillRectVGradient(125, 150, 50, 50, HX8357_GREEN, HX8357_YELLOW);
    tft.fillRectHGradient(200, 150, 50, 50, HX8357_YELLOW, HX8357_GREEN);

// Try a read rect and write rect
#define BAND_WIDTH 8
#define BAND_HEIGHT 20
#define BAND_START_X 200
#define BAND_START_Y 259

    tft.fillRect(BAND_START_X + BAND_WIDTH * 0, BAND_START_Y, BAND_WIDTH, BAND_HEIGHT, HX8357_RED);
    tft.fillRect(BAND_START_X + BAND_WIDTH * 1, BAND_START_Y, BAND_WIDTH, BAND_HEIGHT, HX8357_GREEN);
    tft.fillRect(BAND_START_X + BAND_WIDTH * 2, BAND_START_Y, BAND_WIDTH, BAND_HEIGHT, HX8357_BLUE);
    tft.fillRect(BAND_START_X + BAND_WIDTH * 3, BAND_START_Y, BAND_WIDTH, BAND_HEIGHT, HX8357_BLACK);
    tft.fillRect(BAND_START_X + BAND_WIDTH * 4, BAND_START_Y, BAND_WIDTH, BAND_HEIGHT, HX8357_WHITE);
    tft.fillRect(BAND_START_X + BAND_WIDTH * 5, BAND_START_Y, BAND_WIDTH, BAND_HEIGHT, HX8357_YELLOW);
    tft.fillRect(BAND_START_X + BAND_WIDTH * 6, BAND_START_Y, BAND_WIDTH, BAND_HEIGHT, HX8357_CYAN);
    tft.fillRect(BAND_START_X + BAND_WIDTH * 7, BAND_START_Y, BAND_WIDTH, BAND_HEIGHT, HX8357_PINK);
    memset(pixel_data, 0, sizeof(pixel_data));
    tft.readRect(BAND_START_X, BAND_START_Y, BAND_WIDTH * 8, BAND_HEIGHT, pixel_data);
    Serial.printf("%04X %04X %04X %04X %04X %04X %04X %04X\n",
                  HX8357_RED, HX8357_GREEN, HX8357_BLUE, HX8357_BLACK, HX8357_WHITE, HX8357_YELLOW, HX8357_CYAN, HX8357_PINK);
    MemoryHexDump(Serial, pixel_data, BAND_WIDTH * 8 * 2, true, "\nColor bars:\n");

    tft.writeRect(BAND_START_X, BAND_START_Y + BAND_HEIGHT + 3, BAND_WIDTH * 8, BAND_HEIGHT, pixel_data);
    //WaitForUserInput();

    tft.readRect(0, 0, 50, 50, pixel_data);
    //    MemoryHexDump(Serial, pixel_data, 1024, true);
    // For heck of it lets make sure readPixel and ReadRect
    // give us same data, maybe check along diagnal?
    for (uint i = 0; i < 50; i++) {
        uint16_t pixel_color = tft.readPixel(i, i);
        if (pixel_color != pixel_data[i * 50 + i]) {
            Serial.printf("Read rect/pixel mismatch: %d %x %x\n", i, pixel_color, pixel_data[i * 50 + i]);
        }
    }

#ifdef DEBUG_PIN
    digitalWrite(DEBUG_PIN, LOW);
#endif
    tft.writeRect(250, 0, 50, 50, pixel_data);

    // Lets try to pack this rectangle of data into 8 byte
    tft.readRect(85, 65, 50, 50, pixel_data);
    uint16_t *ppd16 = pixel_data;
    uint8_t *ppd8 = (uint8_t *)pixel_data;
    uint8_t palette_cnt = 0;
    int palette_index;
    for (int i = 0; i < 2500; i++) {
        for (palette_index = 0; palette_index < palette_cnt; palette_index++) {
            if (*ppd16 == palette[palette_index])
                break;
        }
        if (palette_index >= palette_cnt) {
            palette[palette_cnt++] = *ppd16;  // save away the color
        }
        *ppd8++ = palette_index;
        ppd16++;
    }
    tft.writeRect8BPP(200, 50, 50, 50, (uint8_t *)pixel_data, palette);
    palette[0] = HX8357_CYAN;
    palette[1] = HX8357_OLIVE;
    tft.writeRect1BPP(75, 100, 16, 16, pict1bpp, palette);
    tft.writeRect1BPP(320 - 90, 75, 16, 16, pict1bpp, palette);

    palette[2] = HX8357_MAROON;
    palette[3] = HX8357_PINK;
    tft.writeRect2BPP(75, 125, 32, 16, pict2bpp, palette);

    tft.writeRectNBPP(15, 125, 32, 16, 2, pict2bpp, palette);
    tft.writeRectNBPP(75, 150, 16, 16, 4, pict4bpp, palette);

    // Try drawing button
    tft.setFontAdafruit();
    //button.drawButton();
    // Lets fill up some more of the larger screen.

    tft.fillCircle(380, 220, 80, HX8357_GREEN);
    tft.fillCircle(380, 220, 60, HX8357_BLUE);
    tft.drawCircle(380, 220, 40, HX8357_PINK);
    tft.drawCircle(380, 220, 20, HX8357_YELLOW);

    tft.fillTriangle(20, 300, 170, 300, 95, 240, HX8357_GREEN);
    tft.fillTriangle(40, 280, 150, 280, 95, 220, HX8357_PINK);
    tft.drawTriangle(60, 260, 130, 260, 95, 200, HX8357_YELLOW);
    tft.drawTriangle(80, 240, 110, 240, 95, 180, HX8357_BLUE);

    tft.setFont(&FreeMonoBoldOblique12pt7b);
    tft.setCursor(250, 50);
    tft.setTextColor(HX8357_WHITE);
    tft.println("Adafruit");
    tft.setCursor(250, tft.getCursorY());
    tft.setTextColor(HX8357_WHITE, HX8357_GREEN);
    tft.println("MonoBold");

    // Lets see the colors at the 4 corners:
    Serial.printf("UL:%x UR:%x, LL:%x, LR:%x\n", tft.readPixel(0, 0), tft.readPixel(tft.width()-1, 0),
        tft.readPixel(tft.height()-1, 0), tft.readPixel(tft.width()-1, tft.height()-1));

    if (use_dma) {
        tft.updateScreenAsync();
    } else {
        tft.updateScreen();
    }

    Serial.println(millis() - start_time, DEC);


    use_fb = use_fb ? 0 : 1;
    Serial.println(use_fb, DEC);
}

void fillScreenTest() {
    tft.useFrameBuffer(0);
    SetupOrClearClipRectAndOffsets();

    tft.fillScreen(HX8357_RED);
    WaitForUserInput();
    tft.fillScreen(HX8357_GREEN);
    WaitForUserInput();
    tft.fillScreen(HX8357_WHITE);
    WaitForUserInput();
    tft.fillScreen(HX8357_BLACK);
}
void printTextSizes(const char *sz) {
    Serial.printf("%s(%d,%d): SPL:%u ", sz, tft.getCursorX(), tft.getCursorY(), tft.strPixelLen(sz));
    int16_t x, y;
    uint16_t w, h;
    tft.getTextBounds(sz, tft.getCursorX(), tft.getCursorY(), &x, &y, &w, &h);
    Serial.printf(" Rect(%d, %d, %u %u)\n", x, y, w, h);
    tft.drawRect(x, y, w, h, HX8357_GREEN);
}


void drawTextScreen(bool fOpaque) {
    SetupOrClearClipRectAndOffsets();
    tft.setTextSize(1);
    uint32_t start_time = millis();
    tft.useFrameBuffer(use_fb);
    tft.fillScreen(use_fb ? HX8357_RED : HX8357_BLACK);
    tft.setFont(Arial_28_Bold);
    //t  tft.setFont(Arial_40_Bold);
    if (fOpaque)
        tft.setTextColor(HX8357_WHITE, use_fb ? HX8357_BLACK : HX8357_RED);
    else
        tft.setTextColor(HX8357_WHITE);
    tft.setCursor(0, 5);
    tft.println("AbCdEfGhIj");
#if 1
    tft.setFont(Arial_28_Bold);
    tft.println("0123456789!@#$");
    tft.setFont(Arial_20_Bold);
    tft.println("abcdefghijklmnopq");
    tft.setFont(Arial_14_Bold);
    tft.println("ABCDEFGHIJKLMNOPQRST");
    tft.setFont(Arial_10_Bold);
    tft.println("0123456789zyxwvutu");
#endif
    tft.setFont(&FreeMonoBoldOblique12pt7b);
    printTextSizes("AdaFruit_MB_12");
    if (fOpaque) {
        tft.setTextColor(HX8357_RED, HX8357_BLUE);
        tft.print("A");
        tft.setTextColor(HX8357_WHITE, HX8357_GREEN);
        tft.print("d");
        tft.setTextColor(HX8357_RED, HX8357_BLUE);
        tft.print("a");
        tft.setTextColor(HX8357_WHITE, HX8357_GREEN);
        tft.print("F");
        tft.setTextColor(HX8357_RED, HX8357_BLUE);
        tft.print("R");
        tft.setTextColor(HX8357_WHITE, HX8357_GREEN);
        tft.print("u");
        tft.setTextColor(HX8357_RED, HX8357_BLUE);
        tft.print("i");
        tft.setTextColor(HX8357_WHITE, HX8357_GREEN);
        tft.print("t");
        tft.setTextColor(HX8357_RED, HX8357_BLUE);
        tft.print("_");
        tft.setTextColor(HX8357_WHITE, HX8357_GREEN);
        tft.print("M");
        tft.setTextColor(HX8357_RED, HX8357_BLUE);
        tft.print("B");
        tft.setTextColor(HX8357_WHITE, HX8357_GREEN);
        tft.print("_");
        tft.setTextColor(HX8357_RED, HX8357_BLUE);
        tft.print("1");
        tft.setTextColor(HX8357_WHITE, HX8357_GREEN);
        tft.println("2");
        tft.setTextColor(HX8357_WHITE, use_fb ? HX8357_BLACK : HX8357_RED);
    } else tft.println("AdaFruit_MB_12");
    tft.setFont(&FreeSerif12pt7b);
    printTextSizes("FreeSan12");
    tft.println("FreeSan12");
    tft.println();
    tft.setTextSize(1, 3);
    printTextSizes("Size 1,3");
    tft.println("Size 1,3");
    tft.setFont();
    tft.setCursor(0, 190);
    tft.setTextSize(1, 2);
    printTextSizes("Sys(1,2)");
    tft.println("Sys(1,2)");
    tft.setTextSize(1);
    printTextSizes("System");
    tft.println("System");
    tft.setTextSize(1);


    tft.updateScreen();
    Serial.printf("Use FB: %d OP: %d, DT: %d OR: %d\n", use_fb, fOpaque, use_set_origin, millis() - start_time);
}


void drawGFXTextScreen(bool fOpaque) {
    SetupOrClearClipRectAndOffsets();
    tft.setTextSize(1);
    tft.setCursor(0, 10);
    if (fOpaque)
        tft.setTextColor(HX8357_WHITE, use_fb ? HX8357_BLACK : HX8357_RED);
    else
        tft.setTextColor(HX8357_WHITE);
    uint32_t start_time = millis();
    tft.useFrameBuffer(use_fb);
    tft.fillScreen(use_fb ? HX8357_RED : HX8357_BLACK);
    tft.setFont(&FreeMonoBoldOblique12pt7b);
    tft.println("MonoBold");
    tft.println("ABCDEFGHIJKLMNO");
    tft.println("abcdefghijklmno");
    tft.println("0123456789!@#$%^&*()_");
    tft.setFont(&FreeSerif12pt7b);
    tft.println("Serif12");
    tft.println("ABCDEFGHIJKLMNO");
    tft.println("abcdefghijklmno");
    tft.println("0123456789!@#$%^&*()_");
    tft.updateScreen();
    tft.setTextSize(1);
    tft.setFont();
    Serial.printf("Use FB: %d OP: %d, DT: %d\n", use_fb, fOpaque, millis() - start_time);
}
//=============================================================================
// Wait for user input
//=============================================================================
void WaitForUserInput() {
    Serial.println("Hit Enter to continue");
    Serial.flush();
    while (Serial.read() == -1)
        ;
    while (Serial.read() != -1)
        ;
}


void loop(void) {
    // See if any text entered
    int ich;
    if ((ich = Serial.read()) != -1) {
        while (Serial.read() != -1) delay(1);

        // See if We have a dma operation in progress?

        if (ich == 'c') {
            use_clip_rect = !use_clip_rect;
            if (use_clip_rect) Serial.println("Clip Rectangle Turned on");
            else Serial.println("Clip Rectangle turned off");
            return;
        }
        if (ich == 'd') {
            use_dma = !use_dma;
            if (use_dma) Serial.println("DMA Turned on");
            else Serial.println("DMA turned off");
            return;
        }

        if (ich == 's') {
            use_set_origin = !use_set_origin;
            if (use_set_origin) Serial.printf("Set origin to %d, %d\n", ORIGIN_TEST_X, ORIGIN_TEST_Y);
            else Serial.println("Clear origin");
            return;
        }
        if (ich == 'o')
            drawTextScreen(1);
        else if (ich == 'f')
            fillScreenTest();
        else if (ich == 't')
            drawTextScreen(0);
        else if (ich == 'g')
            drawGFXTextScreen(0);
        else
            drawTestScreen();
    }
}
