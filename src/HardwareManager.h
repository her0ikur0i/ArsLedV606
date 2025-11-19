#ifndef HARDWAREMANAGER_H
#define HARDWAREMANAGER_H

#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <INA226.h>
#include "SSD1306Wire.h"
#include "config.h"

class HardwareManager {
private:
    RTC_DS3231 rtc;
    INA226 ina226;
    SSD1306Wire display;
    const uint8_t ledPins[NUM_LED_CHANNELS] = {LED_RB_PIN, LED_CW_PIN, LED_B_PIN, LED_FS_PIN};
    float lastWaterTemp = -999.0f;
    float lastSinkTemp = -999.0f;
    float lastRoomTemp = -999.0f;
    float totalKwh = 0.0f;
    unsigned long lastPowerUpdate = 0;
    bool isRTC_OK = false;
    bool isINA226_OK = false;
    bool isOLED_OK = false;
    bool isLED_OK = false;
    bool isFAN_OK = false;
    bool isTEMP_OK = false;
    bool isPCTimeReceived = false;
    char pcTimeStr[9] = "";
    char pcDateStr[11] = "";
    float readThermistor(int pin);
    void setupPWMChannels();
    void updateTemperatures();
    void updatePowerMonitoring();
    void updateFanControl();
    void updateOLEDDisplay(uint8_t brightness[]);
    void scanI2CDevices();
    bool verifyRTC();
    bool verifyINA226();
    bool verifyOLED();
public:
    HardwareManager();
    void begin();
    void run(uint8_t brightness[]);
    void setChannelBrightness(uint8_t channel, uint8_t percent);
    void testLEDHardware();
    float getCurrentWatts();
    float getTotalKwh() { return totalKwh; }
    float getCostRp();
    DateTime getRTCTime();
    uint8_t getCurrentFanSpeed();
    float getWaterTemp() { return lastWaterTemp; }
    float getRoomTemp() { return lastRoomTemp; }
    float getSinkTemp() { return lastSinkTemp; }
    void adjustRTCTime(int h, int m, int s);
    void adjustRTCDate(int y, int m, int d);
    void printBootSummary();
    void printLiveDashboard(uint8_t brightness[]);
    bool isPWM_OK = false;
};

#endif
