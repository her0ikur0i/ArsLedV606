#!/bin/bash

echo "=== ArsLed V606 Final - Ceiling Fix Edition ==="
echo "Base: v606 + MAX_PERCENT_LIMIT as ceiling (not clamping)"
echo "Peak brightness now percentage of MAX_PERCENT_LIMIT"
echo ""

mkdir -p src

cat <<'EOT_INI' > platformio.ini
[env:lolin_s2_mini]
platform = espressif32
board = lolin_s2_mini
framework = arduino
monitor_speed = 115200
upload_protocol = esptool
upload_speed = 460800
upload_port = COM4
board_build.f_cpu = 240000000L
board_build.arduino.loop_stack_size = 32768
lib_deps = 
    adafruit/RTClib@^2.1.3
    robtillaart/INA226@^0.6.4
    thingpulse/ESP8266 and ESP32 OLED driver for SSD1306 displays@^4.6.1
EOT_INI

cat <<'EOT_SYNC' > sync_time.py
#!/usr/bin/env python3
import serial
import time
from datetime import datetime
import sys

PORT = "COM5" if len(sys.argv) < 2 else sys.argv[1]
BAUD_RATE = 115200
WAIT_TIME = 8

def sync_time():
    now = datetime.now()
    time_str = now.strftime("%H:%M:%S")
    date_str = now.strftime("%y:%m:%d")
    
    print(f"\n=== ArsLed V606 Manual Time Sync ===")
    print(f"Port: {PORT}")
    print(f"Time: {time_str}")
    print(f"Date: {date_str}")
    print(f"\nWaiting {WAIT_TIME} seconds for ESP32 stability...")
    
    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=2)
        time.sleep(WAIT_TIME)
        
        print("\n[1/2] Sending SETTIME command...")
        ser.write(f"SETTIME={time_str}\n".encode('utf-8'))
        ser.flush()
        time.sleep(1)
        
        if ser.in_waiting:
            print(f"Response: {ser.read(ser.in_waiting).decode('utf-8', errors='ignore').strip()}")
        
        print("\n[2/2] Sending SETDATE command...")
        ser.write(f"SETDATE={date_str}\n".encode('utf-8'))
        ser.flush()
        time.sleep(1)
        
        if ser.in_waiting:
            print(f"Response: {ser.read(ser.in_waiting).decode('utf-8', errors='ignore').strip()}")
        
        ser.close()
        print("\n✓ Time synchronization completed!")
        
    except serial.SerialException as e:
        print(f"\n✗ ERROR: {e}")
        sys.exit(1)

if __name__ == "__main__":
    sync_time()
EOT_SYNC
chmod +x sync_time.py

cat <<'EOT_CONFIG' > src/config.h
#ifndef CONFIG_H
#define CONFIG_H

#define NUM_LED_CHANNELS 4
#define LED_RB_PIN 4
#define LED_CW_PIN 5
#define LED_B_PIN 6
#define LED_FS_PIN 7
#define ONBOARD_LED_PIN 15
#define I2C_SDA_PIN 1
#define I2C_SCL_PIN 2
#define THERM_WATER_PIN 8
#define THERM_SINK_PIN 9
#define THERM_ROOM_PIN 10
#define FAN_PIN 3

#define PWM_FREQUENCY 5000UL
#define PWM_RESOLUTION_BITS 12
#define MAX_DUTY_VALUE ((1 << PWM_RESOLUTION_BITS) - 1)

// MAX_PERCENT_LIMIT = ceiling/batas maksimal kecerahan (untuk menjaga usia LED)
#define MAX_PERCENT_LIMIT 90

#define CH_RB 0
#define CH_CW 1
#define CH_B 2
#define CH_FS 3

#define SCHED_ON_HOUR 8
#define SCHED_ON_MINUTE 0
#define SCHED_PEAK_START_HOUR 11
#define SCHED_PEAK_START_MINUTE 0
#define SCHED_PEAK_END_HOUR 15
#define SCHED_PEAK_END_MINUTE 0
#define SCHED_OFF_HOUR 21
#define SCHED_OFF_MINUTE 0

// Peak brightness = persentase dari MAX_PERCENT_LIMIT
// Contoh: PEAK_BRIGHT_RB = 90 berarti 90% dari 90% = 81% duty cycle aktual
#define PEAK_BRIGHT_RB 90
#define PEAK_BRIGHT_CW 10
#define PEAK_BRIGHT_B 100
#define PEAK_BRIGHT_FS 10

#define DEFAULT_KWH_COST 1444
#define LIVE_DASHBOARD_INTERVAL 60000UL
#define HEARTBEAT_INTERVAL 60000UL

#define THERM_NOMINAL_RESISTANCE 10000.0f
#define THERM_NOMINAL_TEMP_K 298.15f
#define THERM_BETA_VALUE 3950.0f
#define THERM_SERIES_RESISTOR 10000.0f

#endif
EOT_CONFIG

cat <<'EOT_SCHED_H' > src/Scheduler.h
#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <Arduino.h>
#include "config.h"

enum SchedulePhase {PHASE_OFF, PHASE_SUNRISE, PHASE_PEAK, PHASE_SUNSET};

class Scheduler {
private:
    uint8_t peakBrightness[NUM_LED_CHANNELS] = {PEAK_BRIGHT_RB, PEAK_BRIGHT_CW, PEAK_BRIGHT_B, PEAK_BRIGHT_FS};
    uint8_t currentBrightness[NUM_LED_CHANNELS];
    float cubicBezier(float t);
    void calculateAutoBrightness(int currentMinutes);
public:
    Scheduler();
    void begin();
    void run(int currentHour, int currentMinute);
    uint8_t getBrightness(uint8_t channel);
    SchedulePhase getCurrentPhase(int currentHour, int currentMinute);
};

#endif
EOT_SCHED_H

cat <<'EOT_SCHED_CPP' > src/Scheduler.cpp
#include "Scheduler.h"

Scheduler::Scheduler() {
    for(int i = 0; i < NUM_LED_CHANNELS; i++) currentBrightness[i] = 0;
}

void Scheduler::begin() {}

float Scheduler::cubicBezier(float t) {
    float t2 = t * t, t3 = t2 * t, mt = 1 - t, mt2 = mt * mt;
    return 3 * mt2 * t * 0.42 + 3 * mt * t2 * 0.58 + t3;
}

SchedulePhase Scheduler::getCurrentPhase(int currentHour, int currentMinute) {
    int currentMinutes = currentHour * 60 + currentMinute;
    const int scheduleOn = SCHED_ON_HOUR * 60 + SCHED_ON_MINUTE;
    const int schedulePeakStart = SCHED_PEAK_START_HOUR * 60 + SCHED_PEAK_START_MINUTE;
    const int schedulePeakEnd = SCHED_PEAK_END_HOUR * 60 + SCHED_PEAK_END_MINUTE;
    const int scheduleOff = SCHED_OFF_HOUR * 60 + SCHED_OFF_MINUTE;
    
    if(currentMinutes < scheduleOn || currentMinutes >= scheduleOff) return PHASE_OFF;
    else if(currentMinutes < schedulePeakStart) return PHASE_SUNRISE;
    else if(currentMinutes < schedulePeakEnd) return PHASE_PEAK;
    else return PHASE_SUNSET;
}

void Scheduler::calculateAutoBrightness(int currentMinutes) {
    const int scheduleOn = SCHED_ON_HOUR * 60 + SCHED_ON_MINUTE;
    const int schedulePeakStart = SCHED_PEAK_START_HOUR * 60 + SCHED_PEAK_START_MINUTE;
    const int schedulePeakEnd = SCHED_PEAK_END_HOUR * 60 + SCHED_PEAK_END_MINUTE;
    const int scheduleOff = SCHED_OFF_HOUR * 60 + SCHED_OFF_MINUTE;
    
    SchedulePhase phase;
    if(currentMinutes < scheduleOn || currentMinutes >= scheduleOff) phase = PHASE_OFF;
    else if(currentMinutes < schedulePeakStart) phase = PHASE_SUNRISE;
    else if(currentMinutes < schedulePeakEnd) phase = PHASE_PEAK;
    else phase = PHASE_SUNSET;
    
    for(int i = 0; i < NUM_LED_CHANNELS; i++) {
        float brightness = 0.0f;
        switch(phase) {
            case PHASE_OFF: brightness = 0.0f; break;
            case PHASE_SUNRISE: {
                int duration = schedulePeakStart - scheduleOn;
                int elapsed = currentMinutes - scheduleOn;
                float progress = (float)elapsed / (float)duration;
                brightness = cubicBezier(progress) * peakBrightness[i];
                break;
            }
            case PHASE_PEAK: brightness = peakBrightness[i]; break;
            case PHASE_SUNSET: {
                int duration = scheduleOff - schedulePeakEnd;
                int elapsed = currentMinutes - schedulePeakEnd;
                float progress = (float)elapsed / (float)duration;
                brightness = peakBrightness[i] * (1.0f - cubicBezier(progress));
                break;
            }
        }
        currentBrightness[i] = (uint8_t)constrain(brightness, 0, 100);
    }
}

void Scheduler::run(int currentHour, int currentMinute) {
    int currentMinutes = currentHour * 60 + currentMinute;
    calculateAutoBrightness(currentMinutes);
}

uint8_t Scheduler::getBrightness(uint8_t channel) {
    if(channel >= NUM_LED_CHANNELS) return 0;
    return currentBrightness[channel];
}
EOT_SCHED_CPP

cat <<'EOT_HW_H' > src/HardwareManager.h
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
EOT_HW_H

cat <<'EOT_HW_CPP' > src/HardwareManager.cpp
#include "HardwareManager.h"
#include <math.h>

HardwareManager::HardwareManager() : ina226(0x40), display(0x3C, I2C_SDA_PIN, I2C_SCL_PIN) {}

float HardwareManager::readThermistor(int pin) {
    int raw = analogRead(pin);
    if (raw < 10) return -999.0f;
    float vout = (float)raw;
    float resistance = THERM_SERIES_RESISTOR / ((4095.0f / vout) - 1.0f);
    float steinhart = resistance / THERM_NOMINAL_RESISTANCE;
    steinhart = log(steinhart);
    steinhart /= THERM_BETA_VALUE;
    steinhart += (1.0f / THERM_NOMINAL_TEMP_K);
    steinhart = 1.0f / steinhart;
    return steinhart - 273.15f;
}

void HardwareManager::scanI2CDevices() {
    Serial.println("\nI2C Scanner: Scanning...");
    byte count = 0;
    for(byte addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        byte error = Wire.endTransmission();
        if(error == 0) {
            Serial.printf("  Device found at 0x%02X", addr);
            if(addr == 0x3C) Serial.print(" (OLED SSD1306?)");
            else if(addr == 0x40) Serial.print(" (INA226?)");
            else if(addr == 0x68) Serial.print(" (RTC DS3231?)");
            Serial.println();
            count++;
        }
    }
    if(count == 0) Serial.println("  No I2C devices found!");
    else Serial.printf("I2C Scanner: Found %d device(s)\n", count);
    Serial.println();
}

bool HardwareManager::verifyRTC() {
    if(!rtc.begin()) return false;
    DateTime now = rtc.now();
    return (now.year() >= 2000 && now.year() < 2100);
}

bool HardwareManager::verifyINA226() {
    if(!ina226.begin()) return false;
    delay(10);
    float voltage = ina226.getBusVoltage_mV();
    return (voltage >= 0.0f && voltage <= 36000.0f);
}

bool HardwareManager::verifyOLED() {
    Wire.beginTransmission(0x3C);
    byte err = Wire.endTransmission();
    if(err != 0) {
        Serial.println("[OLED] No ACK at 0x3C");
        return false;
    }
    display.init();
    delay(10);
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "ArsLed V606");
    display.display();
    Serial.println("[OLED] SSD1306 initialized (ThingPulse)");
    isOLED_OK = true;
    return true;
}

void HardwareManager::testLEDHardware() {
    Serial.println("\n=== LED HARDWARE TEST ===");
    Serial.println("Testing each channel for 2 seconds at 50%...");
    const char* channelNames[] = {"RB (Royal Blue)", "CW (Cool White)", "B (Blue)", "FS (Full Spec)"};
    for(int i = 0; i < NUM_LED_CHANNELS; i++) {
        Serial.printf("\nChannel %d: %s (GPIO %d)\n", i, channelNames[i], ledPins[i]);
        Serial.println("LED should be ON now...");
        // Test menggunakan 50% dari MAX_PERCENT_LIMIT
        setChannelBrightness(i, 50);
        delay(2000);
        setChannelBrightness(i, 0);
        Serial.println("LED OFF");
        delay(500);
    }
    Serial.println("\n=== ALL CHANNELS TEST (50%) ===");
    Serial.println("All LEDs ON for 3 seconds...");
    for(int i = 0; i < NUM_LED_CHANNELS; i++) {
        setChannelBrightness(i, 50);
    }
    delay(3000);
    for(int i = 0; i < NUM_LED_CHANNELS; i++) {
        setChannelBrightness(i, 0);
    }
    Serial.println("All LEDs OFF");
    Serial.println("=== HARDWARE TEST COMPLETE ===\n");
}

void HardwareManager::begin() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    delay(100);
    scanI2CDevices();
    Serial.println("HardwareManager: Initializing...");
    if(verifyOLED()) isOLED_OK = true;
    if(verifyRTC()) isRTC_OK = true;
    if(verifyINA226()) isINA226_OK = true;
    pinMode(THERM_WATER_PIN, INPUT);
    pinMode(THERM_SINK_PIN, INPUT);
    pinMode(THERM_ROOM_PIN, INPUT);
    if(readThermistor(THERM_WATER_PIN) > -50.0f && readThermistor(THERM_SINK_PIN) > -50.0f) isTEMP_OK = true;
    setupPWMChannels();
    isPWM_OK = true;
    isLED_OK = true;
    isFAN_OK = true;
    Serial.println();
    if(isOLED_OK) delay(500);
}

void HardwareManager::printBootSummary() {
    DateTime now = getRTCTime();
    int readyCount = 0;
    if(isOLED_OK) readyCount++;
    if(isRTC_OK) readyCount++;
    if(isINA226_OK) readyCount++;
    if(isLED_OK) readyCount++;
    if(isFAN_OK) readyCount++;
    if(isTEMP_OK) readyCount++;
    int readyPercent = (readyCount * 100) / 6;
    Serial.println("\n=============================================");
    Serial.println("=== ArsLed Intelligent Marine Light V606 ===");
    Serial.println("System Status:");
    if(isPCTimeReceived && isRTC_OK) {
        Serial.printf("PC Time:  %s %s\n", pcTimeStr, pcDateStr);
        Serial.printf("RTC Time: %02d:%02d:%02d %02d/%02d/%04d\n", now.hour(), now.minute(), now.second(), now.day(), now.month(), now.year());
    } else if(isRTC_OK) {
        Serial.printf("RTC Time: %02d:%02d:%02d %02d/%02d/%04d\n", now.hour(), now.minute(), now.second(), now.day(), now.month(), now.year());
    } else if(isPCTimeReceived) {
        Serial.printf("PC Time:  %s %s\n", pcTimeStr, pcDateStr);
    } else {
        Serial.println("System Time: No Time Source Available");
    }
    Serial.println("---------------------------------------------");
    Serial.println("Hardware Status:");
    Serial.printf("OLED: %s\n", isOLED_OK ? "Ready" : "Failed");
    Serial.printf("RTC: %s\n", isRTC_OK ? "Ready" : "Failed");
    Serial.printf("INA226: %s\n", isINA226_OK ? "Ready" : "Failed");
    Serial.printf("LED: %s\n", isLED_OK ? "Ready" : "Failed");
    Serial.printf("FAN: %s\n", isFAN_OK ? "Ready" : "Failed");
    Serial.printf("TEMP: %s\n", isTEMP_OK ? "Ready" : "Failed");
    Serial.println("---------------------------------------------");
    Serial.printf("Hardware Manager: %d%% Ready (%d/6)\n", readyPercent, readyCount);
    if(isRTC_OK && isINA226_OK && isTEMP_OK) Serial.println("System: Ready!");
    else Serial.println("System: Limited Functionality (Testing Mode)");
    Serial.println("=============================================\n");
}

void HardwareManager::printLiveDashboard(uint8_t brightness[]) {
    DateTime now = getRTCTime();
    Serial.println("\n=============================================");
    Serial.println("=== ArsLed V606 - Live Monitoring ===");
    Serial.printf("Time: %02d:%02d:%02d %02d/%02d/%04d\n", now.hour(), now.minute(), now.second(), now.day(), now.month(), now.year());
    Serial.println("---------------------------------------------");
    if(lastWaterTemp < -50.0f) Serial.print("Water Temp: N/A | ");
    else Serial.printf("Water Temp: %.2fC | ", lastWaterTemp);
    if(lastRoomTemp < -50.0f) Serial.print("Room Temp: N/A\n");
    else Serial.printf("Room Temp: %.2fC\n", lastRoomTemp);
    if(lastSinkTemp < -50.0f) Serial.print("Sink Temp: N/A | ");
    else Serial.printf("Sink Temp:  %.2fC | ", lastSinkTemp);
    Serial.printf("Fan Speed: %d%%\n", getCurrentFanSpeed());
    if(isINA226_OK) {
        Serial.printf("Power: %.2fW | Total kWh: %.3f\n", getCurrentWatts(), totalKwh);
        Serial.printf("Cost (Rp): %.2f\n", getCostRp());
    } else {
        Serial.println("Power: N/A | Total kWh: N/A");
        Serial.println("Cost (Rp): N/A");
    }
    Serial.printf("LED Brightness: RB:%d CW:%d B:%d FS:%d\n", brightness[CH_RB], brightness[CH_CW], brightness[CH_B], brightness[CH_FS]);
    
    // Tambahkan info actual output setelah ceiling
    Serial.printf("Actual Output (ceiling %d%%): RB:%.1f CW:%.1f B:%.1f FS:%.1f\n", 
        MAX_PERCENT_LIMIT,
        (brightness[CH_RB] * MAX_PERCENT_LIMIT) / 100.0f,
        (brightness[CH_CW] * MAX_PERCENT_LIMIT) / 100.0f,
        (brightness[CH_B] * MAX_PERCENT_LIMIT) / 100.0f,
        (brightness[CH_FS] * MAX_PERCENT_LIMIT) / 100.0f
    );
    
    float avgBright = 0;
    for(int i = 0; i < NUM_LED_CHANNELS; i++) avgBright += brightness[i];
    avgBright /= NUM_LED_CHANNELS;
    Serial.printf("Average Brightness: %.0f%%\n", avgBright);
    Serial.println("=============================================\n");
}

void HardwareManager::updateTemperatures() {
    lastWaterTemp = readThermistor(THERM_WATER_PIN);
    lastRoomTemp = readThermistor(THERM_ROOM_PIN);
    lastSinkTemp = readThermistor(THERM_SINK_PIN);
}

void HardwareManager::updatePowerMonitoring() {
    if(!isINA226_OK) return;
    unsigned long currentTime = millis();
    if(currentTime - lastPowerUpdate >= 1000) {
        float watts = getCurrentWatts();
        float hours = (currentTime - lastPowerUpdate) / 3600000.0f;
        totalKwh += (watts / 1000.0f) * hours;
        lastPowerUpdate = currentTime;
    }
}

void HardwareManager::updateFanControl() {
    uint8_t fanSpeed = getCurrentFanSpeed();
    uint16_t duty = (uint16_t)(((float)fanSpeed / 100.0f) * MAX_DUTY_VALUE);
    ledcWrite(NUM_LED_CHANNELS, duty);
}

float HardwareManager::getCurrentWatts() {
    if(!isINA226_OK) return 0.0f;
    float voltage = ina226.getBusVoltage_mV();
    if(voltage < 0.0f) return 0.0f;
    return (voltage / 1000.0f) * ina226.getCurrent_mA() / 1000.0f;
}

float HardwareManager::getCostRp() {
    return totalKwh * DEFAULT_KWH_COST;
}

uint8_t HardwareManager::getCurrentFanSpeed() {
    if(lastSinkTemp < 50.0f) return 0;
    int tempDiff = (int)lastSinkTemp - 50;
    int fanIncrease = (tempDiff / 10) * 10;
    int fanSpeed = 50 + fanIncrease;
    return (uint8_t)constrain(fanSpeed, 50, 90);
}

DateTime HardwareManager::getRTCTime() {
    if(!isRTC_OK) {
        if(isPCTimeReceived) {
            int hh=0, mm=0, ss=0;
            if(sscanf(pcTimeStr, "%02d:%02d:%02d", &hh, &mm, &ss) == 3) {
                return DateTime(2025,1,1,hh,mm,ss);
            }
        }
        return DateTime(2025, 1, 1, 0, 0, 0);
    }
    return rtc.now();
}

void HardwareManager::setupPWMChannels() {
    Serial.println("Setting up PWM channels...");
    for(int i = 0; i < NUM_LED_CHANNELS; i++) {
        ledcSetup(i, PWM_FREQUENCY, PWM_RESOLUTION_BITS);
        ledcAttachPin(ledPins[i], i);
        ledcWrite(i, 0);
        Serial.printf("  CH%d: GPIO %d configured (Freq=%dHz, Res=%dbit)\n", i, ledPins[i], PWM_FREQUENCY, PWM_RESOLUTION_BITS);
    }
    ledcSetup(NUM_LED_CHANNELS, PWM_FREQUENCY, PWM_RESOLUTION_BITS);
    ledcAttachPin(FAN_PIN, NUM_LED_CHANNELS);
    ledcWrite(NUM_LED_CHANNELS, 0);
    Serial.printf("  FAN: GPIO %d configured\n", FAN_PIN);
}

void HardwareManager::setChannelBrightness(uint8_t channel, uint8_t percent) {
    if(channel >= NUM_LED_CHANNELS) return;
    percent = constrain(percent, 0, 100);
    
    // PERBAIKAN: percent adalah persentase dari MAX_PERCENT_LIMIT (ceiling)
    // Contoh: percent=100, MAX_PERCENT_LIMIT=90 → actualPercent=90
    //         percent=50, MAX_PERCENT_LIMIT=90 → actualPercent=45
    float actualPercent = ((float)percent * MAX_PERCENT_LIMIT) / 100.0f;
    
    uint16_t duty = (uint16_t)((actualPercent / 100.0f) * MAX_DUTY_VALUE);
    ledcWrite(channel, duty);
}

void HardwareManager::adjustRTCTime(int h, int m, int s) {
    snprintf(pcTimeStr, sizeof(pcTimeStr), "%02d:%02d:%02d", h, m, s);
    isPCTimeReceived = true;
    digitalWrite(ONBOARD_LED_PIN, HIGH);
    if(!isRTC_OK) {
        Serial.println("RTC not connected. Time stored from PC only.");
        digitalWrite(ONBOARD_LED_PIN, LOW);
        return;
    }
    DateTime rtcNow = rtc.now();
    DateTime newTime(rtcNow.year(), rtcNow.month(), rtcNow.day(), h, m, s);
    rtc.adjust(newTime);
    Serial.printf("✓ RTC Time adjusted to: %02d:%02d:%02d\n", h, m, s);
    digitalWrite(ONBOARD_LED_PIN, LOW);
}

void HardwareManager::adjustRTCDate(int y, int m, int d) {
    int fullYear = (y < 100) ? (y + 2000) : y;
    snprintf(pcDateStr, sizeof(pcDateStr), "%02d/%02d/%04d", d, m, fullYear);
    isPCTimeReceived = true;
    digitalWrite(ONBOARD_LED_PIN, HIGH);
    if(!isRTC_OK) {
        Serial.println("RTC not connected. Date stored from PC only.");
        digitalWrite(ONBOARD_LED_PIN, LOW);
        return;
    }
    DateTime rtcNow = rtc.now();
    DateTime newDate(fullYear, m, d, rtcNow.hour(), rtcNow.minute(), rtcNow.second());
    rtc.adjust(newDate);
    Serial.printf("✓ RTC Date adjusted to: %02d/%02d/%04d\n", d, m, fullYear);
    digitalWrite(ONBOARD_LED_PIN, LOW);
}

void HardwareManager::updateOLEDDisplay(uint8_t brightness[]) {
    if(!isOLED_OK) return;
    char line[64];
    display.clear();
    if(isRTC_OK) {
        DateTime now = rtc.now();
        snprintf(line, sizeof(line), "%02d:%02d:%02d %02d/%02d/%04d", now.hour(), now.minute(), now.second(), now.day(), now.month(), now.year());
        display.drawString(0, 0, line);
    } else if(isPCTimeReceived) {
        snprintf(line, sizeof(line), "%s %s", pcTimeStr, pcDateStr);
        display.drawString(0, 0, line);
    } else {
        display.drawString(0, 0, "No Clock");
    }
    char tbuf[32];
    if(lastWaterTemp < -50.0f) snprintf(tbuf, sizeof(tbuf), "W:N/A");
    else snprintf(tbuf, sizeof(tbuf), "W:%.1f", lastWaterTemp);
    display.drawString(0, 12, tbuf);
    if(lastSinkTemp < -50.0f) snprintf(tbuf, sizeof(tbuf), "S:N/A");
    else snprintf(tbuf, sizeof(tbuf), "S:%.1f", lastSinkTemp);
    display.drawString(64, 12, tbuf);
    if(lastRoomTemp < -50.0f) snprintf(tbuf, sizeof(tbuf), "R:N/A");
    else snprintf(tbuf, sizeof(tbuf), "R:%.1f", lastRoomTemp);
    display.drawString(0, 24, tbuf);
    snprintf(line, sizeof(line), "RB:%d CW:%d", brightness[CH_RB], brightness[CH_CW]);
    display.drawString(0, 36, line);
    snprintf(line, sizeof(line), "B:%d FS:%d", brightness[CH_B], brightness[CH_FS]);
    display.drawString(0, 48, line);
    display.display();
}

void HardwareManager::run(uint8_t brightness[]) {
    updateTemperatures();
    updatePowerMonitoring();
    updateFanControl();
    updateOLEDDisplay(brightness);
}
EOT_HW_CPP

cat <<'EOT_MAIN' > src/main.cpp
#include <Arduino.h>
#include "HardwareManager.h"
#include "Scheduler.h"
#include "config.h"

HardwareManager hw;
Scheduler scheduler;

unsigned long bootTimeStart = 0;
unsigned long lastHeartbeatTime = 0;
unsigned long lastLiveDashboardTime = 0;

void handleSerialCommands() {
    if(Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        command.toUpperCase();
        
        if(command.startsWith("SETTIME=")) {
            String timeStr = command.substring(8);
            int h, m, s;
            if(sscanf(timeStr.c_str(), "%d:%d:%d", &h, &m, &s) == 3) {
                hw.adjustRTCTime(h, m, s);
            } else {
                Serial.println("Format: SETTIME=HH:MM:SS");
            }
        } else if(command.startsWith("SETDATE=")) {
            String dateStr = command.substring(8);
            int y, m, d;
            if(sscanf(dateStr.c_str(), "%d:%d:%d", &y, &m, &d) == 3) {
                hw.adjustRTCDate(y, m, d);
            } else {
                Serial.println("Format: SETDATE=YY:MM:DD");
            }
        } else if(command == "TEST") {
            Serial.println("\n=== HARDWARE TEST MODE ===");
            hw.testLEDHardware();
        } else if(command == "HELP") {
            Serial.println("\n--- COMMANDS ---");
            Serial.println("SETTIME=HH:MM:SS");
            Serial.println("SETDATE=YY:MM:DD");
            Serial.println("TEST (Hardware test)");
            Serial.println("HELP");
            Serial.println("----------------\n");
        } else if(command.length() > 0) {
            Serial.printf("Unknown: %s (type HELP)\n", command.c_str());
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    while(Serial.available()) Serial.read();
    
    pinMode(ONBOARD_LED_PIN, OUTPUT);
    digitalWrite(ONBOARD_LED_PIN, LOW);
    
    Serial.println("\n=== BOOT SEQUENCE ===");
    for(int i = 0; i < 5; i++) {
        digitalWrite(ONBOARD_LED_PIN, HIGH);
        delay(200);
        digitalWrite(ONBOARD_LED_PIN, LOW);
        delay(200);
    }
    
    hw.begin();
    scheduler.begin();
    bootTimeStart = millis();
    
    Serial.println("\n[Waiting 10s for serial commands...]");
    Serial.println("Type TEST to run hardware test");
    unsigned long startWait = millis();
    while(millis() - startWait < 10000) {
        handleSerialCommands();
        delay(10);
    }
    
    hw.printBootSummary();
    
    Serial.println("\n=== ENTERING MAIN LOOP ===");
    Serial.println("LED control via scheduler active");
    Serial.printf("MAX_PERCENT_LIMIT: %d%% (ceiling untuk menjaga usia LED)\n", MAX_PERCENT_LIMIT);
    Serial.println("Peak brightness adalah persentase dari ceiling");
    Serial.println("Type TEST anytime to test hardware\n");
}

void loop() {
    yield();
    handleSerialCommands();
    
    unsigned long currentTime = millis();
    
    if(currentTime - lastHeartbeatTime >= HEARTBEAT_INTERVAL) {
        digitalWrite(ONBOARD_LED_PIN, HIGH);
        delay(50);
        digitalWrite(ONBOARD_LED_PIN, LOW);
        delay(50);
        digitalWrite(ONBOARD_LED_PIN, HIGH);
        delay(50);
        digitalWrite(ONBOARD_LED_PIN, LOW);
        lastHeartbeatTime = currentTime;
    }
    
    DateTime now = hw.getRTCTime();
    scheduler.run(now.hour(), now.minute());
    
    uint8_t currentBrightness[NUM_LED_CHANNELS];
    bool anyOn = false;
    for(int i = 0; i < NUM_LED_CHANNELS; i++) {
        currentBrightness[i] = scheduler.getBrightness(i);
        hw.setChannelBrightness(i, currentBrightness[i]);
        if(currentBrightness[i] > 0) anyOn = true;
    }
    
    static unsigned long lastDebugPrint = 0;
    if(currentTime - lastDebugPrint >= 10000) {
        SchedulePhase phase = scheduler.getCurrentPhase(now.hour(), now.minute());
        const char* phaseStr[] = {"OFF", "SUNRISE", "PEAK", "SUNSET"};
        Serial.printf("[DEBUG] Time:%02d:%02d Phase:%s LED:RB=%d CW=%d B=%d FS=%d", 
            now.hour(), now.minute(), phaseStr[phase], 
            currentBrightness[CH_RB], currentBrightness[CH_CW], 
            currentBrightness[CH_B], currentBrightness[CH_FS]);
        Serial.printf(" (Actual: RB=%.1f CW=%.1f B=%.1f FS=%.1f)\n",
            (currentBrightness[CH_RB] * MAX_PERCENT_LIMIT) / 100.0f,
            (currentBrightness[CH_CW] * MAX_PERCENT_LIMIT) / 100.0f,
            (currentBrightness[CH_B] * MAX_PERCENT_LIMIT) / 100.0f,
            (currentBrightness[CH_FS] * MAX_PERCENT_LIMIT) / 100.0f
        );
        lastDebugPrint = currentTime;
    }
    
    hw.run(currentBrightness);
    
    unsigned long interval = anyOn ? 10000UL : 1800000UL;
    if(currentTime - lastLiveDashboardTime >= interval) {
        hw.printLiveDashboard(currentBrightness);
        lastLiveDashboardTime = currentTime;
    }
    
    delay(100);
}
EOT_MAIN

cat <<'EOT_README' > README.md
# ArsLed Marine Light V606 - Ceiling Fix Edition

## Key Features
- **MAX_PERCENT_LIMIT as CEILING** (not clamping)
- Peak brightness = percentage of MAX_PERCENT_LIMIT
- v602gpt.sh RTC handling (proven working format)
- Correct pin mapping (GPIO 4,5,6,7 for LEDs)
- Lower PWM frequency (5kHz for compatibility)
- Hardware test command (type TEST)
- Debug output every 10s with actual output values

## Critical Concept: MAX_PERCENT_LIMIT as Ceiling

**MAX_PERCENT_LIMIT = 90%** means absolute maximum LED output (untuk menjaga usia LED)

Peak brightness values are **percentages of this ceiling**:

```
PEAK_BRIGHT_RB = 90   → 90% dari 90% = 81% duty cycle actual
PEAK_BRIGHT_CW = 10   → 10% dari 90% = 9% duty cycle actual  
PEAK_BRIGHT_B = 100   → 100% dari 90% = 90% duty cycle actual
PEAK_BRIGHT_FS = 10   → 10% dari 90% = 9% duty cycle actual
```

**Formula:**
```
Actual Output = (PEAK_BRIGHT_X × MAX_PERCENT_LIMIT) / 100
```

## Pin Configuration

**LED Channels:**
- RB (Royal Blue): GPIO 4
- CW (Cool White): GPIO 5  
- B (Blue): GPIO 6
- FS (Full Spec): GPIO 7

**Peripherals:**
- Onboard LED: GPIO 15
- I2C SDA: GPIO 1
- I2C SCL: GPIO 2
- Fan: GPIO 3

**Temperature Sensors:**
- Water: GPIO 8
- Heatsink: GPIO 9
- Room: GPIO 10

## Quick Start

```bash
chmod +x 606_fixed_ceiling.sh
./606_fixed_ceiling.sh
pio run -t upload
pio device monitor
```

## Commands

- **TEST**: Hardware test (50% dari ceiling setiap channel 2s)
- **SETTIME=HH:MM:SS**
- **SETDATE=YY:MM:DD**
- **HELP**

## Expected Output

After SETTIME=12:00:00:

```
[DEBUG] Time:12:00 Phase:PEAK LED:RB=90 CW=10 B=100 FS=10 (Actual: RB=81.0 CW=9.0 B=90.0 FS=9.0)
```

Live dashboard will show both values:
```
LED Brightness: RB:90 CW:10 B:100 FS:10
Actual Output (ceiling 90%): RB:81.0 CW:9.0 B:90.0 FS:9.0
```

## Troubleshooting

**Q: Ingin mengubah batas maksimal LED?**  
A: Edit `#define MAX_PERCENT_LIMIT 90` di `config.h` (nilai 0-100)

**Q: Ingin mengubah kecerahan peak?**  
A: Edit `#define PEAK_BRIGHT_RB 90` dll (nilai 0-100, relatif terhadap ceiling)

**Q: LED terlalu terang/redup?**  
A: Periksa nilai `MAX_PERCENT_LIMIT` dan `PEAK_BRIGHT_X`. Actual output = (peak × ceiling) / 100

**Q: RTC shows garbage (42:123)?**  
A: Battery dead or RTC not init. Use SETTIME command.

**Q: LED not turning ON?**  
A: Type TEST to verify hardware directly.

## Examples

**Contoh 1: Conservative Mode (75% ceiling)**
```cpp
#define MAX_PERCENT_LIMIT 75
#define PEAK_BRIGHT_RB 100    // Actual: 75%
#define PEAK_BRIGHT_B 100     // Actual: 75%
```

**Contoh 2: High Performance (95% ceiling)**
```cpp
#define MAX_PERCENT_LIMIT 95
#define PEAK_BRIGHT_RB 90     // Actual: 85.5%
#define PEAK_BRIGHT_B 100     // Actual: 95%
```

**Contoh 3: Safe Testing (50% ceiling)**
```cpp
#define MAX_PERCENT_LIMIT 50
#define PEAK_BRIGHT_RB 100    // Actual: 50%
#define PEAK_BRIGHT_B 100     // Actual: 50%
```

---

**ArsLed V606 - Ceiling Fix & Production Ready**
EOT_README

echo ""
echo "=== v606 Ceiling Fix Generated Successfully ==="
echo ""
echo "Key Changes:"
echo "  - MAX_PERCENT_LIMIT now acts as CEILING"
echo "  - Peak brightness = % of ceiling"
echo "  - Formula: Actual = (peak × ceiling) / 100"
echo "  - Debug shows both values"
echo "  - Dashboard shows actual output"
echo ""
echo "Example with MAX_PERCENT_LIMIT=90:"
echo "  PEAK_BRIGHT_RB=90  → Actual: 81%"
echo "  PEAK_BRIGHT_B=100  → Actual: 90%"
echo "  PEAK_BRIGHT_CW=10  → Actual: 9%"
echo ""
echo "Next steps:"
echo "  1. pio run -t upload"
echo "  2. pio device monitor"
echo "  3. Check debug output for 'Actual:' values"
echo "  4. Verify LED output matches ceiling formula"
echo ""