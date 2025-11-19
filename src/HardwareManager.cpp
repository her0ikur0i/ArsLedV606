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
        uint16_t duty = (MAX_DUTY_VALUE * 50) / 100;
        ledcWrite(i, duty);
        delay(2000);
        ledcWrite(i, 0);
        Serial.println("LED OFF");
        delay(500);
    }
    Serial.println("\n=== ALL CHANNELS TEST (50%) ===");
    Serial.println("All LEDs ON for 3 seconds...");
    for(int i = 0; i < NUM_LED_CHANNELS; i++) {
        uint16_t duty = (MAX_DUTY_VALUE * 50) / 100;
        ledcWrite(i, duty);
    }
    delay(3000);
    for(int i = 0; i < NUM_LED_CHANNELS; i++) {
        ledcWrite(i, 0);
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
    const float MAX_ALLOWED_DUTY = MAX_DUTY_VALUE * ((float)MAX_PERCENT_LIMIT / 100.0f);
    uint16_t duty = (uint16_t)(((float)percent / 100.0f) * MAX_DUTY_VALUE);
    if(duty > (uint16_t)MAX_ALLOWED_DUTY) duty = (uint16_t)MAX_ALLOWED_DUTY;
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
