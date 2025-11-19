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
Serial.printf("[DEBUG] Time:%02d:%02d Phase:%s LED:RB=%d CW=%d B=%d FS=%d\n", now.hour(), now.minute(), phaseStr[phase], currentBrightness[CH_RB], currentBrightness[CH_CW], currentBrightness[CH_B], currentBrightness[CH_FS]);
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
