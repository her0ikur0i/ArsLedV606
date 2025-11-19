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
