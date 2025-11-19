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
