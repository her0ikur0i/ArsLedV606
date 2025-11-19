ArsLed Marine Light V606 - RTC Format Fixed Edition
Key Features

v602gpt.sh RTC handling (proven working format)
Correct pin mapping (GPIO 4,5,6,7 for LEDs)
Lower PWM frequency (5kHz for compatibility)
Hardware test command (type TEST)
Debug output every 10s
ThingPulse SSD1306Wire driver

Critical Fix
RTC time format now matches v602gpt.sh:

Uses DateTime object correctly
No garbage values (42:123 fixed)
Format: HH:MM:SS DD/MM/YYYY

Pin Configuration
LED Channels:

RB (Royal Blue): GPIO 4
CW (Cool White): GPIO 5
B (Blue): GPIO 6
FS (Full Spec): GPIO 7

Peripherals:

Onboard LED: GPIO 15
I2C SDA: GPIO 1
I2C SCL: GPIO 2
Fan: GPIO 3

Temperature Sensors:

Water: GPIO 8
Heatsink: GPIO 9
Room: GPIO 10

Quick Start
chmod +x v606.sh
./v606.sh
pio run -t upload
pio device monitor
Commands

TEST: Hardware test (50% each channel 2s)
SETTIME=HH:MM:SS
SETDATE=YY:MM:DD
HELP

Expected Output
After SETTIME=12:00:00:
[DEBUG] Time:12:00 Phase:PEAK LED:RB=70 CW=30 B=100 FS=20
If still showing Phase:OFF at 12:00:

Check RTC battery (CR2032)
Verify RTC module wiring
Type TEST to bypass scheduler

Troubleshooting
Q: RTC shows garbage (42:123)?
A: Battery dead or RTC not init. Use SETTIME command.
Q: LED not turning ON?
A: Type TEST to verify hardware directly.
Q: Phase shows OFF at noon?
A: RTC time not set correctly. Verify with serial output.

ArsLed V606 - RTC Format Fixed & Production Ready
