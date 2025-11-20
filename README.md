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
