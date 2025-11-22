
---

# **EV Dashboard 12V (STM32F4, V32 – 16MHz HSI Corrected)**

# 🚗 EV Dashboard – STM32F401RE

A complete **Electric Vehicle Dashboard** implementation on STM32F401, supporting:
- Battery voltage monitoring  
- Current sensing using ACS712  
- RPM + Speed using hall sensor  
- Odometer  
- 16×2 LCD (I²C, PCF8574)  
- Button-based page switching  
- Low-pass filtered data for stable display  

This version is fully corrected and optimized for the **internal 16MHz HSI clock**.

---

# 📐 System Architecture

```
+--------------------------------------------+
|               STM32F401RE                  |
|                                            |
|  ADC1_CH0 <-- PA0 : Battery Voltage        |
|  ADC1_CH1 <-- PA1 : ACS712 Current         |
|  EXTI6    <-- PA6 : Hall Speed Sensor      |
|  PC13     <-- Button                       |
|                                            |
|I²C1 (PB8=SCL, PB9=SDA) --> PCF8574 --> LCD |
+--------------------------------------------+
```

---

# 🧰 Hardware Used

### ✔️ Microcontroller
- STM32F401 Nucleo-64

### ✔️ Sensors
- Voltage divider (scaled ×5 for 12V)
- ACS712 (5A hall current sensor)
- Hall-effect speed sensor with **32-tooth encoder**

### ✔️ Display
- 16×2 LCD (HD44780)
- PCF8574 I²C backpack (address `0x27`)

### ✔️ Button
- User Button (PC13) for display page switching

---

# 🚦 Features

- 🔋 Battery voltage, percentage, and status  
- ⚡ Real-time current measurement with filtering  
- 🚀 RPM and speed (km/h) calculation  
- 📏 Odometer (meters + km)  
- 🖥️ LCD display with 4 pages  
- 🔘 Button page switching  
- 🎚️ LPF filtering for smooth UI  
- 🕒 Accurate timing using TIM2 @16MHz  

---

# 🕒 Timing System

The entire dashboard timing uses **TIM2** configured for a precise **1ms tick** using the 16 MHz HSI clock.

```

PSC = 1600 - 1   → 16MHz / 1600 = 10kHz
ARR = 10 - 1     → 10 counts = 1ms

```

## Timing Intervals
| Task | Interval |
|------|----------|
| Main logic loop | 25 ms (40 Hz) |
| RPM calculation | 100 ms (10 Hz) |
| LCD refresh | 250 ms (4 Hz) |
| EXTI debounce | 1 ms |

---

# 📦 Peripheral Breakdown

## 1️⃣ GPIO
| Pin | Mode | Function |
|-----|------|----------|
| PA0 | Analog | Battery Voltage |
| PA1 | Analog | ACS712 Current |
| PA6 | Input + EXTI | Speed sensor |
| PB8 | AF4 (Open-Drain) | I²C SCL |
| PB9 | AF4 (Open-Drain) | I²C SDA |
| PC13 | Input | Button |

---

## 2️⃣ ADC (Analog to Digital)
- 12-bit resolution  
- ADC Clock = 8 MHz (16MHz ÷ 2)  
- Long sample time (`SMPR2 = 7`)  
- Voltage oversampled: 16 readings  
- Current oversampled: 32 readings  
- ACS712 zero-offset calibrated (100 samples)

---

## 3️⃣ Timer (TIM2 – 1ms Tick)
- PSC = **1599**  
- ARR = **9**  
- Generates **1ms interrupt**  
- Drives all timing (RPM window, display refresh, debounce)

---

## 4️⃣ I²C1 (LCD Communication)
- PB8 = SCL  
- PB9 = SDA  
- Standard Mode (100 kHz)  
- PCF8574 I/O expander  
- LCD uses 4-bit mode over I²C  

---

## 5️⃣ EXTI (Speed Sensor on PA6)
- Rising-edge trigger  
- Pull-down enabled  
- 1 ms debounce inside ISR  
- `pulse_count++` increments per tooth  
- 32 teeth → 1 full revolution  

---

# 📡 Sensor Algorithms

## 🔹 Voltage (mV)
```

16-sample average
scaled_mv = adc_raw_to_mv(raw_avg) * V_SCALE_FACTOR

```

## 🔹 Current (mA)
```

32-sample average
current_mv = avg_mv - acs_zero_mv
filtered & deadbanded
current = (current_mv * 1000) / 185

```

## 🔹 RPM
```

RPM = (pulses * 60000) / (PULSES_PER_REV * window_ms)

```

## 🔹 Speed (km/h)
```

speed = (wheel_rpm * circumference_mm * 60) / 1,000,000

```

## 🔹 Odometer
```

dist_mm = pulses * CIRCUMFERENCE_MM / PULSES_PER_REV

```

---

# 🖥️ LCD Pages

### **Page 0**
```

V:12.45V
Odo:128m

```

### **Page 1**
```

RPM: 180
Spd: 4 km/h

```

### **Page 2**
```

## Curr: 1.24A

```

### **Page 3**
```

Dist: 0 km
125 m

```

---

# 🧪 Calibration Routine

At startup:
- LCD shows **CALIBRATING…**
- 100 samples of ACS712 are taken
- Zero-offset stored as `acs_zero`

This improves accuracy and removes sensor drift.

---

# ▶️ How to Run

1. Flash code into STM32F401  
2. Connect sensors  
3. Power board (USB or external 5V)  
4. Wait for calibration  
5. Dashboard becomes active  
6. Press PC13 to change display pages  

---

# 🔧 Build

Supports:
- STM32CubeIDE  
- Keil uVision  
- ARM GCC (Makefile)  
- PlatformIO  

---

# 📄 License
**MIT License**  
Free to use, modify, and distribute.

---

# 👨‍💻 Author
**K Dinesh, Nithin S** – Embedded Systems Developer  
EV Dashboard Project – Final Version V32

---
