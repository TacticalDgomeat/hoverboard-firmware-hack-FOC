# Hoverboard Driver FOC Motor Control with Encoder and Brake Resistor Support

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

This is a fork of the [hoverboard-firmware-hack-FOC](https://github.com/EFeru/hoverboard-firmware-hack-FOC) project that adds AB encoder support and brake resistor functionality for precision motor control applications. 

For general setup instructions, hardware information, and base firmware features, please refer to the [original repository](https://github.com/EFeru/hoverboard-firmware-hack-FOC).

---

## Key Improvements Over Original Repository

1. **Smooth Torque Output at Zero RPM**

Encoder feedback enables true zero-speed torque control with no cogging or vibration. Perfect for robotics applications requiring precise positioning and smooth low-speed operation.

2. **PSU Support**

Integrated brake resistor support allows safe operation from bench power supplies. The brake resistor dissipates regenerative energy that would otherwise cause voltage spikes and damage PSUs or trip overvoltage protection.

3. **High-Quality Input Control Performance**

Enhanced PWM input processing (both hardware and software implementations) provides:
- low latency 1000hz polling
- Noise-free
- High Resolution

For information on all improvments see [Pull Request #3](https://github.com/SiMachines/hoverboard-firmware-hack-FOC/pull/3#issue-3584417632).

---

## New Configurations

- **ONE_AXIS_VARIANT**: Single motor control with AB encoder, hardware pwm input and internal left driver brake resistor support
- **TWO_AXIS_VARIANT**: Dual motor control with AB encoders, software pwm input and external brake resistor support
---

## Quick Start Guide

### 1. Select Your Variant

In `platformio.ini`, uncomment your desired configuration:

```ini
default_envs = ONE_AXIS_VARIANT    ; Single motor with encoder
;default_envs = TWO_AXIS_VARIANT   ; Dual motors with encoders
```

### 2. Essential Configuration

Edit `Inc/config.h` with your specific parameters:

#### Battery Configuration
```c
#define BAT_CELLS               10      // Your battery cell count:
                                        // 6s  = 24V (22.2V - 25.2V)
                                        // 10s = 36V (37V - 42V)
                                        // 13s = 48V (48.1V - 54.6V)
```

#### Motor Configuration
```c
#define N_POLE_PAIRS            15      // Standard hoverboard motors: 15
                                        // Check your motor specs if different
```
---

## ONE_AXIS_VARIANT Setup

Single motor control with encoder feedback - default seetings setup for FFB WheelBase, change if your project is different

#### MCU Selection
```c
#define GD32F103Rx              1       // Uncomment if using GD32 MCU (108MHz)
                                        // Comment out for STM32F103 (72MHz)
```

### Encoder Configuration

```c
#define ENCODER_X
#define ENCODER_X_PPR              2048     // Your encoder pulses per revolution
#define ALIGNMENT_X_POWER          6553     // Sensor alignment voltage out of 16000 for HW & SW-PWM, out of 1000 for the others
```

### Motor Enable/Disable

```c
#undef MOTOR_LEFT_ENA                       // Disable left motor driver
#define MOTOR_RIGHT_ENA                     // Enable right motor only
```

### Current and Speed Limits

```c
#define I_MOT_MAX                  15       // [A] Maximum motor current
#define I_DC_MAX                   17       // [A] DC link current limit, (Above this value, current chopping is applied. To avoid this make sure that I_DC_MAX = I_MOT_MAX + 2A)  
#define N_MOT_MAX                  2000     // [rpm] Maximum motor speed
```

**Safety Tip**: Start with conservative limits (e.g., `I_MOT_MAX = 5A`) for initial testing, then increase gradually.

### Brake Resistor Setup

Choose **ONE** of the following options:

#### Option A: Internal Brake Resistor
Uses the left motor driver as a brake resistor (left motor must be disabled).

```c
#define INTBRK_L_EN                         // Enable internal brake on left driver
```

#### Option B: External Brake Resistor
Uses an external brake resistor connected positive psu supply in series with a mosfet/gate driver conntected to ground, can be a bts7960 for example if using with under 24v supply.

```c
#define EXTBRK_EN                           // Enable external brake resistor
#define EXTBRK_USE_CH3                      // Use channel 3 for brake control
// OR
//#define EXTBRK_USE_CH4                    // Use channel 4 for brake control
```

#### Brake Resistor Parameters

```c
#define BRAKE_RESISTANCE           300      // [Ohm × 100] Your resistor value
                                            // Example: 3.0Ω = 300, 2.2Ω = 220, enter slightly higher resistance then measured for example 3.0Ω instead of 2.2Ω to ensure adequate braking

#define BRKRESACT_SENS            40/20     // [A] Activation threshold (40mA typical), increase in 20mA steps if braking resistor gets warm while wheel is stationary

#define MAX_REGEN_CURRENT         0/20      // [A] Maximum regen current in 20mA steps
                                            // PSU: Set to 0/20 (0mA) 
                                            // Battery: Set how much regenative braking you want to allow 20/20 (20ma), 40/20(40ma) etc.
```

**⚠️ Critical Notes:**
- **PSU users MUST set `MAX_REGEN_CURRENT = 0/20`** to prevent voltage spikes
- If `BRKRESACT_SENS` is set too high it may cause over voltage in psu
- Add 10-20% safety margin to `BRAKE_RESISTANCE` value

### Control Input Selection

Choose your preferred input method:

```c
// Hardware PWM (best performance)
#define HW_PWM                     0        // Priority 0 (highest), HW pwm outputs to Right Motor
```

### Input Range Configuration

```c
#define PRI_INPUT1                 0, -16000, 0, 16000, 0  //Left Motor disabled
#define PRI_INPUT2                 2, -16000, 0, 16000, 0  //Right Motor Output
// Format: TYPE, MIN, MID, MAX, DEADBAND
//   TYPE: 0=Disabled, 1=Normal Pot, 2=Middle Resting, 3=Auto-detect
//   MIN/MAX: Input signal range
//   MID: Center position for middle-resting inputs
//   DEADBAND: Center deadzone width
```

### Motor Direction

```c
//#define INVERT_R_DIRECTION                // Uncomment to reverse motor direction
```
---

## TWO_AXIS_VARIANT Setup

Dual motor control with encoders on both motors - default setup is for ffb joystick or seat belt puller using Software pwm inputs on right side uart port and external brake resistor on PA2 left uart port

**Same setup as ONE_AXIS_VARIANT with a few differences:** 
- Both Encoders enabled (ofcourse to achive two axis you need two encoders)
- Both Motors enabled (ofcourse to achive two axis you need two motors)
- External brake resistor (left driver is used by left motor now)
- Software pwm(hardware pwm pin is used by encoder_y)
---

## Common Configuration (Both Variants)

### Control Type and Mode

```c
#define CTRL_TYP_SEL               FOC_CTRL // Field Oriented Control (recommended)
#define CTRL_MOD_REQ               TRQ_MODE // Torque mode (best for encoders)
```

**Control Modes:**
- `TRQ_MODE`: Torque control with freewheeling (recommended)
- `SPD_MODE`: Speed control with active velocity regulation
- `VLT_MODE`: Voltage control (fastest response, no feedback)

### Additional Features

```c
#define DIAG_ENA                   0        // Disable diagnostics (allows stall)
#define INACTIVITY_TIMEOUT         100      // Power-off timeout (minutes)
#define HOCP                                // BKIN Tim 1/8 Hardware over-current protection
#define BEEPER_OFF                          // Use led as buzzer
#define DC_LINK_WATCHDOG_ENABLE             // Enable voltage and current monitoring watchdog on vbus (turns off driver in case of large voltage spike above/under vbus and large negative or positive current flow spikes)
#define FIELD_WEAK_ENA             0        // Field weakening, set to 0 for disable
#define RATE                       32767    // Acceleration rate (32767 = fastest) Lower `RATE` values make acceleration smoother but slower. 
#define FILTER                     65535    // Input filtering (65535 = minimal) Lower `FILTER` values increase filtering (smoother but more lag).
```
---

### Flash the Firmware

- Discoconnect psu from driver
- Unlock and erase MCU if first time flashing see [Instructions](https://github.com/EFeru/hoverboard-firmware-hack-FOC/wiki/How-to-Unlock-MCU-Flash)
- Create fork and modify the Battery Configuration at least for your supply voltage. 
- Github will build, click Actions, latest workflow run and when its done building you will be able to download a zip which contains a bin file, flash that with stlink utility
- Alternatively open project in vscode, change what you need and upload from there
---

## Wiring Guide
For the reverse-engineered schematics of the mainboard, see [Schematic](/docs/20150722_hoverboard_sch.pdf)
![mainboard_pinout](/docs/pictures/mainboard_pinout.png)

The original Hardware supports two 4-pin cables that originally were connected to the two sideboards. They break out GND, 12/15V and USART2&3 of the Hoverboard mainboard. Both USART2&3 support UART, PWM, PPM, and iBUS input. Additionally, the USART2 can be used as 12bit ADC, while USART3 can be used for I2C. Note that while USART3 (right sideboard cable) is 5V tolerant, USART2 (left sideboard cable) is **not** 5V tolerant.

Typically, the mainboard brain is an [STM32F103RCT6](/docs/literature/[10]_STM32F103xC_datasheet.pdf), however some mainboards feature a [GD32F103RCT6](/docs/literature/[11]_GD32F103xx-Datasheet-Rev-2.7.pdf) which is also supported by this firmware.

### Encoder Connections

**For ONE_AXIS_VARIANT (Right motor - ENCODER_X):**
- Connect encoder channels A and B to the left HALL port HALL B(PB6) and HALL A(PB7) 
- Ensure proper encoder power supply, connect to GND pin and 5V pin on same port or 3.3v pin on stlink port

**For TWO_AXIS_VARIANT:**
- ENCODER_X: Right motor encoder → Same as one axis
- ENCODER_Y: Left motor encoder → HALL A(PB5) and PB4( PB4 is exposed as a pin on the micro controller, you have to use a high awg wire to connect to it and glue the wire to the board to prevent it breaking off. Leave a bit of excces and wrap it around something on the board for mechanichal anchoring)(PB5 and PB4 are not 5V tolerant but I have set them as floating digital inputs so it shouldnt be an issue with 5v encoder)

### Brake Resistor Connections
- Use high-wattage resistor (minimum 5W, usually recommended 50w for typical applications but I havent face any issues yet with just 5W for ffb wheelbase)

**Internal Brake (`INTBRK_L_EN`):**
- Connect brake resistor between motor driver phase A and GND
- CAUTION Left motor must be disabled and INTBRK_L_EN MUST be enabled otherwise you will cook the brake resistor and could get hot enough to start fire and burn if touched

**External Brake (`EXTBRK_EN`):**
- USE Left Sideboard(uart) port, ouputs 3.3v on PA2 as control pin
- Needs additional mosfet and gate capable which supports your psu voltage with 1.5x safety factor, you an use the 15v pin for the gate driver
- Connect one side of the resistor to mosfet, and other side to V+ of psu. connect other side of the mosfet to psu ground. 
- Alternate use a integrated driver like bts7960 if it supports your psu voltage for example 24v

### Control Input Connections

**Hardware PWM (`HW_PWM`):**
- Left Hall Port HALL A (PB5) (Non 5v tollerant, I made input floating and had no issues with using a arduino pro micro to drive the pin at 5V)

**Software PWM (`SW_PWM_RIGHT`):**
- Right sideboard port pins PB10 and PB11 (5V tolerant)
- Left sideboard port pins PA2 and PA3 (Not 5V tolerant)
---

## Troubleshooting

### Driver not turning on
- Turn on button doesnt turn on driver if supply under 24v
- I use something metalic like a allan key and jiggle it between the pins a bit when using 19.5v supply (Do not use under 19v supply)

### Driver turning off right away
- Supply voltage under 18v or battery voltage not set (check quick start instructions)
- Supply voltage above 50v max
- No firmware flashed
- Shorted Mosfet

### Motor not turning or cogging
- Check ENCODER PPR is set correctly in setup 
 
### Motor not turning under its own power and making a electrical noise when turned
- Triggered Voltage or current protection limits
- 
---

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.

---

## License

This project inherits the GPLv3 license from the original [hoverboard-firmware-hack-FOC](https://github.com/EFeru/hoverboard-firmware-hack-FOC) repository.

---

## Support

For issues specific to encoder and brake resistor functionality, please open an issue in this repository.

For general hoverboard firmware questions, refer to the [original repository](https://github.com/EFeru/hoverboard-firmware-hack-FOC) and its wiki.

---

- Original FOC firmware: [EFeru/hoverboard-firmware-hack-FOC](https://github.com/EFeru/hoverboard-firmware-hack-FOC)

---

## Recent Changes (This Fork)

- Added AB quadrature encoder support for precise motor control
- Implemented internal and external brake resistor functionality
- Enhanced PWM input processing (hardware and software)
- Added ADC watchdog handling
- Performance, safety and reliability improvements

For more changes, see [Pull Request #3](https://github.com/SiMachines/hoverboard-firmware-hack-FOC/pull/3#issue-3584417632).
