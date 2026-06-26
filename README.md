# TCD1304 Driver for STM32F411 (Nucleo-F411RE)

> **Work in Progress **

Firmware to drive the **Toshiba TCD1304DG** 3648-element linear CCD
image sensor using a **Nucleo-F411RE** board, plus a Python host script to
acquire and plot spectra.

---

# TODO

- [ ] Timing has **not been verified on real hardware** — oscilloscope validation of fM, SH, ICG relationships is required before connecting a sensor
- [ ] ADC sampling rate alignment with fM needs empirical tuning (currently assumes 4 fM cycles per pixel)
- [ ] No overflow / saturation detection or warning
- [ ] USB CDC not implemented — uses UART over ST-Link bridge only
- [ ] CMSIS headers must be provided manually (not bundled)
- [ ] Python script has no reconnect logic on serial errors
- [ ] Dark-current subtraction and flat-field correction not implemented

---

## Hardware connections

| TCD1304 pin | Signal | Nucleo pin    | MCU signal         | Notes                               |
|:-----------:|:------:|:-------------:|:------------------:|:------------------------------------|
| 1           | fM     | CN10-13 (PA6) | TIM3_CH1 (AF2)     | 2 MHz master clock                  |
| 2           | ICG    | CN7-30 (PA1)  | TIM2_CH2 (AF1)     | Integration clear gate (active LOW) |
| 3           | SH     | CN7-28 (PA0)  | TIM2_CH1 (AF1)     | Shift gate (active HIGH)            |
| 4           | VDD    | 3.3 V         | —                  | 3.3 V supply                        |
| 5           | GND    | GND           | —                  | Ground                              |
| 6           | OS     | CN7-32 (PA4)  | ADC1_IN4           | Analog output                       |
| 7           | OSCK   | NC            | —                  | (not used in this firmware)         |

Connect a **100 nF decoupling capacitor** between VDD and GND close to the
sensor, and optionally a **10 kΩ pull-up** from OS to 3.3 V.

> **ICG polarity**: The TCD1304 datasheet specifies ICG as an active-LOW
> signal. This firmware inverts TIM2_CH2 via the `CC2P` polarity bit, so
> **no external inverter is required**. If you add a hex inverter (e.g.
> 74HC04), remove `TIM_CCER_CC2P` from `TIM2->CCER` in `tim2_sh_icg_init()`.

---

## Timing summary

```
fM   : 2 MHz PWM on TIM3_CH1
       Period = 50 counts (100 MHz / 2 MHz), duty = 25 (50%)

SH   : TIM2_CH1, prescaler = 49 (timer clock = 2 MHz)
       Pulse width = 4 counts = 2 µs (≥ 1 µs required)
       Period = integration_time_µs × 2 counts (default 10 ms)

ICG  : TIM2_CH2, same timer as SH, active LOW
       Pulse width = 10 counts = 5 µs (≥ SH_width + 1 µs required)
       Period = smallest multiple of SH_period ≥ 7.4 ms readout time

ADC  : ADC1 IN4, DMA2 Stream0, 12-bit
       Converts one pixel every 4 fM cycles → 500 kHz effective rate
       DMA captures PIXEL_COUNT (3694) samples per frame
```

---

## Timer architecture

```
TIM3 (fM, 2 MHz) ──────────────────────────────► PA6 (TCD1304 fM)

TIM2 (prescaled to 2 MHz):
  CH1 (SH, active high) ──────────────────────►  PA0 (TCD1304 SH)
  CH2 (ICG, active low) ──────────────────────►  PA1 (TCD1304 ICG)
     │ CC2 interrupt fires on ICG pulse
     └──── (re-start ADC if gated) ───────────►  PA4 (TCD1304 OS → ADC1_IN4)
                                                       │
DMA2 Stream0 ◄──────────────── ADC1_DR ───────────────┘
     │ Transfer-complete interrupt
     └──── sets capture_done = 1 ──────────────► main loop sends data over UART
```

---

## Build

### Prerequisites

- `arm-none-eabi-gcc` toolchain
- CMSIS headers for STM32F4 (from STM32CubeF4 or standalone CMSIS pack)
- `st-flash` (stlink package) **or** OpenOCD

### Adjust header paths

Edit `Makefile` to point `CMSIS_INC` and `CORE_INC` at your local
STM32CubeF4 installation:

```makefile
CMSIS_INC = ~/STM32CubeF4/Drivers/CMSIS/Device/ST/STM32F4xx/Include
CORE_INC  = ~/STM32CubeF4/Drivers/CMSIS/Include
```

### Compile and flash

```bash
make          # builds tcd1304_f411.bin
make flash    # flashes via st-flash
```

---

## Host Python script

```bash
pip install pyserial numpy matplotlib

# Single acquisition, show plot
python host_read.py --port /dev/ttyACM0 --plot

# Set 50 ms integration time, average 4 frames, save CSV
python host_read.py --port /dev/ttyACM0 --tint 50000 --n 4 --save spectrum.csv

# Windows COM port example
python host_read.py --port COM5 --tint 5000 --plot
```

---

## UART protocol (115200 8N1)

| Command | Payload (host → MCU) | Response (MCU → host) |
|---------|---------------------|-----------------------|
| `'S'`  | — | `0xAA 0x55 LO HI` + 3648 × uint16 LE |
| `'I'`  | 4-byte uint32 LE (µs) | `'K'` |
| `'R'`  | — | 4-byte uint32 LE (µs) |

MCU sends `'K'` once on power-up to signal it is ready.

---

## Integration time

- **Minimum**: 10 µs (hardware limit of TCD1304)
- **Maximum**: ~35 minutes (32-bit timer at 2 MHz)
- **Default**: 10 ms

---

## File structure

```
tcd1304-stm32f411-driver/
├── src/
│   ├── main.c                   ← firmware (all peripherals in one file)
│   └── startup_stm32f411xe.s    ← minimal startup + vector table
├── STM32F411RETx_FLASH.ld       ← linker script (512K Flash, 128K RAM)
├── Makefile
├── host_read.py                 ← Python acquisition script
└── README.md
```

---

## References

- [TCD1304 timing requirements](https://tcd1304.wordpress.com/timing-requirements/)
- [STM32F4 timer setup for TCD1304](https://tcd1304.wordpress.com/timers/)
- Toshiba TCD1304DG datasheet
- RM0383 STM32F411xC/E Reference Manual
