/**
 * @file main.c
 * @brief TCD1304 Linear CCD Driver for STM32F411 (Nucleo-F411RE)
 *
 * =============================================================================
 * PIN MAPPING (Nucleo-F411RE / STM32F411RE)
 * =============================================================================
 *
 *  Signal  | MCU Pin |    Timer    | Alt Func | Connector
 * ---------+----------+-------------+----------+-----------
 *  fM      |  PA6    |  TIM3 CH1   |   AF2    |  CN10-13
 *  SH      |  PA0    |  TIM2 CH1   |   AF1    |  CN7-28
 *  ICG     |  PA1    |  TIM2 CH2   |   AF1    |  CN7-30
 *  OS(ADC) |  PA4    |  ADC1 IN4   |  Analog  |  CN7-32
 *  UART TX |  PA2    |  USART2     |   AF7    |  (ST-Link)
 *  UART RX |  PA3    |  USART2     |   AF7    |  (ST-Link)
 *
 * =============================================================================
 * TIMING OVERVIEW (fM = 2 MHz, SYS_CLK = 100 MHz)
 * =============================================================================
 *
 *  fM  : 2 MHz square wave -> TIM3, period = 50 (100 MHz / 2 MHz), 50% duty
 *  ADC : triggered at 500 kHz (4 fM cycles per pixel)
 *  SH  : min 1000 ns high -> TIM2 CH1, prescaler to 2 MHz
 *  ICG : min 1000 ns > SH high time -> TIM2 CH2
 *
 *  TIM2 prescaler: (100 MHz / 2 MHz) - 1 = 49
 *  SH  pulse width : 4 cycles at 2 MHz = 2 us
 *  ICG pulse width : 10 cycles at 2 MHz = 5 us
 *  SH  period      : configurable, minimum 20 cycles (10 us)
 *  ICG period      : configurable, minimum 14776 cycles (7.4 ms readout)
 *
 *  Total pixels (incl. dummies): 3694
 *  Readout time @ 0.5 MHz      : 3694 / 0.5e6 = 7.388 ms
 *
 * =============================================================================
 * PROTOCOL (UART @ 115200 baud, via ST-Link USB)
 * =============================================================================
 *  'S'           -> single acquisition, send back 3648 uint16
 *  'I' + 4 bytes -> set integration time in us (little-endian)
 *  'R'           -> read integration time back
 */

#include <stdint.h>
#include <stddef.h>
#include "stm32f4xx.h"

/* -----------------------------------------------------------------------------
   Configuration constants
   ----------------------------------------------------------------------------- */
#define SYS_CLK_HZ    100000000UL
#define FM_HZ           2000000UL
#define PIXEL_COUNT        3694U
#define ACTIVE_PIXELS      3648U

#define TIM3_PERIOD   (SYS_CLK_HZ / FM_HZ)          /* 50  */
#define TIM3_DUTY     (TIM3_PERIOD / 2)              /* 25  */
#define TIM2_PRESCALER ((SYS_CLK_HZ / FM_HZ) - 1)   /* 49  */

#define SH_PULSE_WIDTH     4U
#define ICG_PULSE_WIDTH   10U

#define DEFAULT_SH_PERIOD_US  10000UL
#define SH_PERIOD_MIN_US         10UL
#define ICG_READOUT_COUNTS    14776U

#define UART_BAUDRATE   115200UL

/* -----------------------------------------------------------------------------
   Global state
   ----------------------------------------------------------------------------- */
static volatile uint16_t adc_buf[PIXEL_COUNT];
static volatile uint32_t adc_idx      = 0;
static volatile uint8_t  capture_done = 0;
static volatile uint32_t sh_period_us = DEFAULT_SH_PERIOD_US;

/* -----------------------------------------------------------------------------
   Forward declarations
   ----------------------------------------------------------------------------- */
static void clock_init(void);
static void gpio_init(void);
static void tim3_fm_init(void);
static void tim2_sh_icg_init(void);
static void adc_init(void);
static void dma_init(void);
static void usart2_init(void);
static void update_integration_time(uint32_t us);
static void start_acquisition(void);
static void uart_send_byte(uint8_t b);
static void uart_send_buf(const uint8_t *buf, uint32_t len);
static uint8_t uart_recv_byte(void);

/* =============================================================================
   MAIN
   ============================================================================= */
int main(void)
{
    clock_init();
    gpio_init();
    usart2_init();
    tim3_fm_init();
    adc_init();
    dma_init();
    tim2_sh_icg_init();

    uart_send_byte('K');   /* signal host that the MCU is ready */

    while (1) {
        uint8_t cmd = uart_recv_byte();

        switch (cmd) {

        case 'S':
            start_acquisition();
            while (!capture_done) { __WFI(); }   /* sleep until DMA fires */
            capture_done = 0;
            uart_send_byte(0xAA);                /* frame start marker */
            uart_send_byte(0x55);
            uart_send_byte(ACTIVE_PIXELS & 0xFF);
            uart_send_byte((ACTIVE_PIXELS >> 8) & 0xFF);
            uart_send_buf((const uint8_t *)&adc_buf[32], ACTIVE_PIXELS * 2);
            break;

        case 'I':
            {
                uint8_t b[4];
                for (int i = 0; i < 4; i++) b[i] = uart_recv_byte();
                /* Reassemble four bytes into a 32-bit value (little-endian):
                   b[0] is the LSB, b[3] is the MSB.
                   Each byte is cast to uint32_t before shifting to avoid
                   undefined behaviour from shifting an 8-bit value. */
                uint32_t us = (uint32_t)b[0]
                            | ((uint32_t)b[1] <<  8)
                            | ((uint32_t)b[2] << 16)
                            | ((uint32_t)b[3] << 24);
                if (us < SH_PERIOD_MIN_US) us = SH_PERIOD_MIN_US;
                update_integration_time(us);
                uart_send_byte('K');
            }
            break;

        case 'R':
            {
                uint8_t b[4];
                b[0] =  sh_period_us        & 0xFF;
                b[1] = (sh_period_us >>  8) & 0xFF;
                b[2] = (sh_period_us >> 16) & 0xFF;
                b[3] = (sh_period_us >> 24) & 0xFF;
                uart_send_buf(b, 4);
            }
            break;

        default:
            break;
        }
    }
}

/* =============================================================================
   CLOCK - PLL: HSI 16 MHz -> /8 -> x100 -> /2 = 100 MHz
   APB1 /2 = 50 MHz  (timer clock x2 = 100 MHz),  APB2 /1 = 100 MHz
   ============================================================================= */
static void clock_init(void)
{
    /* -- 1. Power interface clock --------------------------------------------
       The PWR peripheral sits on APB1. Its clock must be enabled through the
       RCC before we can write any PWR register.
       RCC_APB1ENR.PWREN (bit 28) -- RM0383 §6.3.11 */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    /* -- 2. Voltage scaling --------------------------------------------------
       The internal voltage regulator has two performance/power modes called
       "voltage scales". Scale 1 (VOS = 0b11, the library constant PWR_CR_VOS)
       allows the CPU to run up to 100 MHz; lower scales cap the maximum
       frequency but reduce current draw.
       We must select Scale 1 before ramping the PLL to 100 MHz.
       PWR_CR.VOS[1:0] (bits 15:14) -- RM0383 §5.4.1 */
    PWR->CR |= PWR_CR_VOS;

    /* -- 3. Flash wait states and prefetch / cache ---------------------------
       Flash read speed is limited by the supply voltage and the number of
       "wait states" (extra CPU cycles inserted per read). At 100 MHz / 3.3 V
       the table in RM0383 §3.4.1 requires 3 wait states.
       PRFTEN enables the prefetch buffer (reads an extra cache line ahead).
       ICEN   enables the instruction cache (64 lines x 128 bits).
       DCEN   enables the data cache (8 lines x 128 bits).
       Together these hide most of the latency introduced by wait states.
       FLASH_ACR register -- RM0383 §3.8.1 */
    FLASH->ACR = FLASH_ACR_LATENCY_3WS   /* 3 wait states for 90-100 MHz  */
               | FLASH_ACR_PRFTEN         /* prefetch buffer enable         */
               | FLASH_ACR_ICEN           /* instruction cache enable       */
               | FLASH_ACR_DCEN;          /* data cache enable              */

    /* -- 4. PLL configuration ------------------------------------------------
       The PLL output frequency is given by (RM0383 §6.3.2):
           fVCO = fHSI  x PLLN / PLLM
           fPLL = fVCO  / PLLP
         with HSI = 16 MHz, PLLM = 8, PLLN = 100, PLLP = 2:
           fVCO = 16 MHz x 100 / 8  = 200 MHz   (must be 100-432 MHz)
           fPLL = 200 MHz / 2       = 100 MHz    OK

       PLLP encoding: 00 -> /2 | 01 -> /4 | 10 -> /6 | 11 -> /8
       So PLLP = 0b00 means divide by 2 -> writing 0U to the field.

       PLLQ = 4 gives fUSB = 200 MHz / 4 = 50 MHz (USB not used here, but
       the field must not be left at 0 which would be an invalid config).

       PLLSRC_HSI selects the internal 16 MHz RC oscillator as PLL source.

       IMPORTANT: the PLL must be configured BEFORE it is enabled (PLLON),
       because PLLCFGR is write-protected once the PLL is running.
       RCC_PLLCFGR register -- RM0383 §6.3.2 */
    RCC->PLLCFGR = (8U   << RCC_PLLCFGR_PLLM_Pos)   /* PLLM=8  -> VCO in  = 2 MHz    */
                 | (100U << RCC_PLLCFGR_PLLN_Pos)    /* PLLN=100-> VCO out = 200 MHz  */
                 | (0U   << RCC_PLLCFGR_PLLP_Pos)    /* PLLP=0 (÷2) -> fPLL = 100 MHz */
                 | (4U   << RCC_PLLCFGR_PLLQ_Pos)    /* PLLQ=4 -> 50 MHz for USB/SDIO */
                 | RCC_PLLCFGR_PLLSRC_HSI;           /* PLL source = HSI 16 MHz       */

    /* -- 5. Enable PLL and wait for lock -------------------------------------
       Set PLLON in RCC_CR, then poll PLLRDY until hardware confirms the PLL
       is locked (phase/frequency detector has settled).
       RCC_CR.PLLON (bit 24) and RCC_CR.PLLRDY (bit 25) -- RM0383 §6.3.1 */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {}   /* spin until PLL is locked */

    /* -- 6. Bus prescalers and system clock switch ---------------------------
       HPRE  (bits 7:4)  : AHB  prescaler -> DIV1 -> HCLK  = 100 MHz
       PPRE1 (bits 12:10): APB1 prescaler -> DIV2 -> PCLK1 =  50 MHz
                           (APB1 max is 50 MHz per RM0383 §6.2)
                           Timer 2/3 clock = 2 x PCLK1 = 100 MHz (§6.2 doubling rule)
       PPRE2 (bits 15:13): APB2 prescaler -> DIV1 -> PCLK2 = 100 MHz

       SW (bits 1:0) = 0b10 switches SYSCLK source to PLL.
       SWS (bits 3:2) is read-only status; we poll it to confirm the switch.
       RCC_CFGR register -- RM0383 §6.3.3 */
    RCC->CFGR = RCC_CFGR_HPRE_DIV1    /* AHB  = SYSCLK / 1 = 100 MHz */
              | RCC_CFGR_PPRE1_DIV2   /* APB1 = HCLK   / 2 =  50 MHz */
              | RCC_CFGR_PPRE2_DIV1;  /* APB2 = HCLK   / 1 = 100 MHz */
    RCC->CFGR |= RCC_CFGR_SW_PLL;     /* select PLL as system clock   */
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}  /* wait for switch */
}

/* =============================================================================
   GPIO
   ============================================================================= */
static void gpio_init(void)
{
    /* -- 1. Enable GPIOA clock -----------------------------------------------
       All GPIO ports sit on the AHB1 bus. Their clocks are individually
       gated through RCC_AHB1ENR. Until this bit is set, any read or write
       to the GPIOA register block produces a bus error.
       RCC_AHB1ENR.GPIOAEN (bit 0) -- RM0383 §6.3.9 */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    /* -- 2. Pin mode (MODER) -------------------------------------------------
       GPIOx_MODER holds two bits per pin (2n+1 : 2n).
       Encoding:  00 = Input | 01 = General-purpose output
                  10 = Alternate function | 11 = Analog
       We first clear the relevant bit-pairs (AND with inverted mask), then
       OR in the desired mode values:
         PA0 -> AF (0b10) for TIM2_CH1  (SH)
         PA1 -> AF (0b10) for TIM2_CH2  (ICG)
         PA2 -> AF (0b10) for USART2_TX
         PA3 -> AF (0b10) for USART2_RX
         PA4 -> Analog (0b11) for ADC1_IN4  (OS signal from TCD1304)
         PA6 -> AF (0b10) for TIM3_CH1  (fM)
       GPIOx_MODER register -- RM0383 §8.4.1 */
    GPIOA->MODER &= ~(  0x3U << (0 * 2)   /* clear PA0 */
                      | 0x3U << (1 * 2)   /* clear PA1 */
                      | 0x3U << (2 * 2)   /* clear PA2 */
                      | 0x3U << (3 * 2)   /* clear PA3 */
                      | 0x3U << (4 * 2)   /* clear PA4 */
                      | 0x3U << (6 * 2)); /* clear PA6 */
    GPIOA->MODER |=  (  0x2U << (0 * 2)   /* PA0 = Alternate function */
                      | 0x2U << (1 * 2)   /* PA1 = Alternate function */
                      | 0x2U << (2 * 2)   /* PA2 = Alternate function */
                      | 0x2U << (3 * 2)   /* PA3 = Alternate function */
                      | 0x3U << (4 * 2)   /* PA4 = Analog             */
                      | 0x2U << (6 * 2)); /* PA6 = Alternate function */

    /* -- 3. Pull-up / pull-down resistors (PUPDR) ----------------------------
       GPIOx_PUPDR: 00 = no pull | 01 = pull-up | 10 = pull-down.
       Alternate-function outputs drive the line actively, so internal
       pull resistors are not needed. Analog pins must have no pull (they
       would distort the ADC input voltage otherwise).
       Clearing all six fields leaves them at 00 = no pull.
       GPIOx_PUPDR register -- RM0383 §8.4.4 */
    GPIOA->PUPDR &= ~(  0x3U << (0 * 2)
                      | 0x3U << (1 * 2)
                      | 0x3U << (2 * 2)
                      | 0x3U << (3 * 2)
                      | 0x3U << (4 * 2)
                      | 0x3U << (6 * 2));

    /* -- 4. Output speed (OSPEEDR) -------------------------------------------
       GPIOx_OSPEEDR: 00 = Low | 01 = Medium | 10 = Fast | 11 = High.
       High speed (0b11) gives the shortest rise/fall times, required for
       2 MHz timer outputs (PA0 SH, PA1 ICG, PA6 fM). UART pins (PA2, PA3)
       and the analog input (PA4) do not need high speed.
       GPIOx_OSPEEDR register -- RM0383 §8.4.3 */
    GPIOA->OSPEEDR |= (  0x3U << (0 * 2)    /* PA0 high speed (SH,  2 MHz) */
                       | 0x3U << (1 * 2)    /* PA1 high speed (ICG, 2 MHz) */
                       | 0x3U << (6 * 2));  /* PA6 high speed (fM,  2 MHz) */

    /* -- 5. Alternate function selection (AFR) --------------------------------
       Each pin has a 4-bit AF field. For pins 0-7 the field lives in
       AFR[0] (AFRL); for pins 8-15 in AFR[1] (AFRH).
       The field for pin N inside AFR[0] starts at bit position (N * 4).
       AF numbers come from the STM32F411 datasheet alternate-function table:
         AF1 (0x1) -> TIM1/TIM2   -> PA0 (TIM2_CH1) and PA1 (TIM2_CH2)
         AF2 (0x2) -> TIM3/TIM4   -> PA6 (TIM3_CH1)
         AF7 (0x7) -> USART1/2/3  -> PA2 (USART2_TX) and PA3 (USART2_RX)
       We clear all relevant nibbles first, then OR in the correct values.
       GPIOx_AFRL register (AFR[0]) -- RM0383 §8.4.9 */
    GPIOA->AFR[0] &= ~(  0xFU << (0 * 4)    /* clear PA0 AF field */
                       | 0xFU << (1 * 4)    /* clear PA1 AF field */
                       | 0xFU << (2 * 4)    /* clear PA2 AF field */
                       | 0xFU << (3 * 4)    /* clear PA3 AF field */
                       | 0xFU << (6 * 4));  /* clear PA6 AF field */
    GPIOA->AFR[0] |=   (  1U  << (0 * 4)    /* PA0 -> AF1 = TIM2_CH1  (SH)  */
                        | 1U  << (1 * 4)    /* PA1 -> AF1 = TIM2_CH2  (ICG) */
                        | 7U  << (2 * 4)    /* PA2 -> AF7 = USART2_TX       */
                        | 7U  << (3 * 4)    /* PA3 -> AF7 = USART2_RX       */
                        | 2U  << (6 * 4));  /* PA6 -> AF2 = TIM3_CH1  (fM)  */
}

/* =============================================================================
   TIM3 - fM 2 MHz on PA6
   ============================================================================= */
static void tim3_fm_init(void)
{
    /* -- 1. Enable TIM3 clock ------------------------------------------------
       TIM3 is on APB1. Enable its bus clock before touching its registers.
       RCC_APB1ENR.TIM3EN (bit 1) -- RM0383 §6.3.11 */
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    /* -- 2. Prescaler (PSC) --------------------------------------------------
       TIM3 receives the APB1 timer clock. Because PPRE1 = /2 (not /1), the
       timer clock is automatically doubled by hardware: 50 MHz x 2 = 100 MHz
       (RM0383 §6.2, timer clock doubling rule).
       PSC = 0 -> timer counts at 100 MHz (no further division needed).
       TIMx_PSC register -- RM0383 §14.4.7 */
    TIM3->PSC = 0;

    /* -- 3. Auto-reload register (ARR) ---------------------------------------
       ARR sets the period. The counter runs 0 -> ARR, then wraps.
       Period = (ARR + 1) / fTIM.
       For 2 MHz: ARR + 1 = 100 MHz / 2 MHz = 50, so ARR = 49.
       TIM3_PERIOD is defined as 50, so ARR = 50 - 1 = 49.
       TIMx_ARR register -- RM0383 §14.4.8 */
    TIM3->ARR = TIM3_PERIOD - 1;          /* ARR = 49 -> 100 MHz / 50 = 2 MHz */

    /* -- 4. Capture/Compare register 1 (CCR1) --------------------------------
       In PWM mode 1 the output is HIGH while counter < CCR1, LOW otherwise.
       TIM3_DUTY = 25 -> HIGH for counts 0-24, LOW for 25-49 -> 50% duty.
       TIMx_CCR1 register -- RM0383 §14.4.13 */
    TIM3->CCR1 = TIM3_DUTY;              /* 25 -> 50% duty at 2 MHz */

    /* -- 5. Capture/Compare mode register 1 (CCMR1) --------------------------
       OC1M[2:0] (bits 6:4): output-compare mode for channel 1.
         Value 6 (0b110) = PWM mode 1: active while counter < CCR1.
       OC1PE (bit 3): enables CCR1 preload register. The new CCR1 value
         written in software only takes effect at the next update event (UEV),
         preventing glitches on the output.
       TIMx_CCMR1 register (output compare mode) -- RM0383 §14.4.9 */
    TIM3->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos)   /* OC1M = 110 -> PWM mode 1  */
                | TIM_CCMR1_OC1PE;              /* OC1PE = 1  -> preload on  */

    /* -- 6. Capture/Compare enable register (CCER) ---------------------------
       CC1E (bit 0) enables the output of channel 1 onto the pin (PA6 via AF2).
       Without this bit set the GPIO is not driven by the timer.
       TIMx_CCER register -- RM0383 §14.4.11 */
    TIM3->CCER = TIM_CCER_CC1E;          /* enable CH1 output on PA6 */

    /* -- 7. Event generation register (EGR) ----------------------------------
       Writing UG (bit 0) forces an update event immediately. This loads the
       ARR and CCR1 preload registers into the active registers so the timer
       starts with the correct period and duty cycle from the very first count.
       TIMx_EGR register -- RM0383 §14.4.6 */
    TIM3->EGR = TIM_EGR_UG;

    /* -- 8. Control register 1 (CR1) - start the timer ----------------------
       ARPE (bit 7): enables ARR preload (mirrors OC1PE behaviour for period).
       CEN  (bit 0): enables the counter. Timer starts; 2 MHz wave on PA6.
       TIMx_CR1 register -- RM0383 §14.4.1 */
    TIM3->CR1 = TIM_CR1_ARPE    /* ARR buffered / preloaded */
              | TIM_CR1_CEN;    /* counter enable            */
}

/* =============================================================================
   TIM2 - SH (CH1, active high) and ICG (CH2, active low)
   ============================================================================= */
static void tim2_sh_icg_init(void)
{
    /* -- 1. Enable TIM2 clock ------------------------------------------------
       TIM2 is on APB1. Same doubling rule: timer clock = 100 MHz.
       RCC_APB1ENR.TIM2EN (bit 0) -- RM0383 §6.3.11 */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    /* -- 2. Prescaler --------------------------------------------------------
       TIM2_PRESCALER = 49 -> timer counts at 100 MHz / 50 = 2 MHz.
       All timing values (SH_PULSE_WIDTH, ICG_PULSE_WIDTH, ICG_READOUT_COUNTS)
       are expressed in units of this 2 MHz clock (0.5 us per tick).
       TIMx_PSC register -- RM0383 §14.4.7 */
    TIM2->PSC = TIM2_PRESCALER;           /* prescaler 49 -> 2 MHz tick */

    /* -- 3. Calculate periods ------------------------------------------------
       sh_period is the integration (exposure) time in 2 MHz ticks:
         sh_period_us x 2 ticks/us = sh_period ticks.
       icg_period must be a multiple of sh_period AND >= ICG_READOUT_COUNTS
       (14776 ticks = 7.388 ms). The loop increments by sh_period until the
       constraint is met, ensuring SH and ICG remain synchronised. */
    uint32_t sh_period  = sh_period_us * 2;
    uint32_t icg_period = sh_period;
    while (icg_period < ICG_READOUT_COUNTS + sh_period)
        icg_period += sh_period;

    /* -- 4. Auto-reload and compare registers --------------------------------
       ARR defines the overall period of TIM2 (= ICG period).
       CCR1: SH pulse width = SH_PULSE_WIDTH = 4 ticks = 2 us.
             TCD1304 requires SH high >= 1 us; 2 us is safe.
       CCR2: ICG pulse width = ICG_PULSE_WIDTH = 10 ticks = 5 us.
             ICG must be high >= SH high + 1 us; 5 us > 3 us. OK.
       TIMx_ARR/CCR1/CCR2 -- RM0383 §14.4.8 / §14.4.13 */
    TIM2->ARR  = icg_period - 1;
    TIM2->CCR1 = SH_PULSE_WIDTH;          /* 4 ticks = 2 us  (SH)  */
    TIM2->CCR2 = ICG_PULSE_WIDTH;         /* 10 ticks = 5 us (ICG) */

    /* -- 5. Capture/Compare mode register 1 (CCMR1) --------------------------
       Both channels configured as PWM mode 1 (OC1M / OC2M = 0b110).
       Preload enabled for both (OC1PE / OC2PE) for glitch-free updates.
       Bits 6:4 control CH1 (SH), bits 14:12 control CH2 (ICG).
       TIMx_CCMR1 register -- RM0383 §14.4.9 */
    TIM2->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE   /* CH1: SH  */
                | (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;  /* CH2: ICG */

    /* -- 6. Capture/Compare enable register (CCER) ---------------------------
       CC1E  (bit 0): enables CH1 output on PA0 (SH, active HIGH).
       CC2E  (bit 4): enables CH2 output on PA1 (ICG).
       CC2P  (bit 5): inverts the polarity of CH2. With PWM mode 1 and
                      CC2P = 1, the pin is LOW while counter < CCR2 and
                      HIGH otherwise. This produces an active-LOW ICG pulse
                      as required by the TCD1304, with no external inverter.
       TIMx_CCER register -- RM0383 §14.4.11 */
    TIM2->CCER = TIM_CCER_CC1E                      /* CH1 enable (SH, active HIGH)  */
               | TIM_CCER_CC2E | TIM_CCER_CC2P;    /* CH2 enable + invert (ICG low) */

    /* -- 7. DMA/interrupt enable register (DIER) -----------------------------
       CC2IE (bit 2): interrupt on CH2 compare event (ICG pulse end).
       Used to trigger ADC/DMA at the correct moment in the TCD1304 cycle.
       TIMx_DIER register -- RM0383 §14.4.4 */
    TIM2->DIER = TIM_DIER_CC2IE;
    NVIC_SetPriority(TIM2_IRQn, 1);
    NVIC_EnableIRQ(TIM2_IRQn);

    /* -- 8. Force update, then start -----------------------------------------
       UG loads all preload registers into active registers immediately.
       TIMx_EGR -- RM0383 §14.4.6 | TIMx_CR1 -- RM0383 §14.4.1 */
    TIM2->EGR = TIM_EGR_UG;
    TIM2->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

/* =============================================================================
   ADC - 12-bit, single channel 4 (PA4), DMA mode
   ============================================================================= */
static void adc_init(void)
{
    /* -- 1. Enable ADC1 clock ------------------------------------------------
       ADC1 is on APB2 (100 MHz). Clock gate in RCC_APB2ENR.
       RCC_APB2ENR.ADC1EN (bit 8) -- RM0383 §6.3.12 */
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    /* -- 2. ADC common control register (ADC_CCR) - prescaler ---------------
       ADC clock = PCLK2 / prescaler. Must not exceed 36 MHz (RM0383 §11.3.2).
       ADCPRE[1:0] (bits 17:16):
         00 = /2 -> 50 MHz (too fast)
         01 = /4 -> 25 MHz  OK   <- chosen here (value 1)
         10 = /6 -> ~17 MHz
         11 = /8 -> 12.5 MHz
       ADC_CCR register -- RM0383 §11.12.15 */
    ADC->CCR = (1U << ADC_CCR_ADCPRE_Pos);   /* ADCPRE=01 -> PCLK2/4 = 25 MHz */

    /* -- 3. ADC control register 1 (ADC_CR1) ---------------------------------
       SCAN (bit 8): enables scan mode. The ADC converts every channel in
       the regular sequence in order and repeats. Required when using DDS
       (continuous DMA) in CR2.
       ADC_CR1 register -- RM0383 §11.12.2 */
    ADC1->CR1 = ADC_CR1_SCAN;               /* scan mode (required with DDS) */

    /* -- 4. ADC control register 2 (ADC_CR2) ---------------------------------
       DMA  (bit 8): enable DMA requests after each conversion. The ADC
                     signals the DMA to copy ADC_DR to adc_buf each time.
       DDS  (bit 9): DMA disable selection - keeps ADC issuing DMA requests
                     continuously. Without DDS the ADC stops after 1 pixel.
       ADON (bit 0): powers on the ADC. A stabilisation delay (tSTAB) is
                     needed before the first conversion; it is covered by
                     the GPIO and timer init that follows.
       ADC_CR2 register -- RM0383 §11.12.3 */
    ADC1->CR2 = ADC_CR2_DMA     /* enable DMA transfer after each conversion */
              | ADC_CR2_DDS     /* keep issuing DMA requests (continuous)     */
              | ADC_CR2_ADON;   /* power on the ADC                           */

    /* -- 5. Sample time register 2 (ADC_SMPR2) - channel 4 ------------------
       Channels 0-9 are in ADC_SMPR2 (channels 10-18 in ADC_SMPR1).
       Each channel occupies 3 bits; channel 4 is at position 4*3 = bit 12.
       Value 0b000 = 3 ADC clock cycles (shortest, fastest sample rate).
       ADC_SMPR2 register -- RM0383 §11.12.5 */
    ADC1->SMPR2 = (0U << (4 * 3));    /* channel 4: 3-cycle sample time */

    /* -- 6. Regular sequence registers (ADC_SQR1 / ADC_SQR3) ----------------
       SQR1.L[3:0] (bits 23:20): sequence length. L=0 means 1 conversion.
       SQR3.SQ1[4:0] (bits 4:0): first (only) channel = 4 (PA4).
       ADC_SQR1 -- RM0383 §11.12.9 | ADC_SQR3 -- RM0383 §11.12.11 */
    ADC1->SQR1 = 0;                   /* L = 0 -> 1 conversion in sequence */
    ADC1->SQR3 = 4U;                  /* SQ1 = channel 4 (PA4)             */
}

/* =============================================================================
   DMA - DMA2 Stream0 Channel0 -> ADC1 -> adc_buf
   ============================================================================= */
static void dma_init(void)
{
    /* -- 1. Enable DMA2 clock ------------------------------------------------
       ADC1 is hardwired to DMA2 (not DMA1) - fixed in silicon.
       See RM0383 Table 28 (DMA2 request mapping).
       RCC_AHB1ENR.DMA2EN (bit 22) -- RM0383 §6.3.9 */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    /* -- 2. Disable stream before configuration ------------------------------
       The EN bit must be 0 before writing any stream register. Writing CR
       to 0 clears EN and all config bits. Poll EN until the stream actually
       stops (it may be finishing a transfer in hardware).
       DMA_SxCR.EN (bit 0) -- RM0383 §9.5.5 */
    DMA2_Stream0->CR = 0;
    while (DMA2_Stream0->CR & DMA_SxCR_EN) {}   /* wait for disable */

    /* -- 3. Stream configuration register (DMA_SxCR) -------------------------
       CHSEL[2:0] (bits 27:25): channel 0 = ADC1 request (RM0383 Table 28).
       MSIZE[1:0] (bits 14:13): memory data width = 1 (16-bit halfword),
                                matching uint16_t adc_buf elements.
       PSIZE[1:0] (bits 12:11): peripheral data width = 1 (16-bit),
                                matching ADC_DR (12-bit result in 16-bit field).
       MINC (bit 10): memory increment mode. The memory address advances by
                      MSIZE after each transfer, filling adc_buf sequentially.
                      The peripheral address (ADC_DR) is fixed (no PINC).
       TCIE (bit 4): transfer-complete interrupt. Fires when NDTR reaches 0
                     (all PIXEL_COUNT samples transferred). The ISR sets
                     capture_done to wake the main loop.
       DMA_SxCR register -- RM0383 §9.5.5 */
    DMA2_Stream0->CR = (0U << DMA_SxCR_CHSEL_Pos)   /* channel 0 -> ADC1   */
                     | (1U << DMA_SxCR_MSIZE_Pos)   /* mem  width = 16-bit */
                     | (1U << DMA_SxCR_PSIZE_Pos)   /* periph width= 16-bit*/
                     | DMA_SxCR_MINC                /* increment mem ptr   */
                     | DMA_SxCR_TCIE;               /* TC interrupt enable */

    /* -- 4. Peripheral and memory addresses ----------------------------------
       PAR  = address of ADC1_DR (holds the conversion result).
       M0AR = start address of adc_buf[] in SRAM.
       DMA_SxPAR register  -- RM0383 §9.5.7
       DMA_SxM0AR register -- RM0383 §9.5.8 */
    DMA2_Stream0->PAR  = (uint32_t)&ADC1->DR;     /* source: ADC data register */
    DMA2_Stream0->M0AR = (uint32_t)adc_buf;       /* dest:   pixel buffer      */

    /* -- 5. Number of data items (NDTR) --------------------------------------
       The stream transfers exactly PIXEL_COUNT halfwords (3694), then fires
       TCIE. Reloaded in start_acquisition() for each new frame.
       DMA_SxNDTR register -- RM0383 §9.5.6 */
    DMA2_Stream0->NDTR = PIXEL_COUNT;

    NVIC_SetPriority(DMA2_Stream0_IRQn, 2);
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/* =============================================================================
   USART2 - 115200 baud (APB1 clock = 50 MHz)
   ============================================================================= */
static void usart2_init(void)
{
    /* -- 1. Enable USART2 clock ----------------------------------------------
       USART2 is on APB1 (50 MHz).
       RCC_APB1ENR.USART2EN (bit 17) -- RM0383 §6.3.11 */
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* -- 2. Baud rate register (USART_BRR) -----------------------------------
       BRR = fPCLK / baud  (oversampling by 16, the default after reset).
       fPCLK1 = SYS_CLK_HZ / 2 = 50 MHz.
       BRR = 50 000 000 / 115 200 = 434.
       The STM32 BRR format encodes mantissa in bits 15:4 and a 4-bit
       fractional part in bits 3:0. Integer division of fPCLK/baud yields
       the correct combined value when the result fits in 16 bits.
       USART_BRR register -- RM0383 §19.6.3 */
    USART2->BRR = (uint32_t)(SYS_CLK_HZ / 2 / UART_BAUDRATE);  /* = 434 */

    /* -- 3. Control register 1 (USART_CR1) -----------------------------------
       TE  (bit 3): transmitter enable - activates TX pin (PA2).
       RE  (bit 2): receiver enable    - activates RX pin (PA3).
       UE  (bit 13): USART enable - starts the peripheral.
       USART_CR1 register -- RM0383 §19.6.4 */
    USART2->CR1 = USART_CR1_TE    /* TX enable  */
                | USART_CR1_RE    /* RX enable  */
                | USART_CR1_UE;  /* USART enable */
}

/* -----------------------------------------------------------------------------
   UART helpers
   ----------------------------------------------------------------------------- */
static void uart_send_byte(uint8_t b)
{
    /* TXE (bit 7) in USART_SR is set when the TX data register is empty
       and ready for a new byte.
       USART_SR register -- RM0383 §19.6.1 */
    while (!(USART2->SR & USART_SR_TXE)) {}
    USART2->DR = b;
}

static void uart_send_buf(const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) uart_send_byte(buf[i]);
    /* TC (bit 6): transmission complete. Both shift register and DR are
       empty; safe to disable TX without losing the last byte.
       USART_SR register -- RM0383 §19.6.1 */
    while (!(USART2->SR & USART_SR_TC)) {}
}

static uint8_t uart_recv_byte(void)
{
    /* RXNE (bit 5): receive register not empty - a byte has arrived.
       Blocks until data is available.
       USART_SR register -- RM0383 §19.6.1 */
    while (!(USART2->SR & USART_SR_RXNE)) {}
    return (uint8_t)USART2->DR;
}

/* =============================================================================
   update_integration_time
   ============================================================================= */
static void update_integration_time(uint32_t us)
{
    if (us < SH_PERIOD_MIN_US) us = SH_PERIOD_MIN_US;
    sh_period_us = us;

    /* Recalculate periods using the same logic as tim2_sh_icg_init().
       Writing ARR, CCR1, CCR2 while the timer is running is safe because
       the preload bits (ARPE, OC1PE, OC2PE) are set: new values become
       active only after the next update event (counter overflow), so the
       waveform changes cleanly between frames without glitches. */
    uint32_t sh_period  = us * 2;
    uint32_t icg_period = sh_period;
    while (icg_period < ICG_READOUT_COUNTS + sh_period)
        icg_period += sh_period;

    TIM2->ARR  = icg_period - 1;
    TIM2->CCR1 = SH_PULSE_WIDTH;
    TIM2->CCR2 = ICG_PULSE_WIDTH;
}

/* =============================================================================
   start_acquisition - arms DMA + ADC for one frame
   ============================================================================= */
static void start_acquisition(void)
{
    capture_done = 0;
    adc_idx      = 0;

    /* Disable the DMA stream, clear all its interrupt flags, reload NDTR,
       reset the destination pointer, then re-enable.
       NDTR must be written while the stream is disabled (EN = 0).

       DMA2->LIFCR = 0x3FU clears all six stream-0 status flags in one write:
         bit 5 = CTCIF0  (clear transfer complete)
         bit 4 = CHTIF0  (clear half transfer)
         bit 3 = CTEIF0  (clear transfer error)
         bit 2 = CDMEIF0 (clear direct mode error)
         bit 1 = reserved (write 0; written as part of 0x3F but harmless)
         bit 0 = CFEIF0  (clear FIFO error)
       Writing 1 to a clear bit clears the corresponding flag in DMA_LISR.
       This ensures no stale flags from a previous acquisition can
       spuriously trigger the TC interrupt or block the stream from starting.
       DMA_SxCR.EN -- RM0383 §9.5.5
       DMA_LIFCR   -- RM0383 §9.5.3 */
    DMA2_Stream0->CR  &= ~DMA_SxCR_EN;
    while (DMA2_Stream0->CR & DMA_SxCR_EN) {}
    DMA2->LIFCR        = 0x3FU;          /* clear all stream-0 flags (§9.5.3) */
    DMA2_Stream0->NDTR = PIXEL_COUNT;
    DMA2_Stream0->M0AR = (uint32_t)adc_buf;
    DMA2_Stream0->CR  |= DMA_SxCR_EN;   /* re-enable stream */

    /* Start continuous ADC conversions.
       CONT (bit 1): keeps the ADC converting without interruption until cleared.
       SWSTART (bit 30): triggers the first conversion; subsequent ones follow
                         automatically in continuous mode.
       ADC_CR2.CONT and ADC_CR2.SWSTART -- RM0383 §11.12.3 */
    ADC1->CR2 |= ADC_CR2_CONT | ADC_CR2_SWSTART;
}

/* =============================================================================
   INTERRUPT HANDLERS
   ============================================================================= */

/* TIM2 CH2 compare match - fires when the ICG pulse ends (ICG returns HIGH).
   On Cortex-M, timer flags are cleared by writing 0 to the target bit.
   The recommended idiom is to write the bitwise complement of the flag mask:
   all bits stay 1 (writing 1 to a flag is a no-op), except CC2IF which
   is set to 0 and therefore cleared.
   TIMx_SR.CC2IF (bit 2) -- RM0383 §14.4.5 */
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_CC2IF) {
        TIM2->SR = ~TIM_SR_CC2IF;   /* clear CC2IF (write 0; 1s elsewhere are no-ops) */
    }
}

/* DMA2 Stream0 transfer complete - all PIXEL_COUNT samples moved to adc_buf.
   Stop ADC and signal the main loop.
   TCIF0  (bit 5 in DMA_LISR):  transfer-complete flag for stream 0.
   CTCIF0 (bit 5 in DMA_LIFCR): write 1 to clear TCIF0.
   DMA_LISR.TCIF0   -- RM0383 §9.5.1
   DMA_LIFCR.CTCIF0 -- RM0383 §9.5.3 */
void DMA2_Stream0_IRQHandler(void)
{
    if (DMA2->LISR & DMA_LISR_TCIF0) {
        DMA2->LIFCR = DMA_LIFCR_CTCIF0;              /* clear TC flag             */
        ADC1->CR2  &= ~(ADC_CR2_CONT | ADC_CR2_SWSTART); /* stop ADC conversions  */
        DMA2_Stream0->CR &= ~DMA_SxCR_EN;            /* disable DMA stream        */
        capture_done = 1;                             /* wake main loop            */
    }
}
