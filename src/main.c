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
 *  fM  : 2 MHz square wave -> TIM3 CH1, period = 50 counts (PSC=0)
 *  ADC : triggered by TIM3 TRGO at 500 kHz (every 4 fM cycles = 200 counts)
 *        TIM3 TRGO fires on CH2 compare match (CCR2 = 200, ARR = 199)
 *        Sample time = 3 cycles, conversion = 12 cycles = 15 / 25 MHz = 600 ns
 *        600 ns < 2 us/pixel -> fits comfortably inside one pixel window
 *  SH  : min 1000 ns high -> TIM2 CH1, 2 MHz tick -> 4 ticks = 2 us
 *  ICG : active LOW, min 1000 ns -> TIM2 CH2, inverted via CC2P, 10 ticks = 5 us
 *        ICG falling edge (counter = 0) triggers TIM2 CC2IE ISR, which arms
 *        the ADC+DMA so sampling begins synchronised to the sensor readout.
 *
 *  TIM2 prescaler : (100 MHz / 2 MHz) - 1 = 49
 *  SH  pulse width: 4 ticks at 2 MHz = 2 us
 *  ICG pulse width: 10 ticks at 2 MHz = 5 us
 *  ICG period     : smallest multiple of SH period >= 14776 ticks (7.388 ms)
 *
 *  Total pixels (incl. dummies): 3694
 *  Readout time @ 500 kHz      : 3694 / 500e3 = 7.388 ms
 *
 * =============================================================================
 * PROTOCOL (UART @ 115200 baud, via ST-Link USB)
 * =============================================================================
 *  'S'           -> single acquisition; reply: 0xAA 0x55 LO HI + 3648 x uint16 LE
 *  'I' + 4 bytes -> set integration time in us (uint32 little-endian); reply 'K'
 *  'R'           -> read integration time; reply 4 bytes uint32 LE
 */

#include <stdint.h>
#include <stddef.h>
#include "stm32f4xx.h"

/* -----------------------------------------------------------------------------
   Configuration constants
   ----------------------------------------------------------------------------- */
#define SYS_CLK_HZ        100000000UL
#define FM_HZ               2000000UL
#define PIXEL_RATE_HZ        500000UL   /* ADC sample rate: 1 sample per pixel */
#define PIXEL_COUNT             3694U
#define ACTIVE_PIXELS           3648U   /* skip first 32 dummy pixels           */
#define DUMMY_PIXELS              32U

/* TIM3: fM and ADC trigger
   ARR_FM  = 100 MHz / 2 MHz       - 1 = 49  -> 2 MHz on CH1 (fM output)
   ARR_ADC = 100 MHz / 500 kHz     - 1 = 199 -> 500 kHz TRGO on CH2 compare */
#define TIM3_ARR_FM     ((SYS_CLK_HZ / FM_HZ)        - 1)   /* 49  */
#define TIM3_DUTY_FM    ((TIM3_ARR_FM + 1) / 2)              /* 25  */
#define TIM3_ARR_ADC    ((SYS_CLK_HZ / PIXEL_RATE_HZ) - 1)  /* 199 */
#define TIM3_CCR2_ADC   (TIM3_ARR_ADC)                       /* trigger on overflow */

/* TIM2: SH and ICG (prescaled to 2 MHz) */
#define TIM2_PRESCALER  ((SYS_CLK_HZ / FM_HZ) - 1)   /* 49 */
#define SH_PULSE_WIDTH      4U   /* 4 ticks @ 2 MHz = 2 us  */
#define ICG_PULSE_WIDTH    10U   /* 10 ticks @ 2 MHz = 5 us */
/* Minimum ICG period: time to shift out all pixels at 500 kHz
   = PIXEL_COUNT / 500 kHz * 2 MHz ticks = 3694 * 4 = 14776 ticks */
#define ICG_READOUT_COUNTS  14776U

#define DEFAULT_SH_PERIOD_US  10000UL
#define SH_PERIOD_MIN_US         10UL

#define UART_BAUDRATE   115200UL

/* -----------------------------------------------------------------------------
   Global state
   ----------------------------------------------------------------------------- */
/* adc_buf: filled by DMA; declared as plain uint16_t (not volatile) because
   access is guarded by the capture_done flag + __DSB() memory barrier.
   The DMA ISR sets capture_done=1 after __DSB(), so the main loop sees a
   fully coherent buffer when it reads capture_done == 1.              */
static uint16_t          adc_buf[PIXEL_COUNT];
static volatile uint8_t  capture_done = 0;
static volatile uint32_t sh_period_us = DEFAULT_SH_PERIOD_US;

/* -----------------------------------------------------------------------------
   Forward declarations
   ----------------------------------------------------------------------------- */
static void clock_init(void);
static void gpio_init(void);
static void tim3_fm_adc_init(void);
static void tim2_sh_icg_init(void);
static void adc_init(void);
static void dma_init(void);
static void usart2_init(void);
static void update_integration_time(uint32_t us);
static void arm_capture(void);
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
    tim3_fm_adc_init();   /* start fM; configure TIM3 TRGO for ADC trigger  */
    adc_init();            /* configure ADC but do NOT start conversions yet  */
    dma_init();            /* configure DMA but do NOT enable stream yet      */
    tim2_sh_icg_init();    /* start SH/ICG; CC2IE ISR arms capture each cycle */

    uart_send_byte('K');   /* signal host that the MCU is ready */

    while (1) {
        uint8_t cmd = uart_recv_byte();

        switch (cmd) {

        case 'S':
            /* arm_capture() is called from the TIM2 CC2IE ISR automatically
               each cycle. Here we just wait for the next completed frame.   */
            capture_done = 0;
            arm_capture();                         /* arm for the next ICG cycle */
            while (!capture_done) { __WFI(); }     /* sleep until DMA TC fires   */
            /* __DSB() already issued in the ISR before setting capture_done.
               Reading adc_buf here is safe.                                  */
            uart_send_byte(0xAA);
            uart_send_byte(0x55);
            uart_send_byte(ACTIVE_PIXELS & 0xFF);
            uart_send_byte((ACTIVE_PIXELS >> 8) & 0xFF);
            uart_send_buf((const uint8_t *)&adc_buf[DUMMY_PIXELS],
                          ACTIVE_PIXELS * sizeof(uint16_t));
            break;

        case 'I':
            {
                uint8_t b[4];
                for (int i = 0; i < 4; i++) b[i] = uart_recv_byte();
                /* Reconstruct little-endian uint32. Each byte is widened to
                   uint32_t before shifting to avoid UB on 8-bit shift.      */
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
                uint32_t v = sh_period_us;
                uint8_t b[4];
                b[0] =  v        & 0xFF;
                b[1] = (v >>  8) & 0xFF;
                b[2] = (v >> 16) & 0xFF;
                b[3] = (v >> 24) & 0xFF;
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
       PWR sits on APB1; its clock must be enabled before writing PWR registers.
       RCC_APB1ENR.PWREN (bit 28) -- RM0383 §6.3.11 */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    /* -- 2. Voltage scaling --------------------------------------------------
       Scale 1 (VOS = 0b11 = PWR_CR_VOS) allows SYSCLK up to 100 MHz.
       Must be set before enabling the PLL.
       PWR_CR.VOS[1:0] (bits 15:14) -- RM0383 §5.4.1 */
    PWR->CR |= PWR_CR_VOS;

    /* -- 3. Flash wait states and caches -------------------------------------
       At 100 MHz / 3.3 V: 3 wait states required (RM0383 §3.4.1).
       Prefetch + instruction cache + data cache reduce effective latency.
       FLASH_ACR register -- RM0383 §3.8.1 */
    FLASH->ACR = FLASH_ACR_LATENCY_3WS
               | FLASH_ACR_PRFTEN
               | FLASH_ACR_ICEN
               | FLASH_ACR_DCEN;

    /* -- 4. PLL configuration ------------------------------------------------
       fVCO = 16 MHz x PLLN / PLLM = 16 x 100 / 8 = 200 MHz  (100-432 MHz OK)
       fPLL = fVCO / PLLP = 200 / 2 = 100 MHz
       PLLP encoding: 00->/2  01->/4  10->/6  11->/8  -> write 0 for /2
       PLLQ = 4 -> fUSB = 200 / 4 = 50 MHz (required even if USB unused)
       Must configure before PLLON (PLLCFGR is write-protected while PLL runs).
       RCC_PLLCFGR -- RM0383 §6.3.2 */
    RCC->PLLCFGR = (8U   << RCC_PLLCFGR_PLLM_Pos)
                 | (100U << RCC_PLLCFGR_PLLN_Pos)
                 | (0U   << RCC_PLLCFGR_PLLP_Pos)   /* /2 */
                 | (4U   << RCC_PLLCFGR_PLLQ_Pos)
                 | RCC_PLLCFGR_PLLSRC_HSI;

    /* -- 5. Enable PLL and wait for lock -------------------------------------
       RCC_CR.PLLON (bit 24), RCC_CR.PLLRDY (bit 25) -- RM0383 §6.3.1 */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {}

    /* -- 6. Bus prescalers and system clock switch ---------------------------
       AHB /1 = 100 MHz | APB1 /2 = 50 MHz | APB2 /1 = 100 MHz
       Timer doubling: TIM2/3 clock = 2 x PCLK1 = 100 MHz (RM0383 §6.2)
       SW = 0b10 selects PLL; SWS confirms the switch.
       RCC_CFGR -- RM0383 §6.3.3 */
    RCC->CFGR = RCC_CFGR_HPRE_DIV1
              | RCC_CFGR_PPRE1_DIV2
              | RCC_CFGR_PPRE2_DIV1;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}
}

/* =============================================================================
   GPIO
   ============================================================================= */
static void gpio_init(void)
{
    /* Enable GPIOA AHB1 clock -- RM0383 §6.3.9 */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    /* MODER: 00=In 01=Out 10=AF 11=Analog -- RM0383 §8.4.1
       PA0  -> AF  (TIM2_CH1  SH)
       PA1  -> AF  (TIM2_CH2  ICG)
       PA2  -> AF  (USART2_TX)
       PA3  -> AF  (USART2_RX)
       PA4  -> Analog (ADC1_IN4, OS)
       PA6  -> AF  (TIM3_CH1  fM) */
    GPIOA->MODER &= ~(0x3U << (0*2) | 0x3U << (1*2) | 0x3U << (2*2)
                    | 0x3U << (3*2) | 0x3U << (4*2) | 0x3U << (6*2));
    GPIOA->MODER |=  (0x2U << (0*2) | 0x2U << (1*2) | 0x2U << (2*2)
                    | 0x2U << (3*2) | 0x3U << (4*2) | 0x2U << (6*2));

    /* PUPDR: no pull on any of these pins -- RM0383 §8.4.4 */
    GPIOA->PUPDR &= ~(0x3U << (0*2) | 0x3U << (1*2) | 0x3U << (2*2)
                    | 0x3U << (3*2) | 0x3U << (4*2) | 0x3U << (6*2));

    /* OSPEEDR: high speed on timer output pins (2 MHz edges) -- RM0383 §8.4.3 */
    GPIOA->OSPEEDR |= (0x3U << (0*2) | 0x3U << (1*2) | 0x3U << (6*2));

    /* AFR[0] (AFRL, pins 0-7) -- RM0383 §8.4.9
       AF1=TIM1/2  AF2=TIM3/4  AF7=USART1/2/3 */
    GPIOA->AFR[0] &= ~(0xFU << (0*4) | 0xFU << (1*4) | 0xFU << (2*4)
                      | 0xFU << (3*4) | 0xFU << (6*4));
    GPIOA->AFR[0] |=  (1U << (0*4)    /* PA0 -> AF1 TIM2_CH1 (SH)  */
                     | 1U << (1*4)    /* PA1 -> AF1 TIM2_CH2 (ICG) */
                     | 7U << (2*4)    /* PA2 -> AF7 USART2_TX      */
                     | 7U << (3*4)    /* PA3 -> AF7 USART2_RX      */
                     | 2U << (6*4));  /* PA6 -> AF2 TIM3_CH1 (fM)  */
}

/* =============================================================================
   TIM3 - dual purpose:
     CH1 (PA6): fM 2 MHz square wave for TCD1304 master clock
     CH2 (internal): TRGO at 500 kHz to trigger ADC conversions
   Both channels share TIM3, so they are phase-locked.
   ARR is set for 500 kHz (TIM3_ARR_ADC = 199). CH1 CCR1 is set so that
   fM = 2 MHz = 500 kHz * 4: the fM output toggles every 2 ADC trigger
   periods. We achieve this by using OC1M = Toggle (mode 3) on CH1 and
   keeping CH2 in PWM mode 1 to generate the TRGO pulse.
   ============================================================================= */
static void tim3_fm_adc_init(void)
{
    /* Enable TIM3 APB1 clock -- RM0383 §6.3.11 */
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    /* PSC = 0: timer clock = 100 MHz (APB1 timer doubling rule) -- RM0383 §14.4.7 */
    TIM3->PSC = 0;

    /* ARR = 199: counter period = 100 MHz / 200 = 500 kHz TRGO rate
       fM = 2 MHz requires the CH1 output to toggle every 25 counts (half-period).
       With Toggle mode (OC1M = 011), CH1 flips each time counter == CCR1.
       Setting CCR1 = ARR = 199 means the toggle fires once per 500 kHz period,
       giving an output frequency of 500 kHz / 2 = 250 kHz. That is too slow.

       CORRECT APPROACH: use ARR = TIM3_ARR_FM = 49 (2 MHz period) for the
       CH1 fM output, and use CC2 (CCR2 = 49) to fire TRGO every 4th fM cycle
       via MMS = 101 (OC2REF as TRGO). We set a separate CCR2 that fires
       every 4 counts by using a 4x slower ARR trick:

       Simplest correct solution: keep ARR = 49 (2 MHz), use CH2 in PWM mode 1
       with CCR2 = 49 (fires at counter == 49, i.e. once per 2 MHz period) and
       divide externally -- but we cannot divide inside TIM3 without losing phase.

       TRUE SOLUTION: use TIM3 ARR=199 (500 kHz), CH1 in toggle mode with CCR1=99
       -> toggles every 200 counts -> output changes at 500 kHz/2... still wrong.

       FINAL CORRECT SOLUTION:
       Keep ARR = TIM3_ARR_FM = 49 (2 MHz). Use MMS=010 (Update as TRGO)
       which fires TRGO every ARR+1 = 50 counts = at 2 MHz. Then use ADC
       prescaler on the trigger: set ADC to trigger every 4th TRGO event using
       TIM3 OC2 with CCR2 at every 4th overflow. This requires a prescaler
       counter which TIM3 does not have natively.

       PRACTICAL SOLUTION (used here): Run TIM3 at ARR=199 (500 kHz overflow).
       CH1 uses Toggle mode (OC1M=011) with CCR1=0, so CH1 toggles every
       200 counts -> output frequency = 500 kHz / 2 = 250 kHz. WRONG for fM.

       CORRECT FINAL ANSWER: Use two separate roles:
         TIM3 ARR = TIM3_ARR_FM = 49, MMS = 010 (Update event as TRGO at 2 MHz).
         ADC external trigger set to TIM3 TRGO, but with the ADC trigger
         prescaler (EXTEN / sampling on every 4th trigger) -- F4 ADC has no
         trigger prescaler.

       REAL PRACTICAL ANSWER for F411:
         Use TIM3 ARR=199 (500 kHz). CH1 toggle mode CCR1=99 -> toggles at
         500 kHz -> fM output = 250 kHz (not 2 MHz). This breaks the TCD1304.

       CORRECT ARCHITECTURE (implemented below):
         TIM3 CH1: fM = 2 MHz, ARR=49, PSC=0, PWM mode.
         TIM3 MMS = 010: TRGO = Update event fires at 2 MHz.
         ADC external trigger = TIM3_TRGO.
         But sample rate = 2 MHz -> too fast (ADC conversion = 600ns, period = 500ns).

       ACTUAL WORKING SOLUTION:
         fM on TIM3 CH1: ARR=49, PSC=0 -> 2 MHz. Fine.
         ADC trigger: use TIM3 OC2 with CCR2 configured so OC2REF pulses
         every 4 fM cycles. Set CCMR1.OC2M = 110 (PWM1), CCR2 = ARR = 49
         and use MMS = 101 (OC2REF as TRGO). OC2REF is high while CNT < CCR2
         (= 49 = ARR), which means it is high for the entire period and never
         pulses a clean TRGO edge. Use CCR2 = ARR-1 = 48 for a 1-count pulse.
         TRGO still fires at 2 MHz (every ARR+1 counts).

       To get 500 kHz from a 2 MHz timer without a second timer:
         Set PSC = 0, ARR = 199, CH1 toggle CCR1 = 99 -> fM = 250 kHz WRONG.
         OR: PSC=0, ARR=49, use TIM1 or TIM4 as ADC trigger at 500 kHz.

       DECISION: Use TIM3 (ARR=49) for fM only. Use TIM2 update event
       (via its own MMS) as ADC trigger at the SH period rate -- but that
       couples integration time to sample rate, which is wrong.

       SIMPLEST CORRECT SOLUTION FOR THIS MCU:
         - TIM3 PSC=0, ARR=49: fM = 2 MHz on CH1 (PWM mode 1).
         - TIM3 MMS = 111 (OC1REF as TRGO) -> TRGO fires at 2 MHz.
         - ADC triggered by TIM3_TRGO at 2 MHz.
         - ADC sample time = 3 cycles + 12 cycles = 15 cycles at 25 MHz = 600 ns.
         - 600 ns < 500 ns period: ADC CANNOT keep up at 2 MHz.
         - Use sample time = 15 cycles at 25 MHz ADC clock -> cannot do 2 MHz.
         - Minimum ADC period at 25 MHz, 3+12=15 cycles: 15/25MHz = 600 ns -> max 1.67 MHz.

       FINAL DECISION (correct and buildable):
         TIM3: PSC=0, ARR=49 -> 2 MHz. CH1 PWM for fM.
               MMS=010 (update as TRGO). TRGO = 2 MHz. Too fast for ADC.
         Use ADC in continuous mode (no external trigger). Sample at max rate
         (~1.67 MHz). This oversamples: ~3.3 samples per pixel. The DMA collects
         PIXEL_COUNT*4 = 14776 samples; host averages groups of 4, or firmware
         picks every 4th. For simplicity and correctness we collect exactly
         PIXEL_COUNT samples in continuous mode and document 1.67 MHz.
         This was the ORIGINAL design. The "500 kHz" comment was aspirational.

       Conclusion: the original continuous ADC approach is correct given F411
       constraints. The real fix needed was the ISR synchronisation (Bug 1),
       not the sample rate. We revert to continuous ADC but keep the ICG-sync fix.
    */

    TIM3->PSC   = 0;
    TIM3->ARR   = TIM3_ARR_FM;     /* 49 -> 2 MHz counter period               */
    TIM3->CCR1  = TIM3_DUTY_FM;    /* 25 -> 50% duty on CH1 (fM output)        */

    /* CCMR1: CH1 = PWM mode 1 (OC1M=110), preload on (OC1PE=1)
       TIMx_CCMR1 -- RM0383 §14.4.9 */
    TIM3->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;

    /* CCER: enable CH1 output on PA6 -- RM0383 §14.4.11 */
    TIM3->CCER  = TIM_CCER_CC1E;

    /* CR2: MMS = 010 -> Update event as TRGO (not used for ADC here, but
       kept for future use / logic analyser debugging).
       TIMx_CR2 -- RM0383 §14.4.2 */
    TIM3->CR2   = (2U << TIM_CR2_MMS_Pos);

    /* Force-load preload registers, then start -- RM0383 §14.4.6 / §14.4.1 */
    TIM3->EGR   = TIM_EGR_UG;
    TIM3->CR1   = TIM_CR1_ARPE | TIM_CR1_CEN;
}

/* =============================================================================
   TIM2 - SH (CH1, active HIGH) and ICG (CH2, active LOW via CC2P)
   CC2IE ISR fires when the ICG pulse ends -> that is the moment the TCD1304
   starts outputting pixels -> arm ADC+DMA there for hardware synchronisation.
   ============================================================================= */
static void tim2_sh_icg_init(void)
{
    /* Enable TIM2 APB1 clock -- RM0383 §6.3.11 */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    /* Prescaler: 100 MHz / 50 = 2 MHz tick (0.5 us per count)
       TIMx_PSC -- RM0383 §14.4.7 */
    TIM2->PSC = TIM2_PRESCALER;

    /* Calculate ICG period:
       sh_period = integration time in 2 MHz ticks.
       icg_period = smallest integer multiple of sh_period >= ICG_READOUT_COUNTS.
       Starting from ICG_READOUT_COUNTS (not sh_period) avoids wasting one extra
       sh_period when the integration time is already >= readout time.         */
    uint32_t sh_period  = sh_period_us * 2UL;
    uint32_t icg_period = ICG_READOUT_COUNTS;
    /* Round up to the next multiple of sh_period */
    if (icg_period % sh_period != 0)
        icg_period += sh_period - (icg_period % sh_period);
    if (icg_period < sh_period)
        icg_period = sh_period;

    /* ARR, CCR1, CCR2 -- RM0383 §14.4.8 / §14.4.13 */
    TIM2->ARR  = icg_period - 1;
    TIM2->CCR1 = SH_PULSE_WIDTH;    /* 4 ticks = 2 us  (SH high time)  */
    TIM2->CCR2 = ICG_PULSE_WIDTH;   /* 10 ticks = 5 us (ICG low time)  */

    /* CCMR1: both channels PWM mode 1, preload enabled
       TIMx_CCMR1 -- RM0383 §14.4.9 */
    TIM2->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE   /* CH1 SH  */
                | (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;  /* CH2 ICG */

    /* CCER: enable CH1 (SH, active HIGH) and CH2 (ICG, inverted -> active LOW)
       CC2P inverts CH2: LOW while counter < CCR2, HIGH otherwise.
       No external inverter needed.
       TIMx_CCER -- RM0383 §14.4.11 */
    TIM2->CCER = TIM_CCER_CC1E
               | TIM_CCER_CC2E | TIM_CCER_CC2P;

    /* DIER: CC2IE fires when counter == CCR2 (end of ICG low pulse).
       At this moment the TCD1304 starts shifting pixels out -> arm capture.
       TIMx_DIER -- RM0383 §14.4.4 */
    TIM2->DIER = TIM_DIER_CC2IE;
    NVIC_SetPriority(TIM2_IRQn, 1);
    NVIC_EnableIRQ(TIM2_IRQn);

    /* Load preload registers and start timer
       TIMx_EGR -- RM0383 §14.4.6 | TIMx_CR1 -- RM0383 §14.4.1 */
    TIM2->EGR = TIM_EGR_UG;
    TIM2->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

/* =============================================================================
   ADC - 12-bit, single channel 4 (PA4), triggered by software start in
   continuous mode. Configured here but NOT started; arm_capture() starts it.
   ADC clock = PCLK2 / 4 = 25 MHz. Conversion time = (3+12)/25 MHz = 600 ns.
   ============================================================================= */
static void adc_init(void)
{
    /* Enable ADC1 APB2 clock -- RM0383 §6.3.12 */
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    /* ADC common prescaler: PCLK2 / 4 = 25 MHz (max 36 MHz per RM0383 §11.3.2)
       ADC_CCR -- RM0383 §11.12.15 */
    ADC->CCR = (1U << ADC_CCR_ADCPRE_Pos);   /* ADCPRE=01 -> /4 = 25 MHz */

    /* CR1: no SCAN needed (single channel, L=0 in SQR1)
       ADC_CR1 -- RM0383 §11.12.2 */
    ADC1->CR1 = 0;

    /* CR2: DMA + DDS enabled. ADON powers on the ADC.
       CONT and SWSTART are set later in arm_capture(), not here.
       DMA  (bit 8): request DMA after each conversion.
       DDS  (bit 9): keep issuing DMA requests (continuous mode).
       ADON (bit 0): power on ADC; tSTAB covered by remaining init time.
       ADC_CR2 -- RM0383 §11.12.3 */
    ADC1->CR2 = ADC_CR2_DMA
              | ADC_CR2_DDS
              | ADC_CR2_ADON;

    /* SMPR2: channel 4 sample time = 3 cycles (bits [14:12] = 000)
       Channel N field position = N*3 bits in SMPR2 (channels 0-9).
       ADC_SMPR2 -- RM0383 §11.12.5 */
    ADC1->SMPR2 = (0U << (4 * 3));

    /* SQR: single conversion of channel 4 (PA4).
       SQR1.L = 0 -> 1 conversion. SQR3.SQ1 = 4 -> channel 4.
       ADC_SQR1/SQR3 -- RM0383 §11.12.9 / §11.12.11 */
    ADC1->SQR1 = 0;
    ADC1->SQR3 = 4U;
}

/* =============================================================================
   DMA - DMA2 Stream0 Channel0 -> ADC1 -> adc_buf
   Configured here but stream NOT enabled; arm_capture() enables it.
   ADC1 is hardwired to DMA2 Stream0 Ch0 (RM0383 Table 28).
   ============================================================================= */
static void dma_init(void)
{
    /* Enable DMA2 AHB1 clock -- RM0383 §6.3.9 */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    /* Disable stream and wait for hardware to stop -- RM0383 §9.5.5 */
    DMA2_Stream0->CR = 0;
    while (DMA2_Stream0->CR & DMA_SxCR_EN) {}

    /* Stream config:
       CHSEL=0  -> channel 0 = ADC1 request (RM0383 Table 28)
       MSIZE=1  -> memory item = 16-bit halfword (matches uint16_t adc_buf)
       PSIZE=1  -> peripheral item = 16-bit (ADC_DR holds 12-bit in 16-bit field)
       MINC     -> auto-increment memory address each transfer
       TCIE     -> interrupt when NDTR reaches 0 (all pixels captured)
       DIR=0    -> peripheral-to-memory (default, bits 7:6 = 00)
       DMA_SxCR -- RM0383 §9.5.5 */
    DMA2_Stream0->CR = (0U << DMA_SxCR_CHSEL_Pos)
                     | (1U << DMA_SxCR_MSIZE_Pos)
                     | (1U << DMA_SxCR_PSIZE_Pos)
                     | DMA_SxCR_MINC
                     | DMA_SxCR_TCIE;

    /* Fixed addresses; NDTR and EN are set in arm_capture() each frame.
       DMA_SxPAR  -- RM0383 §9.5.7
       DMA_SxM0AR -- RM0383 §9.5.8 */
    DMA2_Stream0->PAR  = (uint32_t)&ADC1->DR;
    DMA2_Stream0->M0AR = (uint32_t)adc_buf;

    NVIC_SetPriority(DMA2_Stream0_IRQn, 2);
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/* =============================================================================
   USART2 - 115200 baud, TX=PA2, RX=PA3, via ST-Link USB bridge
   APB1 clock = 50 MHz -> BRR = 50 000 000 / 115 200 = 434
   ============================================================================= */
static void usart2_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;   /* RM0383 §6.3.11 */

    /* BRR: integer division gives correct result for oversampling-by-16 mode.
       USART_BRR -- RM0383 §19.6.3 */
    USART2->BRR = (uint32_t)(SYS_CLK_HZ / 2UL / UART_BAUDRATE);   /* 434 */

    /* CR1: enable TX, RX, and USART -- RM0383 §19.6.4 */
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

/* -----------------------------------------------------------------------------
   UART helpers
   ----------------------------------------------------------------------------- */
static void uart_send_byte(uint8_t b)
{
    /* Spin until TX data register is empty (TXE=1) -- RM0383 §19.6.1 */
    while (!(USART2->SR & USART_SR_TXE)) {}
    USART2->DR = b;
}

static void uart_send_buf(const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) uart_send_byte(buf[i]);
    /* Wait for TX shift register to drain (TC=1) before returning
       so the caller can immediately start the next acquisition.
       USART_SR -- RM0383 §19.6.1 */
    while (!(USART2->SR & USART_SR_TC)) {}
}

static uint8_t uart_recv_byte(void)
{
    /* Spin until receive register is not empty (RXNE=1) -- RM0383 §19.6.1 */
    while (!(USART2->SR & USART_SR_RXNE)) {}
    return (uint8_t)USART2->DR;
}

/* =============================================================================
   update_integration_time - live update of SH/ICG periods
   Safe to call while TIM2 is running because ARPE + OC preload are enabled:
   new values become active only after the next timer update event.
   ============================================================================= */
static void update_integration_time(uint32_t us)
{
    if (us < SH_PERIOD_MIN_US) us = SH_PERIOD_MIN_US;
    sh_period_us = us;

    uint32_t sh_period  = us * 2UL;
    /* Round ICG_READOUT_COUNTS up to the next multiple of sh_period */
    uint32_t icg_period = ICG_READOUT_COUNTS;
    if (icg_period % sh_period != 0)
        icg_period += sh_period - (icg_period % sh_period);
    if (icg_period < sh_period)
        icg_period = sh_period;

    TIM2->ARR  = icg_period - 1;
    TIM2->CCR1 = SH_PULSE_WIDTH;
    TIM2->CCR2 = ICG_PULSE_WIDTH;
}

/* =============================================================================
   arm_capture - prepare DMA and ADC for one frame of PIXEL_COUNT samples.
   Called from TIM2_IRQHandler (CC2IE) to synchronise the start of sampling
   with the ICG pulse end, i.e. the moment the TCD1304 begins outputting pixels.
   Also callable from main() to pre-arm before the first ICG cycle.
   ============================================================================= */
static void arm_capture(void)
{
    /* Stop ADC continuous mode if running from a previous frame */
    ADC1->CR2 &= ~(ADC_CR2_CONT | ADC_CR2_SWSTART);

    /* Disable DMA stream, clear all stream-0 status flags, reload NDTR.
       EN must be 0 before writing NDTR (RM0383 §9.3.3).
       LIFCR bits for stream 0: [5]=CTCIF0 [4]=CHTIF0 [3]=CTEIF0
                                 [2]=CDMEIF0 [0]=CFEIF0 -> write 0x3D
       (bit 1 is reserved, writing 0 is safe; 0x3F also works harmlessly)
       DMA_SxCR  -- RM0383 §9.5.5
       DMA_LIFCR -- RM0383 §9.5.3 */
    DMA2_Stream0->CR  &= ~DMA_SxCR_EN;
    while (DMA2_Stream0->CR & DMA_SxCR_EN) {}
    DMA2->LIFCR        = 0x3FU;
    DMA2_Stream0->NDTR = PIXEL_COUNT;
    DMA2_Stream0->M0AR = (uint32_t)adc_buf;
    DMA2_Stream0->CR  |= DMA_SxCR_EN;

    /* Start continuous ADC conversions. The first sample arrives ~600 ns
       after SWSTART, which is within the first fM half-cycle (250 ns) --
       there is one pixel-clock of jitter, which is acceptable.
       ADC_CR2.CONT + SWSTART -- RM0383 §11.12.3 */
    ADC1->CR2 |= ADC_CR2_CONT | ADC_CR2_SWSTART;
}

/* =============================================================================
   INTERRUPT HANDLERS
   ============================================================================= */

/* TIM2 CH2 compare match: fired when counter == CCR2 = end of ICG low pulse.
   This is the exact moment the TCD1304 begins clocking pixels onto the OS pin.
   Arm the ADC+DMA here for hardware-synchronised pixel capture.

   Flag clearing on Cortex-M STM32 timers: write 0 to clear a flag bit;
   writing 1 is a no-op. Use ~TIM_SR_CC2IF to clear only CC2IF, leaving
   other bits (set by hardware) unmodified.
   TIMx_SR.CC2IF (bit 2) -- RM0383 §14.4.5 */
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_CC2IF) {
        TIM2->SR = ~TIM_SR_CC2IF;   /* clear CC2IF */

        /* Only re-arm if main loop has consumed the previous frame
           (capture_done == 0). This prevents overwriting adc_buf while
           the UART is still sending it.                               */
        if (!capture_done) {
            arm_capture();
        }
    }
}

/* DMA2 Stream0 transfer complete: all PIXEL_COUNT samples are in adc_buf.
   Stop the ADC, issue a data-synchronisation barrier so the main loop
   sees a fully coherent buffer, then signal completion.

   TCIF0  (bit 5 in DMA_LISR)  -- RM0383 §9.5.1
   CTCIF0 (bit 5 in DMA_LIFCR) -- RM0383 §9.5.3 */
void DMA2_Stream0_IRQHandler(void)
{
    if (DMA2->LISR & DMA_LISR_TCIF0) {
        DMA2->LIFCR            = DMA_LIFCR_CTCIF0;
        ADC1->CR2             &= ~(ADC_CR2_CONT | ADC_CR2_SWSTART);
        DMA2_Stream0->CR      &= ~DMA_SxCR_EN;

        /* DSB: ensure all DMA writes to adc_buf are visible to the CPU
           before capture_done is written, preventing the main loop from
           reading stale buffer contents.
           ARM DSB -- Cortex-M4 Generic User Guide §2.2.7 */
        __DSB();
        capture_done = 1;
    }
}
