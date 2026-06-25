/**
 * @file    main.c
 * @brief   TCD1304 Linear CCD Driver for STM32F411 (Nucleo-F411RE)
 *
 * =============================================================================
 * PIN MAPPING (Nucleo-F411RE / STM32F411RE)
 * =============================================================================
 *
 *  Signal  | MCU Pin  | Timer       | Alt Func | Connector
 * ---------+----------+-------------+----------+-----------
 *  fM      | PA6      | TIM3 CH1    | AF2      | CN10-13
 *  SH      | PA0      | TIM2 CH1    | AF1      | CN7-28
 *  ICG     | PA1      | TIM2 CH2    | AF1      | CN7-30
 *  OS(ADC) | PA4      | ADC1 IN4    | Analog   | CN7-32
 *  UART TX | PA2      | USART2      | AF7      | (ST-Link)
 *  UART RX | PA3      | USART2      | AF7      | (ST-Link)
 *
 * =============================================================================
 * TIMING OVERVIEW (fM = 2 MHz, SYS_CLK = 100 MHz)
 * =============================================================================
 *
 *  fM   : 2 MHz square wave → TIM3, period = 50 (100 MHz / 2 MHz), 50% duty
 *  ADC  : triggered at 500 kHz (4 fM cycles per pixel)
 *  SH   : min 1000 ns high → TIM2 CH1, prescaler to 2 MHz
 *  ICG  : min 1000 ns > SH high time → TIM2 CH2
 *
 *  TIM2 prescaler: (100 MHz / 2 MHz) - 1 = 49
 *  SH  pulse width : 4 cycles at 2 MHz = 2 µs
 *  ICG pulse width : 10 cycles at 2 MHz = 5 µs
 *  SH  period      : configurable, minimum 20 cycles (10 µs)
 *  ICG period      : configurable, minimum 14776 cycles (7.4 ms readout)
 *
 *  Total pixels (incl. dummies): 3694
 *  Readout time @ 0.5 MHz : 3694 / 0.5e6 = 7.388 ms
 *
 * =============================================================================
 * PROTOCOL (UART @ 115200 baud, via ST-Link USB)
 * =============================================================================
 *  'S'             → single acquisition, send back 3648 uint16
 *  'I' + 4 bytes   → set integration time in µs (little-endian)
 *  'R'             → read integration time back
 */

#include <stdint.h>
#include <string.h>
#include "stm32f4xx.h"

/* ─────────────────────────────────────────────
   Configuration constants
   ───────────────────────────────────────────── */
#define SYS_CLK_HZ       100000000UL
#define FM_HZ            2000000UL
#define PIXEL_COUNT      3694U
#define ACTIVE_PIXELS    3648U

#define TIM3_PERIOD      (SYS_CLK_HZ / FM_HZ)   /* 50 */
#define TIM3_DUTY        (TIM3_PERIOD / 2)        /* 25 */
#define TIM2_PRESCALER   ((SYS_CLK_HZ / FM_HZ) - 1)  /* 49 */

#define SH_PULSE_WIDTH   4U
#define ICG_PULSE_WIDTH  10U

#define DEFAULT_SH_PERIOD_US  10000UL
#define SH_PERIOD_MIN_US      10UL
#define ICG_READOUT_COUNTS    14776U

#define UART_BAUDRATE    115200UL

/* ─────────────────────────────────────────────
   Global state
   ───────────────────────────────────────────── */
static volatile uint16_t adc_buf[PIXEL_COUNT];
static volatile uint32_t adc_idx        = 0;
static volatile uint8_t  capture_done   = 0;
static volatile uint32_t sh_period_us   = DEFAULT_SH_PERIOD_US;

/* ─────────────────────────────────────────────
   Forward declarations
   ───────────────────────────────────────────── */
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

/* ═══════════════════════════════════════════════════════════════════════════
   MAIN
   ═══════════════════════════════════════════════════════════════════════════ */
int main(void)
{
    clock_init();
    gpio_init();
    usart2_init();
    tim3_fm_init();
    adc_init();
    dma_init();
    tim2_sh_icg_init();

    uart_send_byte('K');

    while (1) {
        uint8_t cmd = uart_recv_byte();

        switch (cmd) {

        case 'S':
            start_acquisition();
            while (!capture_done) { __WFI(); }
            capture_done = 0;
            uart_send_byte(0xAA);
            uart_send_byte(0x55);
            uart_send_byte(ACTIVE_PIXELS & 0xFF);
            uart_send_byte((ACTIVE_PIXELS >> 8) & 0xFF);
            uart_send_buf((const uint8_t *)&adc_buf[32], ACTIVE_PIXELS * 2);
            break;

        case 'I':
        {
            uint8_t b[4];
            for (int i = 0; i < 4; i++) b[i] = uart_recv_byte();
            uint32_t us = (uint32_t)b[0]
                        | ((uint32_t)b[1] << 8)
                        | ((uint32_t)b[2] << 16)
                        | ((uint32_t)b[3] << 24);
            if (us < SH_PERIOD_MIN_US) us = SH_PERIOD_MIN_US;
            update_integration_time(us);
            uart_send_byte('K');
            break;
        }

        case 'R':
        {
            uint8_t b[4];
            b[0] =  sh_period_us        & 0xFF;
            b[1] = (sh_period_us >> 8)  & 0xFF;
            b[2] = (sh_period_us >> 16) & 0xFF;
            b[3] = (sh_period_us >> 24) & 0xFF;
            uart_send_buf(b, 4);
            break;
        }

        default:
            break;
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
   CLOCK  –  PLL: HSI 16 MHz → /8 → ×100 → /2 = 100 MHz
   APB1 /2 = 50 MHz (timer ×2 = 100 MHz),  APB2 /1 = 100 MHz
   ═══════════════════════════════════════════════════════════════════════════ */
static void clock_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS;

    FLASH->ACR = FLASH_ACR_LATENCY_3WS | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;

    RCC->PLLCFGR = (8U   << RCC_PLLCFGR_PLLM_Pos)
                 | (100U << RCC_PLLCFGR_PLLN_Pos)
                 | (0U   << RCC_PLLCFGR_PLLP_Pos)
                 | (4U   << RCC_PLLCFGR_PLLQ_Pos)
                 | RCC_PLLCFGR_PLLSRC_HSI;

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {}

    RCC->CFGR = RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}
}

/* ═══════════════════════════════════════════════════════════════════════════
   GPIO
   ═══════════════════════════════════════════════════════════════════════════ */
static void gpio_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    GPIOA->MODER &= ~(0x3U<<(0*2)|0x3U<<(1*2)|0x3U<<(2*2)
                    |0x3U<<(3*2)|0x3U<<(4*2)|0x3U<<(6*2));
    GPIOA->MODER |=  (0x2U<<(0*2))|(0x2U<<(1*2))|(0x2U<<(2*2))
                  |  (0x2U<<(3*2))|(0x3U<<(4*2))|(0x2U<<(6*2));

    GPIOA->PUPDR &= ~(0x3U<<(0*2)|0x3U<<(1*2)|0x3U<<(2*2)
                    |0x3U<<(3*2)|0x3U<<(4*2)|0x3U<<(6*2));

    GPIOA->OSPEEDR |= (0x3U<<(0*2))|(0x3U<<(1*2))|(0x3U<<(6*2));

    GPIOA->AFR[0] &= ~(0xFU<<(0*4)|0xFU<<(1*4)|0xFU<<(2*4)
                     |0xFU<<(3*4)|0xFU<<(6*4));
    GPIOA->AFR[0] |=  (1U<<(0*4))   /* PA0 TIM2_CH1 */
                   |  (1U<<(1*4))   /* PA1 TIM2_CH2 */
                   |  (7U<<(2*4))   /* PA2 USART2_TX */
                   |  (7U<<(3*4))   /* PA3 USART2_RX */
                   |  (2U<<(6*4));  /* PA6 TIM3_CH1 */
}

/* ═══════════════════════════════════════════════════════════════════════════
   TIM3 – fM 2 MHz on PA6
   ═══════════════════════════════════════════════════════════════════════════ */
static void tim3_fm_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM3->PSC  = 0;
    TIM3->ARR  = TIM3_PERIOD - 1;
    TIM3->CCR1 = TIM3_DUTY;
    TIM3->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM3->CCER  = TIM_CCER_CC1E;
    TIM3->EGR  = TIM_EGR_UG;
    TIM3->CR1  = TIM_CR1_ARPE | TIM_CR1_CEN;
}

/* ═══════════════════════════════════════════════════════════════════════════
   TIM2 – SH (CH1, active high) and ICG (CH2, active low)
   ═══════════════════════════════════════════════════════════════════════════ */
static void tim2_sh_icg_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->PSC = TIM2_PRESCALER;

    uint32_t sh_period  = sh_period_us * 2;
    uint32_t icg_period = sh_period;
    while (icg_period < ICG_READOUT_COUNTS + sh_period) icg_period += sh_period;

    TIM2->ARR  = icg_period - 1;
    TIM2->CCR1 = SH_PULSE_WIDTH;
    TIM2->CCR2 = ICG_PULSE_WIDTH;

    TIM2->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE
                | (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;

    /* CH2 inverted (active low = ICG active low with no external inverter) */
    TIM2->CCER  = TIM_CCER_CC1E
                | TIM_CCER_CC2E | TIM_CCER_CC2P;

    TIM2->DIER = TIM_DIER_CC2IE;
    NVIC_SetPriority(TIM2_IRQn, 1);
    NVIC_EnableIRQ(TIM2_IRQn);

    TIM2->EGR = TIM_EGR_UG;
    TIM2->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

/* ═══════════════════════════════════════════════════════════════════════════
   ADC – 12-bit, single channel 4 (PA4), DMA mode
   ═══════════════════════════════════════════════════════════════════════════ */
static void adc_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    ADC->CCR  = (1U << ADC_CCR_ADCPRE_Pos);  /* PCLK2/4 = 25 MHz */
    ADC1->CR1 = ADC_CR1_SCAN;
    ADC1->CR2 = ADC_CR2_DMA | ADC_CR2_DDS | ADC_CR2_ADON;
    ADC1->SMPR2 = (0U << (4*3));  /* 3 cycles on CH4 */
    ADC1->SQR1  = 0;
    ADC1->SQR3  = 4U;
}

/* ═══════════════════════════════════════════════════════════════════════════
   DMA – DMA2 Stream0 Channel0 → ADC1 → adc_buf
   ═══════════════════════════════════════════════════════════════════════════ */
static void dma_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    DMA2_Stream0->CR = 0;
    while (DMA2_Stream0->CR & DMA_SxCR_EN) {}

    DMA2_Stream0->CR  = (0U << DMA_SxCR_CHSEL_Pos)
                      | (1U << DMA_SxCR_MSIZE_Pos)
                      | (1U << DMA_SxCR_PSIZE_Pos)
                      | DMA_SxCR_MINC
                      | DMA_SxCR_TCIE;
    DMA2_Stream0->PAR  = (uint32_t)&ADC1->DR;
    DMA2_Stream0->M0AR = (uint32_t)adc_buf;
    DMA2_Stream0->NDTR = PIXEL_COUNT;

    NVIC_SetPriority(DMA2_Stream0_IRQn, 2);
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/* ═══════════════════════════════════════════════════════════════════════════
   USART2 – 115200 baud (APB1 clock = 50 MHz)
   ═══════════════════════════════════════════════════════════════════════════ */
static void usart2_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    USART2->BRR = (uint32_t)(SYS_CLK_HZ / 2 / UART_BAUDRATE);  /* ≈ 434 */
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

/* ─────────────────────────────────────────────
   UART helpers
   ───────────────────────────────────────────── */
static void uart_send_byte(uint8_t b)
{
    while (!(USART2->SR & USART_SR_TXE)) {}
    USART2->DR = b;
}

static void uart_send_buf(const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) uart_send_byte(buf[i]);
    while (!(USART2->SR & USART_SR_TC)) {}
}

static uint8_t uart_recv_byte(void)
{
    while (!(USART2->SR & USART_SR_RXNE)) {}
    return (uint8_t)USART2->DR;
}

/* ═══════════════════════════════════════════════════════════════════════════
   update_integration_time
   ═══════════════════════════════════════════════════════════════════════════ */
static void update_integration_time(uint32_t us)
{
    if (us < SH_PERIOD_MIN_US) us = SH_PERIOD_MIN_US;
    sh_period_us = us;

    uint32_t sh_period  = us * 2;
    uint32_t icg_period = sh_period;
    while (icg_period < ICG_READOUT_COUNTS + sh_period) icg_period += sh_period;

    TIM2->ARR  = icg_period - 1;
    TIM2->CCR1 = SH_PULSE_WIDTH;
    TIM2->CCR2 = ICG_PULSE_WIDTH;
}

/* ═══════════════════════════════════════════════════════════════════════════
   start_acquisition – arms DMA+ADC for one frame
   ═══════════════════════════════════════════════════════════════════════════ */
static void start_acquisition(void)
{
    capture_done = 0;
    adc_idx      = 0;

    DMA2_Stream0->CR  &= ~DMA_SxCR_EN;
    while (DMA2_Stream0->CR & DMA_SxCR_EN) {}
    DMA2->LIFCR = 0x3FU;
    DMA2_Stream0->NDTR = PIXEL_COUNT;
    DMA2_Stream0->M0AR = (uint32_t)adc_buf;
    DMA2_Stream0->CR  |= DMA_SxCR_EN;

    ADC1->CR2 |= ADC_CR2_CONT | ADC_CR2_SWSTART;
}

/* ═══════════════════════════════════════════════════════════════════════════
   INTERRUPT HANDLERS
   ═══════════════════════════════════════════════════════════════════════════ */
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_CC2IF) {
        TIM2->SR = ~TIM_SR_CC2IF;
    }
}

void DMA2_Stream0_IRQHandler(void)
{
    if (DMA2->LISR & DMA_LISR_TCIF0) {
        DMA2->LIFCR = DMA_LIFCR_CTCIF0;
        ADC1->CR2 &= ~(ADC_CR2_CONT | ADC_CR2_SWSTART);
        DMA2_Stream0->CR &= ~DMA_SxCR_EN;
        capture_done = 1;
    }
}
