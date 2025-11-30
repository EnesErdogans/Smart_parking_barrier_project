#include "stm32f4xx.h"

/*
 * ======================= PIN SUMMARY ==========================
 *
 *  LED (green)           ? PD12  (Output)
 *  Buzzer                ? PD13  (Output)
 *
 *  Ultrasonic Trigger    ? PB10 (Output)
 *  Ultrasonic Echo       ? PB11 (Input)
 *
 *  RFID CS               ? PA4  (Output)
 *  RFID RST              ? PB0  (Output)
 *
 *  Servo signal pin      ? PB6  (Output for now, will be PWM later)
 *
 *  The purpose of this file is to configure all required digital
 *  pins purely using **register-level programming**, as requested.
 *  No HAL, no libraries — only direct register access.
 * =================================================================
 */

/* ---------------- Basic SysTick delay (1 ms) ---------------- */
static volatile uint32_t msTicks = 0;

void SysTick_Handler(void)
{
    msTicks++;
}

void delay_ms(uint32_t ms)
{
    uint32_t target = msTicks + ms;
    while ((int32_t)(target - msTicks) > 0) {
        __NOP();
    }
}

/* ------------------- GPIO Configuration ---------------------- */
void GPIO_Init_All(void)
{
    /* Enable clocks for GPIOA, GPIOB and GPIOD */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
                    RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_GPIODEN;

    /* ==== PD12: LED  |  PD13: Buzzer ==== */
    /* Both pins are configured as simple digital outputs */
    GPIOD->MODER &= ~((3U<<(12*2)) | (3U<<(13*2)));
    GPIOD->MODER |=  ((1U<<(12*2)) | (1U<<(13*2)));

    /* ==== PB10: TRIG (output) | PB11: ECHO (input) ==== */
    GPIOB->MODER &= ~((3U<<(10*2)) | (3U<<(11*2)));
    GPIOB->MODER |=  (1U<<(10*2));     // PB10 = output
    // PB11 remains input
    GPIOB->PUPDR &= ~((3U<<(10*2)) | (3U<<(11*2)));
    GPIOB->PUPDR |=  (1U<<(11*2));     // pull-up for echo line

    /* ==== PA4: RFID CS (output) ==== */
    GPIOA->MODER &= ~(3U<<(4*2));
    GPIOA->MODER |=  (1U<<(4*2));

    /* ==== PB0: RFID RST (output) ==== */
    GPIOB->MODER &= ~(3U<<(0*2));
    GPIOB->MODER |=  (1U<<(0*2));

    /* ==== PB6: Servo pin (simple output for testing) ==== */
    GPIOB->MODER &= ~(3U<<(6*2));
    GPIOB->MODER |=  (1U<<(6*2));
}

/* ----------------- Helper functions for GPIO ------------------ */

void LED_On(void)        { GPIOD->BSRR = (1U<<12); }
void LED_Off(void)       { GPIOD->BSRR = (1U<<(12+16)); }
void LED_Toggle(void)    { GPIOD->ODR ^= (1U<<12); }

void Buzzer_On(void)     { GPIOD->BSRR = (1U<<13); }
void Buzzer_Off(void)    { GPIOD->BSRR = (1U<<(13+16)); }

void RFID_CS_Low(void)   { GPIOA->BSRR = (1U<<(4+16)); }  // CS = 0
void RFID_CS_High(void)  { GPIOA->BSRR = (1U<<4); }       // CS = 1

void RFID_Reset_Low(void)   { GPIOB->BSRR = (1U<<(0+16)); }
void RFID_Reset_High(void)  { GPIOB->BSRR = (1U<<0); }

void Ultrasonic_Trigger(void)
{
    /* A short trigger pulse. Not precise here, but enough for testing. */
    GPIOB->BSRR = (1U<<10);        // TRIG high
    delay_ms(1);
    GPIOB->BSRR = (1U<<(10+16));   // TRIG low
}

uint8_t Ultrasonic_ReadEcho(void)
{
    return (GPIOB->IDR & (1U<<11)) ? 1 : 0;
}

void Servo_TestPulse(void)
{
    /* For now we just toggle the servo pin to verify digital output */
    GPIOB->BSRR = (1U<<6);
    delay_ms(300);

    GPIOB->BSRR = (1U<<(6+16));
    delay_ms(300);
}

/* ------------------------------ MAIN --------------------------- */

int main(void)
{
    SystemInit();
    SysTick_Config(SystemCoreClock / 1000U);   // 1 ms SysTick

    GPIO_Init_All();

    /* Make sure RFID pins start in a proper state */
    RFID_Reset_High();
    RFID_CS_High();

    while (1)
    {
        /* 1) LED + Buzzer simple test */
        LED_Toggle();
        Buzzer_On();
        delay_ms(150);
        Buzzer_Off();

        /* 2) RFID CS toggle test */
        RFID_CS_Low();
        delay_ms(50);
        RFID_CS_High();

        /* 3) Ultrasonic TRIG/ECHO basic test */
        Ultrasonic_Trigger();
        delay_ms(5);

        if (Ultrasonic_ReadEcho())
            LED_On();
        else
            LED_Off();

        /* 4) Servo pin digital test */
        Servo_TestPulse();
    }
}
