/*===========================================================================
  SMART AIRPORT BAGGAGE MANAGEMENT SYSTEM  -  STM32F401CCU6 (Black Pill)
  Queue-Aware Edition:
    - Tracks Stalled Bags between Sensor 1 & Sensor 2
    - If a jammed ball is freed & reaches Sensor 2 -> Automatically counts it!
    - Automatically clears the Jam Alarm once all stalled bags exit
    - Real-time "Stalled Bags" count displayed on mobile phone app
  
  PINS:
    - Sensor 1 (Entry)  : PA1 (TCRT5000 Analog A0)  [3.3V Power, GND]
    - Sensor 2 (Exit)   : PA4 (TCRT5000 Analog A0)  [3.3V Power, GND]
    - Buzzer            : PA5 (Open-Drain: GND/Reset = ON, Float/Set = OFF)
    - Reset Button      : PB5 (EXTI5 Interrupt, shorts to GND on press)
    - On-board Blue LED : PC13 (Glows while bags are between sensors)
    - Bluetooth Module  : PA2 (TX) and PA3 (RX) at 9600 baud [HC-05]
 ===========================================================================*/

#include "stm32f4xx.h"

/*---------------------------------------------------------------------------
  SETTINGS
 ---------------------------------------------------------------------------*/
#define BELT_GAP_MM   150     // Distance between Sensor 1 and Sensor 2 (mm)
#define LEN_SMALL     50      // < 50mm  = Carry-on (SM)
#define LEN_LARGE     120     // > 120mm = Oversized (LG)
#define TH_ON         1800    // Reading BELOW this = Object present (reflection)
#define TH_OFF        2200    // Reading ABOVE this = Clear
#define TIMEOUT_MS    20000   // 20s conveyor stoppage after first bag -> Alert
#define PAIR_MS       2000    // 2.0s: if transit exceeds this -> Declare JAM
#define MIN_DT_MS     15      // Noise debounce: ignore pulses faster than 15ms
#define SEND_MS       1500    // Phone update rate when idle (1.5 seconds)

/*---------------------------------------------------------------------------
  GLOBAL VARIABLES
 ---------------------------------------------------------------------------*/
volatile uint32_t msTicks = 0;       // 1 ms hardware stopwatch

volatile uint32_t total = 0;         // Total successfully completed bags
volatile uint32_t inTransit = 0;     // Number of bags currently between Sensor 1 and 2
volatile uint32_t violations = 0;    // Bags going wrong direction (Exit -> Entry)
volatile uint32_t speed = 0;         // Conveyor speed (mm/s)
volatile uint32_t bagLength = 0;     // Measured bag length / diameter (mm)
volatile uint8_t  category = 3;      // 0=SM, 1=ST, 2=LG, 3=Unknown
volatile uint32_t alert = 0;         // 1 = Jam / Inactivity alert active

volatile uint32_t lastEventTime = 0;
volatile uint32_t lastS1Entry = 0;
volatile uint32_t s1Time = 0;
volatile uint32_t s2entry = 0;
volatile uint32_t buzzerEndTime = 0;

volatile uint8_t  s1Blocked = 0;     // Sensor 1 beam state (1 = blocked, 0 = clear)
volatile uint8_t  s2Blocked = 0;     // Sensor 2 beam state (1 = blocked, 0 = clear)
volatile uint8_t  measuringLength = 0;
volatile uint8_t  newEvent = 0;
volatile uint8_t  resetFlag = 0;

/* SysTick Interrupt: runs every 1 ms */
void SysTick_Handler(void)
{
    msTicks++;
}

/*---------------------------------------------------------------------------
  BUZZER CONTROL (PA5 - Open-Drain Active-Low)
  - ON  : Reset PA5 (bit 21) pulls pin hard to GND -> Current flows -> BEEP ON
  - OFF : Set PA5 (bit 5) floats pin to High-Z     -> Zero leakage  -> SILENT
 ---------------------------------------------------------------------------*/
static inline void buzzerOn(void)
{
    GPIOA->BSRR = (1U << 21); // Reset PA5 (bit 21) to GND -> ON
}

static inline void buzzerOff(void)
{
    GPIOA->BSRR = (1U << 5);  // Set PA5 (bit 5) to High-Z (Floating) -> OFF
}

/*---------------------------------------------------------------------------
  UART FUNCTIONS (USART2 on PA2, 9600 baud)
 ---------------------------------------------------------------------------*/
static void uartSendChar(char c)
{
    while (!(USART2->SR & (1U << 7))) {} // Wait TXE
    USART2->DR = (uint8_t)c;
}

static void uartSendString(const char *str)
{
    while (*str) {
        uartSendChar(*str++);
    }
}

static void uartSendNumber(uint32_t n)
{
    char buf[11];
    int i = 0;
    if (n == 0) {
        uartSendChar('0');
        return;
    }
    while (n > 0) {
        buf[i++] = (char)('0' + (n % 10));
        n /= 10;
    }
    while (i > 0) {
        uartSendChar(buf[--i]);
    }
}

/*---------------------------------------------------------------------------
  CLEAR, LABELED DATA TRANSMISSION TO MOBILE PHONE
  Formatted with standard CRLF (\r\n) for clean display on terminal apps
 ---------------------------------------------------------------------------*/
static void sendData(void)
{
    const char *catStr = "None";
    if (category == 0)      catStr = "Carry-on (SM)";
    else if (category == 1) catStr = "Standard Hold (ST)";
    else if (category == 2) catStr = "Oversized (LG)";

    uartSendString("\r\n==============================\r\n");
    uartSendString("   AIRPORT BAGGAGE STATUS\r\n");
    uartSendString("==============================\r\n");

    /* Total Bags Completed */
    uartSendString("Total Bags   : ");
    uartSendNumber(total);
    uartSendString("\r\n");

    /* Live Count of Bags between Sensor 1 & Sensor 2 */
    uartSendString("Stalled Bags : ");
    uartSendNumber(inTransit);
    uartSendString(" (between sensors)\r\n");

    /* Measured Conveyor Speed */
    uartSendString("Speed        : ");
    uartSendNumber(speed);
    uartSendString(" mm/s\r\n");

    /* Measured Suitcase Length / Diameter */
    uartSendString("Bag Length   : ");
    uartSendNumber(bagLength);
    uartSendString(" mm\r\n");

    /* Luggage Sizing Category */
    uartSendString("Category     : ");
    uartSendString(catStr);
    uartSendString("\r\n");

    /* Jam / Alert Status */
    uartSendString("Alert Status : ");
    if (alert == 1) {
        if (inTransit > 0) {
            uartSendString(">> JAM DETECTED (Bag Stalled)! <<\r\n");
        } else {
            uartSendString(">> CONVEYOR STOPPED / IDLE <<\r\n");
        }
    } else {
        uartSendString("NORMAL (Running)\r\n");
    }

    /* Wrong Direction Security Violations */
    uartSendString("Violations   : ");
    uartSendNumber(violations);
    uartSendString("\r\n");
    uartSendString("==============================\r\n");
}

/* Fast ADC read (~12 us per channel, non-blocking) */
static uint16_t adcRead(uint8_t channel)
{
    ADC1->SQR3 = channel;
    ADC1->CR2 |= (1U << 30);            // Start conversion (SWSTART)
    while (!(ADC1->SR & (1U << 1))) {}  // Wait for EOC
    return (uint16_t)ADC1->DR;
}

/*===========================================================================
  MAIN
 ===========================================================================*/
int main(void)
{
    uint32_t nextSend = 0;
    uint32_t dt = 0;

    /* 1. Clocks for GPIOA, GPIOB, GPIOC, ADC1, USART2, SYSCFG */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= (1U << 17);                         // USART2 Clock Enable
    RCC->APB2ENR |= (1U << 8) | (1U << 14);             // ADC1, SYSCFG

    /* 2. Configure USART2 Pins: PA2 (TX) and PA3 (RX) -> AF7
          - High Speed (10) for crisp edges
          - Pull-Up (01) so line idles solidly HIGH (no floating false start bits) */
    GPIOA->MODER   &= ~((3U << (2 * 2)) | (3U << (3 * 2)));
    GPIOA->MODER   |=  ((2U << (2 * 2)) | (2U << (3 * 2))); // Mode 10 = Alternate Function
    GPIOA->AFR[0]  &= ~((0xFU << (2 * 4)) | (0xFU << (3 * 4)));
    GPIOA->AFR[0]  |=  ((7U << (2 * 4)) | (7U << (3 * 4)));  // AF7 = USART2
    GPIOA->OSPEEDR |=  ((3U << (2 * 2)) | (3U << (3 * 2)));  // Very High Speed
    GPIOA->PUPDR   &= ~((3U << (2 * 2)) | (3U << (3 * 2)));
    GPIOA->PUPDR   |=  ((1U << (2 * 2)) | (1U << (3 * 2)));  // Pull-Up

    /* 3. Configure USART2 Peripheral: 9600 Baud @ 16 MHz HSI, 8N1
          DIV = 16,000,000 / (16 * 9600) = 104.1875 -> 0x0683
          CR1: UE (bit 13), TE (bit 3), RE (bit 2) */
    USART2->CR1 = 0;                                         // Reset CR1 first
    USART2->CR2 = 0;                                         // 1 Stop bit
    USART2->CR3 = 0;                                         // No flow control
    USART2->BRR = 0x0683;                                    // 9600 baud
    USART2->CR1 = (1U << 13) | (1U << 3) | (1U << 2);        // UE, TE, RE Enable

    /* 4. Buzzer Output on PA5 (Open-Drain, No Pull, Medium Speed)
       - In Open-Drain mode, writing 0 (RESET) pulls pin hard to GND -> BUZZER ON
       - In Open-Drain mode, writing 1 (SET) floats pin completely   -> BUZZER OFF */
    GPIOA->MODER   &= ~(3U << (5 * 2)); 
    GPIOA->MODER   |=  (1U << (5 * 2));                 // Mode 01 = Output
    GPIOA->OTYPER  |=  (1U << 5);                       // Output type: 1 = Open-Drain
    GPIOA->PUPDR   &= ~(3U << (5 * 2));                 // 00 = No pull-up / pull-down
    GPIOA->OSPEEDR &= ~(3U << (5 * 2));
    GPIOA->OSPEEDR |=  (1U << (5 * 2));                 // Medium speed
    buzzerOff();                                        // Floating / Silent initially

    /* 5. Analog Inputs: PA1 (Sensor 1) and PA4 (Sensor 2) */
    GPIOA->MODER |= (3U << (1 * 2)) | (3U << (4 * 2));

    /* 6. Reset Push Button: PB5 Input with Internal Pull-Up */
    GPIOB->MODER &= ~(3U << (5 * 2));
    GPIOB->PUPDR &= ~(3U << (5 * 2));
    GPIOB->PUPDR |=  (1U << (5 * 2)); // Pull-Up

    /* 7. On-board Blue LED: PC13 Output */
    GPIOC->MODER &= ~(3U << (13 * 2));
    GPIOC->MODER |=  (1U << (13 * 2));
    GPIOC->ODR   |=  (1U << 13); // LED OFF initially

    /* 8. EXTI5 on PB5 (Reset Button Interrupt) */
    SYSCFG->EXTICR[1] &= ~(0xFU << 4);
    SYSCFG->EXTICR[1] |=  (0x1U << 4); // Port B
    EXTI->IMR  |= (1U << 5);
    EXTI->FTSR |= (1U << 5);           // Falling edge
    NVIC_EnableIRQ(EXTI9_5_IRQn);

    /* 9. ADC1 Setup */
    ADC->CCR    &= ~(3U << 16);
    ADC->CCR    |=  (1U << 16);         // Prescaler PCLK2 / 4
    ADC1->CR1    = 0;
    ADC1->CR2    = (1U << 0);           // ADON = 1
    ADC1->SQR1   = 0;                   // 1 conversion
    ADC1->SMPR2  = (4U << 3) | (4U << 12); // 84 cycles sample time

    /* 10. SysTick 1 ms Stopwatch */
    SysTick->LOAD = 16000 - 1;          // 1 ms
    SysTick->VAL  = 0;
    SysTick->CTRL = 7;                  // Enable SysTick with interrupt

    /* Small startup delay to allow Bluetooth module & voltage rails to stabilize */
    for (volatile int d = 0; d < 500000; d++);

    /* Startup: 2 quick confirmation blinks & buzzer chirps */
    for (int i = 0; i < 2; i++) {
        GPIOC->ODR &= ~(1U << 13); // Blue LED ON
        buzzerOn();                // Buzzer ON
        for (volatile int d = 0; d < 200000; d++);
        GPIOC->ODR |=  (1U << 13); // Blue LED OFF
        buzzerOff();               // Buzzer OFF
        for (volatile int d = 0; d < 200000; d++);
    }

    lastEventTime = msTicks;
    alert = 0;
    buzzerOff();
    nextSend = msTicks + 2000; // First telemetry after 2 seconds

    uartSendString("\r\n====================================\r\n");
    uartSendString(" SMART BAGGAGE SYSTEM READY (9600)\r\n");
    uartSendString("====================================\r\n");

    /*=======================================================================
      HIGH-SPEED SENSING LOOP (Zero delays - runs > 35,000 times / second)
     =======================================================================*/
    while (1)
    {
        uint16_t s1 = adcRead(1); // Read PA1 (Sensor 1)
        uint16_t s2 = adcRead(4); // Read PA4 (Sensor 2)

        /*-------------------------------------------------------------------
          SENSOR 1 (ENTRY) : DETECT BALLS ENTERING THE CHUTE
         -------------------------------------------------------------------*/
        if (s1Blocked == 0)
        {
            if (s1 < TH_ON)
            {
                /* LEADING EDGE: Ball entered Sensor 1 */
                s1Blocked = 1;
                inTransit++;          // Count +1 ball between Sensor 1 and Sensor 2
                s1Time      = msTicks;
                lastS1Entry = msTicks;
                GPIOC->ODR &= ~(1U << 13); // Turn ON on-board Blue LED
                newEvent = 1;         // Send live update to phone immediately
            }
        }
        else
        {
            if (s1 > TH_OFF)
            {
                /* TRAILING EDGE: Ball cleared Sensor 1 */
                s1Blocked = 0;
            }
        }

        /*-------------------------------------------------------------------
          SENSOR 2 (EXIT) : DETECT BALLS EXITING THE CHUTE
         -------------------------------------------------------------------*/
        if (s2Blocked == 0)
        {
            if (s2 < TH_ON)
            {
                /* LEADING EDGE: Ball arrived at Sensor 2 */
                s2Blocked = 1;
                s2entry   = msTicks;

                if (inTransit > 0)
                {
                    /* This is a ball that entered Sensor 1! */
                    inTransit--;      // One less ball between the sensors
                    total++;          // INCREMENT TOTAL COUNT (Even if it was delayed!)
                    lastEventTime = msTicks;

                    dt = msTicks - s1Time;

                    /* Calculate speed only if transit was smooth (< 2 seconds) */
                    if (dt >= MIN_DT_MS && dt <= PAIR_MS)
                    {
                        speed = (BELT_GAP_MM * 1000U) / dt;
                    }

                    /* If all stalled balls have exited, CLEAR THE JAM ALERT! */
                    if (inTransit == 0)
                    {
                        alert = 0;
                    }

                    /* Short 120ms confirmation pulse (Chirp) */
                    buzzerOn();
                    buzzerEndTime = msTicks + 120;

                    measuringLength = 1; // Wait for trailing edge to measure ball diameter
                }
                else if (total > 0)
                {
                    /* Sensor 2 triggered with zero balls entered -> Wrong-way violation! */
                    violations++;
                    newEvent = 1;
                }
            }
        }
        else
        {
            if (s2 > TH_OFF)
            {
                /* TRAILING EDGE: Ball finished passing Sensor 2 */
                s2Blocked = 0;

                if (measuringLength)
                {
                    measuringLength = 0;
                    dt = msTicks - s2entry;

                    if (speed > 0)
                    {
                        bagLength = (speed * dt) / 1000U;

                        /* Categorize luggage size */
                        if (bagLength < LEN_SMALL)       category = 0; // Carry-on (SM)
                        else if (bagLength <= LEN_LARGE) category = 1; // Standard (ST)
                        else                             category = 2; // Oversized (LG)
                    }

                    newEvent = 1; // Send new length and total to phone
                }

                if (inTransit == 0)
                {
                    GPIOC->ODR |= (1U << 13); // Turn OFF on-board Blue LED
                }
            }
        }

        /*-------------------------------------------------------------------
          JAM DETECTION (TRANSIT DELAY)
          If any ball entered Sensor 1 but hasn't reached Sensor 2 in 2.0s
         -------------------------------------------------------------------*/
        if (inTransit > 0)
        {
            if ((msTicks - lastS1Entry) > PAIR_MS)
            {
                /* Declare Jam: Ball is stalled between Sensor 1 & Sensor 2 */
                if (alert == 0)
                {
                    alert = 1;
                    newEvent = 1; // Immediately notify phone
                }
            }
        }

        /*-------------------------------------------------------------------
          INACTIVITY JAM (CONVEYOR STOPPED)
          If 20 seconds pass with zero movement after system started
         -------------------------------------------------------------------*/
        if (total > 0 && inTransit == 0 && alert == 0 && (msTicks - lastEventTime) >= TIMEOUT_MS)
        {
            alert = 1;
        }

        /*-------------------------------------------------------------------
          CONFIRMATION PULSE & JAM ALARM TIMING
         -------------------------------------------------------------------*/
        /* Turn off short 120ms pass confirmation pulse */
        if (buzzerEndTime > 0 && msTicks >= buzzerEndTime)
        {
            buzzerEndTime = 0;
            if (alert == 0) {
                buzzerOff();
            }
        }

        /* Continuous Blinking / Beeping while Alert/Jam is active */
        if (alert == 1)
        {
            if ((msTicks % 1000U) < 200U) {
                buzzerOn();                // Alarm ON (200 ms)
                GPIOC->ODR &= ~(1U << 13); // Flash on-board LED
            } else {
                buzzerOff();               // Alarm OFF (800 ms)
                GPIOC->ODR |=  (1U << 13);
            }
        }
        else if (buzzerEndTime == 0)
        {
            buzzerOff(); // Keep buzzer completely OFF when normal
        }

        /* Soft heartbeat blink on on-board LED when completely idle */
        if (inTransit == 0 && alert == 0)
        {
            if ((msTicks % 500U) == 0U) {
                GPIOC->ODR ^= (1U << 13);
            }
        }

        /*-------------------------------------------------------------------
          RESET PUSH BUTTON (EXTI5 on PB5)
         -------------------------------------------------------------------*/
        if (resetFlag)
        {
            resetFlag     = 0;
            total         = 0;
            inTransit     = 0;
            violations    = 0;
            speed         = 0;
            bagLength     = 0;
            category      = 3;
            alert         = 0;
            s1Blocked     = 0;
            s2Blocked     = 0;
            lastEventTime = msTicks;
            buzzerOff();
            uartSendString("\r\n>>> SYSTEM RESET: All Counters Cleared <<<\r\n");
            sendData();
        }

        /*-------------------------------------------------------------------
          BLUETOOTH TELEMETRY TRANSMISSION
         -------------------------------------------------------------------*/
        if (newEvent)
        {
            newEvent = 0;
            sendData();
        }
        else if (msTicks >= nextSend)
        {
            nextSend = msTicks + SEND_MS;
            sendData();
        }
    }
}

/*===========================================================================
  EXTI9_5 INTERRUPT (PB5 Reset Button)
 ===========================================================================*/
void EXTI9_5_IRQHandler(void)
{
    static uint32_t lastPress = 0;
    if (EXTI->PR & (1U << 5)) {
        EXTI->PR |= (1U << 5); // Clear pending bit

        /* Debounce: 200 ms */
        if ((msTicks - lastPress) > 200U) {
            lastPress = msTicks;
            resetFlag = 1;
        }
    }
}