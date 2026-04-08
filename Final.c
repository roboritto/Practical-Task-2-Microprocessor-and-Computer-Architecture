#include "stm32f4xx.h"
#include <stdio.h>
#include "i2c_oled.h"

// --- GLOBALS (TASKS 2A, 2B, 2C) ---
volatile uint8_t hours = 12, minutes = 59, seconds = 50, update_display = 1; 
volatile uint8_t sw_minutes = 0, sw_seconds = 0, sw_running = 0;
uint8_t display_mode = 0; // 0=Clock, 1=Stopwatch, 2=Dashboard, 3=Wheel Speed, 4=ADC
uint8_t mode_12hr = 0, colon_state = 1;  

#define LED_READY 13  
#define LED_FAULT 14  
volatile uint8_t system_status = 0; 

#define TEETH 20
volatile uint32_t pulse_count = 0, pulse_frequency = 0, wheel_rpm = 0;       
uint16_t test_rpms[] = {10, 500, 3000}; 
uint8_t current_test_idx = 0;

// --- TASK 2D GLOBALS (ADC) ---
uint16_t raw_adc_value = 0;
float v_adc = 0.0f;
float v_battery = 0.0f;

// --- TASK 2E GLOBALS (VEHICLE SPEED) ---
#define PI 3.14159f
#define WHEEL_RADIUS 0.30f // meters

float angular_velocity = 0.0f;
float linear_velocity_ms = 0.0f;
float vehicle_speed_kmh = 0.0f;

// --- TASK 2F GLOBALS (FAIL-SAFE WATCHDOG) ---
volatile uint8_t seconds_without_pulse = 0; 

// For acceleration tracking
float prev_velocity_ms = 0.0f; 
float acceleration = 0.0f;

// --- INTERRUPTS ---
void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF; 
        colon_state = !colon_state; 
        
        pulse_frequency = pulse_count; 
        pulse_count = 0;               
        wheel_rpm = (pulse_frequency * 60) / TEETH; 
			
				// --- NEW TASK 2E: SPEED & ACCELERATION MATH ---
        // 1. Angular Velocity (rad/s)
        angular_velocity = (2.0f * PI * (float)wheel_rpm) / 60.0f;
        
        // 2. Linear Velocity (m/s)
        linear_velocity_ms = angular_velocity * WHEEL_RADIUS;
        
        // 3. Vehicle Speed (km/h)
        vehicle_speed_kmh = linear_velocity_ms * 3.6f;
        
        // 4. Acceleration (m/s^2). Since this timer runs every 1s, dt = 1
        acceleration = linear_velocity_ms - prev_velocity_ms;
        prev_velocity_ms = linear_velocity_ms; // Save current speed for next second
        
        // 5. Overspeed Warning Logic
        // If speed > 80 km/h, force LED2 (Fault) ON.
        if (vehicle_speed_kmh > 80.0f) {
            GPIOB->ODR |= (1 << LED_FAULT); 
            system_status = 2; // Sync our software status variable
        }
				
				// --- TASK 2F: FAIL-SAFE WATCHDOG LOGIC ---
        if (pulse_frequency == 0) {
            seconds_without_pulse++; // Increment if no pulses arrived
        } else {
            seconds_without_pulse = 0; // Sensor is alive, reset watchdog
        }
        
        // --- AUTOMATED FAULT DETECTION ---
        if (seconds_without_pulse >= 2) {
            // CRITICAL FAULT: Signal Lost for > 2 seconds
            system_status = 2;
            GPIOB->ODR |= (1 << LED_FAULT);   // Fault LED ON
            GPIOB->ODR &= ~(1 << LED_READY);  // Ready LED OFF
            wheel_rpm = 0;
            vehicle_speed_kmh = 0.0f;
            acceleration = 0.0f;
        } else if (vehicle_speed_kmh > 80.0f) {
            // WARNING: Overspeed
            system_status = 1;
            GPIOB->ODR |= (1 << LED_FAULT);   // Fault LED ON
            GPIOB->ODR |= (1 << LED_READY);   // Ready LED ON (system is still reading)
        } else {
            // NORMAL OPERATION
            system_status = 0;
            GPIOB->ODR &= ~(1 << LED_FAULT);  // Fault LED OFF
            GPIOB->ODR |= (1 << LED_READY);   // Ready LED ON
        }
        
        seconds++;
        if (seconds >= 60) { seconds = 0; minutes++; if (minutes >= 60) { minutes = 0; hours++; if (hours >= 24) hours = 0; } }
        
        if (sw_running) { sw_seconds++; if (sw_seconds >= 60) { sw_seconds = 0; sw_minutes++; if (sw_minutes >= 99) sw_minutes = 0; } }
        
        update_display = 1; 
    }
}

void TIM3_IRQHandler(void) {
    if (TIM3->SR & TIM_SR_UIF) { TIM3->SR &= ~TIM_SR_UIF; pulse_count++; }
}

// --- INITIALIZATION FUNCTIONS ---
void Timer2_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 15999; TIM2->ARR = 999; TIM2->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM2_IRQn); TIM2->CR1 |= TIM_CR1_CEN;
}

void TIM3_PulseGenerator_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->PSC = 15999; TIM3->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM3_IRQn); TIM3->CR1 |= TIM_CR1_CEN;
}

void Set_Simulated_Speed(uint16_t target_rpm) {
    if (target_rpm == 0) {
        TIM3->CR1 &= ~TIM_CR1_CEN; // Disable timer to completely stop pulses
    } else {
        float target_freq = ((float)target_rpm * TEETH) / 60.0f;
        TIM3->ARR = (uint32_t)((1000.0f / target_freq) - 1); 
        TIM3->EGR |= TIM_EGR_UG; 
        TIM3->CR1 |= TIM_CR1_CEN;  // Ensure timer is enabled
    }
}

void Buttons_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
    GPIOC->PUPDR |= (1 << 26); 
    GPIOA->PUPDR |= (1 << 20) | (1 << 22) | (1 << 24);
}

void LED_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; 
    GPIOB->MODER |= (1 << (LED_READY * 2)) | (1 << (LED_FAULT * 2));
    GPIOB->MODER &= ~((2 << (LED_READY * 2)) | (2 << (LED_FAULT * 2))); 
    GPIOB->ODR &= ~((1 << LED_READY) | (1 << LED_FAULT)); 
}

// --- TASK 2D: ADC INITIALIZATION (PA0) ---
void ADC1_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    
    GPIOA->MODER |= (3 << 0); // PA0 to Analog Mode
    
    ADC1->CR2 &= ~ADC_CR2_CONT; // Single conversion mode
    ADC1->SQR3 = 0;             // Channel 0
    ADC1->CR2 |= ADC_CR2_ADON;  // Enable ADC
}

uint16_t ADC1_Read(void) {
    ADC1->CR2 |= ADC_CR2_SWSTART;           // Start conversion
    while(!(ADC1->SR & ADC_SR_EOC));        // Wait for End Of Conversion
    return ADC1->DR;                        // Return 12-bit data
}

void delay_ms(int ms) {int i = 0; for(i = 0; i < ms * 4000; i++) { __NOP(); } }

// --- MAIN ROUTINE ---
int main(void) {
		char oledBuffer[21];
		char colon = colon_state ? ':' : ' ';
	
    SystemInit(); 
    Buttons_Init();
    LED_Init();
    ADC1_Init();
    OLED_Init(); 
    
    Timer2_Init();
    TIM3_PulseGenerator_Init();
    Set_Simulated_Speed(test_rpms[current_test_idx]);
    
    GPIOB->ODR |= (1 << LED_READY); 

    while (1) {
        // --- TASK 2D: READ ADC CONTINUOUSLY ---
        raw_adc_value = ADC1_Read();
        v_adc = ((float)raw_adc_value * 3.3f) / 4095.0f; 
        
        // NEW MATH: Reconstruct input voltage with R1=1k, R2=220
        v_battery = v_adc * 5.545f;
        
        // --- BUTTON HANDLING ---
        if (!(GPIOC->IDR & (1 << 13))) { // Cycle Modes 0-4
            display_mode++; if (display_mode > 4) display_mode = 0; 
            OLED_Clear(); update_display = 1; delay_ms(300); 
        }
        
        if (!(GPIOA->IDR & (1 << 10))) { 
            if (display_mode == 0) { hours++; if (hours >= 24) hours = 0; } 
            else if (display_mode == 1) { sw_running = !sw_running; } 
            else if (display_mode == 2) {
                system_status++; if (system_status > 2) system_status = 0;
                if (system_status == 0) { GPIOB->ODR |= (1<<LED_READY); GPIOB->ODR &= ~(1<<LED_FAULT); } 
                else { GPIOB->ODR &= ~(1<<LED_READY); GPIOB->ODR |= (1<<LED_FAULT); }
            } 
            else if (display_mode == 3) {
                current_test_idx++; if (current_test_idx > 2) current_test_idx = 0;
                Set_Simulated_Speed(test_rpms[current_test_idx]);
            }
            update_display = 1; delay_ms(300);
        }
        
        if (!(GPIOA->IDR & (1 << 11))) { 
            if (display_mode == 0) { minutes++; if (minutes >= 60) minutes = 0; } 
            else if (display_mode == 1) { if (!sw_running) { sw_minutes = 0; sw_seconds = 0; } }
            update_display = 1; delay_ms(300);
        }

        if (!(GPIOA->IDR & (1 << 12))) { 
            if (display_mode == 0) { mode_12hr = !mode_12hr; } update_display = 1; delay_ms(300);
        }

        // --- DISPLAY UPDATING ---
        if (update_display) {
            update_display = 0;  
            OLED_SetCursor(0, 0); 
            
            if (display_mode == 0) {
                if (mode_12hr) {
                    uint8_t display_hr = (hours % 12 == 0) ? 12 : (hours % 12);
                    sprintf(oledBuffer, "Time: %02d%c%02d %s   ", display_hr, colon, minutes, (hours >= 12) ? "PM" : "AM");
                } else {
                    sprintf(oledBuffer, "Time: %02d%c%02d:%02d   ", hours, colon, minutes, seconds);
                }
                OLED_PrintString(oledBuffer);
            } else if (display_mode == 1) {
                sprintf(oledBuffer, "Stopwatch: %02d%c%02d   ", sw_minutes, colon, sw_seconds);
                OLED_PrintString(oledBuffer);
            } else if (display_mode == 2) {
                // Task 2F: Integrated DBW Dashboard
                if (system_status == 0) {
                    OLED_PrintString("STATUS: NORMAL    ");
                    OLED_SetCursor(1, 0);
                    sprintf(oledBuffer, "RPM: %04d         ", wheel_rpm);
                    OLED_PrintString(oledBuffer);
                    OLED_SetCursor(2, 0);
                    sprintf(oledBuffer, "SPD: %3.0f km/h    ", vehicle_speed_kmh);
                    OLED_PrintString(oledBuffer);
                } else if (system_status == 1) {
                    OLED_PrintString("STATUS: OVERSPEED ");
                    OLED_SetCursor(1, 0);
                    sprintf(oledBuffer, "RPM: %04d         ", wheel_rpm);
                    OLED_PrintString(oledBuffer);
                    OLED_SetCursor(2, 0);
                    sprintf(oledBuffer, "SPD: %3.0f km/h    ", vehicle_speed_kmh);
                    OLED_PrintString(oledBuffer);
                } else if (system_status == 2) {
                    OLED_PrintString("STATUS: SENS FAULT");
                    OLED_SetCursor(1, 0);
                    OLED_PrintString("RPM: 0000         ");
                    OLED_SetCursor(2, 0);
                    OLED_PrintString("NO PULSE DETECTED ");
                }
            } else if (display_mode == 3) {
                // Task 2E: Display RPM and km/h on Row 1
                sprintf(oledBuffer, "RPM:%04d  %3.0fkm/h", wheel_rpm, vehicle_speed_kmh); 
                OLED_PrintString(oledBuffer);
                
                // Display Acceleration on Row 2
                OLED_SetCursor(2, 0); 
                sprintf(oledBuffer, "Acc: %5.2f m/s2 ", acceleration); 
                OLED_PrintString(oledBuffer);
            } else if (display_mode == 4) {
                // Task 2D: Voltage Output [cite: 230-233]
                sprintf(oledBuffer, "ADC = %04d        ", raw_adc_value);
                OLED_PrintString(oledBuffer);
                
                OLED_SetCursor(1, 0); 
                sprintf(oledBuffer, "Volt = %4.1f V      ", v_battery);
                OLED_PrintString(oledBuffer);
                
                OLED_SetCursor(2, 0); 
                if (v_battery >= 7.0f) { OLED_PrintString("Status = OK       "); } 
                else { OLED_PrintString("Status = Fault    "); }
            }
        }
    }
}
