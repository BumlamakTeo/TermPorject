#include <stdint.h>
#include "tm4c123gh6pm.h"
#include <stdio.h>

// Function prototypes
void comparator_init(void);
void COMP0_Handler(void);
void GPIO_ADC_INIT(void);
void GPIOE_Handler(void);
void ADC_measurement(uint32_t *lm35_mV, uint32_t *threshold_mV);
void gpio_init(void);

volatile int adc_ready = 0; // Global flag to signal ADC completion

int main(void) {
		NVIC_SYS_CTRL_R |= 0x04;
    // Initialize peripherals
    comparator_init();
		GPIO_ADC_INIT();
		gpio_init();
    // Assume button initialization happens here
		__asm("CPSIE I"); // Enable global interrupts
		COMP_ACMIS_R = 0x01;
		__asm("WFI");
		uint32_t lm35_mV = 0;
    uint32_t threshold_mV = 0;
    while (1) {
			if (adc_ready) {
            ADC_measurement(&lm35_mV, &threshold_mV);  // Take the ADC measurement
        }
    }
}

// Analog comparator interrupt handler
void COMP0_Handler(void) {
    COMP_ACMIS_R = 0x01; // Clear the interrupt flag
		adc_ready = 1;       // Signal the main program to perform an action
    // Take a temperature reading from BMP280 or perform other actions
}

// GPIO Port E interrupt handler
void GPIOE_Handler(void) {
    GPIO_PORTE_ICR_R |= 0x02;   // Clear interrupt flag for PE1
    COMP_ACMIS_R = 0x01;
		adc_ready = 0;
		__asm("WFI");								// Enter deep sleep
}

// Analog comparator initialization
void comparator_init(void) {
    SYSCTL_RCGCACMP_R |= 0x01; // Enable comparator clock
    SYSCTL_RCGCGPIO_R |= 0x04; // Enable clock for GPIO Port C

    GPIO_PORTC_AFSEL_R |= 0xC0; // Enable alternate function for PC4 (AIN0) and PC5 (AIN1)
    GPIO_PORTC_DEN_R &= ~0xC0;  // Disable digital functionality
    GPIO_PORTC_AMSEL_R |= 0xC0; // Enable analog functionality

    COMP_ACREFCTL_R = 0;         // Disable internal reference, use external (trimpot)
		COMP_ACCTL0_R &= 0x00;			
    COMP_ACCTL0_R |= 0xAD8;  // VIN+ = AIN0, VIN- = AIN1, interrupt on rising edge

    COMP_ACMIS_R = 0;           // Clear any pending comparator interrupts
    COMP_ACINTEN_R |= 0x01;     // Enable comparator interrupt
		NVIC_PRI6_R &= 0xFFFF00FF;
		NVIC_PRI6_R |= 0x00004000;
		NVIC_EN0_R |= (1 << 25);
	
    //NVIC_EN0_R |= (1 << 25);    // Enable comparator interrupt in NVIC
}

void GPIO_ADC_INIT(void) {
    SYSCTL_RCGCGPIO_R |= 0x10; // Enable clock for Port E
    SYSCTL_RCGCADC_R |= 0x3;   // Enable clocks for ADC0 and ADC1

    while ((SYSCTL_PRADC_R & 0x3) != 0x3); // Wait for ADC0 and ADC1 to stabilize

    // Configure PE3 (ADC0, AIN0) for LM35
    GPIO_PORTE_AFSEL_R |= 0x08; // Alternate function for PE3
    GPIO_PORTE_DEN_R &= ~0x08;  // Disable digital functionality
    GPIO_PORTE_AMSEL_R |= 0x08; // Enable analog functionality

    // Configure PE2 (ADC1, AIN1) for Threshold
    GPIO_PORTE_AFSEL_R |= 0x04; // Alternate function for PE2
    GPIO_PORTE_DEN_R &= ~0x04;  // Disable digital functionality
    GPIO_PORTE_AMSEL_R |= 0x04; // Enable analog functionality

    // ADC0 Configuration (LM35 on PE3, AIN0)
    ADC0_ACTSS_R &= ~0x08;       // Disable SS3 during configuration
    ADC0_EMUX_R &= ~0xF000;      // Set software trigger for SS3
    ADC0_SSMUX3_R = 0x00;        // Select AIN0 (PE3)
    ADC0_SSCTL3_R = 0x06;        // End of sequence, enable interrupt
    ADC0_PC_R |= 0x1;            // Configure for 125K samples/sec
    ADC0_ACTSS_R |= 0x08;        // Re-enable SS3

    // ADC1 Configuration (Threshold on PE2, AIN1)
    ADC1_ACTSS_R &= ~0x08;       // Disable SS3 during configuration
    ADC1_EMUX_R &= ~0xF000;      // Set software trigger for SS3
    ADC1_SSMUX3_R = 0x01;        // Select AIN1 (PE2)
    ADC1_SSCTL3_R = 0x06;        // End of sequence, enable interrupt
    ADC1_PC_R |= 0x1;            // Configure for 125K samples/sec
    ADC1_ACTSS_R |= 0x08;        // Re-enable SS3
}


// Initialize GPIO for PE1 as button input
void gpio_init(void) {
    SYSCTL_RCGCGPIO_R |= 0x10; // Enable clock for Port E
    while ((SYSCTL_PRGPIO_R & 0x10) == 0); // Wait until Port E is ready

    GPIO_PORTE_DIR_R &= ~0x02;  // Set PE1 as input
    GPIO_PORTE_DEN_R |= 0x02;   // Enable digital functionality on PE1
    GPIO_PORTE_PUR_R |= 0x02;   // Enable pull-up resistor on PE1

    GPIO_PORTE_IS_R &= ~0x02;   // Make PE1 edge-sensitive
    GPIO_PORTE_IBE_R &= ~0x02;  // Trigger on a single edge
    GPIO_PORTE_IEV_R &= ~0x02;  // Trigger on falling edge
    GPIO_PORTE_ICR_R |= 0x02;   // Clear any prior interrupt on PE1
    GPIO_PORTE_IM_R |= 0x02;    // Enable interrupt for PE1

    NVIC_EN0_R |= (1 << 4);     // Enable interrupt in NVIC for Port E
}

void ADC_measurement(uint32_t *lm35_mV, uint32_t *threshold_mV){
    int16_t result;
    float lm35_ema = 0.0; // Exponential Moving Average for LM35
    float threshold_ema = 0.0; // Exponential Moving Average for Threshold
    const float alpha = 0.1f;  // Smoothing factor for EMA
    const uint32_t offset_correction = 5; // Offset correction in mV

    // Measure LM35 (ADC0, PE3) 1000 times
    for (int i = 0; i < 1000; i++) {
        ADC0_PSSI_R |= 0x08;              // Start conversion
        while ((ADC0_RIS_R & 0x08) == 0); // Wait for conversion to complete
        result = ADC0_SSFIFO3_R & 0x0FFF; // 12-bit result
        ADC0_ISC_R |= 0x08;               // Clear interrupt flag
        float mV = (result * 3300.0f) / 4095.0f; // Convert ADC value to millivolts (3300mV for 3.3V ref)
        lm35_ema = (alpha * mV) + ((1 - alpha) * lm35_ema); // Apply EMA
    }
    *lm35_mV = (uint32_t)(lm35_ema + offset_correction); // Add offset correction and output mV

    // Measure Threshold (ADC1, PE2) 1000 times
    for (int i = 0; i < 1000; i++) {
        ADC1_PSSI_R |= 0x08;              // Start conversion
        while ((ADC1_RIS_R & 0x08) == 0); // Wait for conversion to complete
        result = ADC1_SSFIFO3_R & 0x0FFF; // 12-bit result
        ADC1_ISC_R |= 0x08;               // Clear interrupt flag
        float mV = (result * 3300.0f) / 4095.0f; // Convert ADC value to millivolts
        threshold_ema = (alpha * mV) + ((1 - alpha) * threshold_ema); // Apply EMA
    }
    *threshold_mV = (uint32_t)(threshold_ema + offset_correction); // Add offset correction and output mV
}
