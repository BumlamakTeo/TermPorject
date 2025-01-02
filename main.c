#include <stdint.h>
#include "tm4c123gh6pm.h"
#include <stdio.h>

// Function prototypes
void comparator_init(void);
void COMP0_Handler(void);
void Button_Interrupt_Handler(void);
void GPIO_ADC_INIT(void);
float ADC_measurement(void);

volatile int adc_ready = 0; // Global flag to signal ADC completion

int main(void) {
		NVIC_SYS_CTRL_R |= 0x04;
    // Initialize peripherals
    comparator_init();
		GPIO_ADC_INIT();
    // Assume button initialization happens here
		__asm("CPSIE I"); // Enable global interrupts
		COMP_ACMIS_R = 0x01;
		__asm("WFI");
    while (1) {
			if (adc_ready) {
            ADC_measurement(); // Take the ADC measurement
        }
    }
}

// Analog comparator interrupt handler
void COMP0_Handler(void) {
    COMP_ACMIS_R = 0x01; // Clear the interrupt flag
		adc_ready = 1;       // Signal the main program to perform an action
    // Take a temperature reading from BMP280 or perform other actions
}

// Button interrupt handler
void Button_Interrupt_Handler(void) {
    // Clear the button interrupt flag (depends on the specific GPIO setup)
	
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

void GPIO_ADC_INIT(void){
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOB; //enable port B
		SYSCTL_RCGCADC_R |= 0x1;
		while(SYSCTL_RCGCADC_R != 0x01){};
    GPIO_PORTB_AFSEL_R |= 0x10; // alternate function for PB4
		GPIO_PORTB_DEN_R &= ~0x10;
    GPIO_PORTB_DIR_R &= ~0x10;
    GPIO_PORTB_AMSEL_R |= 0x10;
		GPIO_PORTB_ADCCTL_R = 0x0;
		ADC0_ACTSS_R &= ~0x8;
    ADC0_EMUX_R &= ~0xF000;
    ADC0_SSMUX3_R |= 0xA;
    ADC0_SSCTL3_R |= 0x6;
		ADC0_PC_R |= 0x1;
		ADC0_ACTSS_R |= 0x8;
}

float ADC_measurement(void) {
    int16_t result;
    float voltage;
    float total = 0;
    float out;
    int sample_count = 0;

    while (sample_count < 1000) {
        ADC0_PSSI_R |= 0x8;          // Start ADC conversion
        while ((ADC0_RIS_R & 8) == 0); // Wait for conversion to complete
        result = ADC0_SSFIFO3_R & 0x0FFF; // Ensure 12-bit result
        voltage = (result / 4095.0f) * 3.3f;
        ADC0_ISC_R |= 0x8;           // Clear ADC interrupt flag
        total += voltage;
        sample_count++;
    }
    out = total / 1000; // Calculate average voltage
    total = 0;
		return out;
}
