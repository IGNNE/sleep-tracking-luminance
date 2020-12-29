#include "custom_adc_lib.h"

// ADC constants/variables
/// Parameter für den ADC, zur besseren Lesbarkeit
#define OVERSAMPLING_16X ADC_CFGR2_OVSR_1 | ADC_CFGR2_OVSR_0
/// Parameter für den ADC, zur besseren Lesbarkeit
#define SHIFT_4B ADC_CFGR2_OVSS_2
/// Adresse des Kalibrierwerts für VREFINT
#define VREFINT_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FF80078))

/// Praktisches Makro, um den ADC nur dann zu stopppen, wenn er läuft (sonst kommt es zu Problemen)
#define STOP_ADC_IF_RUNNING() ADC1->CR |= (ADC1->CR & ADC_CR_ADSTART) << (ADC_CR_ADSTP_Pos - ADC_CR_ADSTART_Pos);

/// Speichert den Zeiger auf die Callback-Funktion
static eoc_callback_t eoc_callback = NULL;

/**
 * Interner ADC-Interrupt-Handler, der nur den bereitgestellten Callback aufruft.
 */
void ADC1_COMP_IRQHandler() {
	eoc_callback(ADC1->DR);
}

void enable_EOC_interrupt() {
	STOP_ADC_IF_RUNNING() // make sure that there is no conversion in the background
	NVIC_EnableIRQ(ADC1_COMP_IRQn);
	ADC1->IER |= ADC_IER_EOCIE;
}

void enable_EOC_interrupt_timer(uint16_t time_ms) {
	STOP_ADC_IF_RUNNING() // make sure that there is no conversion in the background
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; // power on timer
	TIM6->PSC = 8000; // timer should tick in ms
	TIM6->ARR = time_ms; // set actual time value as
	TIM6->CR2 |= TIM_CR2_MMS_1; // should be '010' for use with ADC

	NVIC_EnableIRQ(ADC1_COMP_IRQn);
	ADC1->IER |= ADC_IER_EOCIE;
	ADC1->CFGR1 |= ADC_CFGR1_EXTEN_0; // rising edge trigger
	ADC1->CFGR1 &= ~ADC_CFGR1_EXTSEL; // select TIM6, just to be sure; should be default
	TIM6->EGR |= TIM_EGR_UG; // generate update event
	TIM6->CR1 |= TIM_CR1_CEN; // enable timer
}

void disable_adc_interrupts() {
	STOP_ADC_IF_RUNNING() // make sure that there is no conversion in the background
	NVIC_DisableIRQ(ADC1_COMP_IRQn);
	ADC1->IER &= ~ADC_IER_EOCIE; // disable interrupt
	ADC1->CFGR1 &= ~ADC_CFGR1_EXTEN; // remove timer trigger config (if any)
	TIM6->CR1 &= ~TIM_CR1_CEN; // stop timer
}

uint16_t read_adc_raw_blocking(uint32_t channel) {
	if (ADC1->CHSELR != channel) { // only do this when the channel changes
		ADC1->CHSELR = channel; // select channel
		delay_ms(1); // won't work without this
	}
	ADC1->CR |= ADC_CR_ADSTART; // start conversion
	while (!(ADC1->ISR & ADC_ISR_EOC))
		; // wait until ready, if necessary
	ADC1->CR |= ADC_CR_ADSTP;
	return ADC1->DR;
}

void read_adc_interrupt(uint32_t channel) {
	ADC1->CHSELR = channel; // select channel
	ADC1->CR |= ADC_CR_ADSTART; // start conversion
}

uint16_t read_vdda() {
	uint32_t adc_ier = ADC1->IER; // save IER value
	ADC1->IER = 0; // disable interrupts
	//TIM6->CR1 &= ~TIM_CR1_CEN; // stop timer // TODO does not work yet!
	uint8_t adc_adstart = ADC1->CR & ADC_CR_ADSTART; // save the adstart flag
	STOP_ADC_IF_RUNNING() // make sure that there is no conversion in the background
	uint32_t adc_chselr = ADC1->CHSELR; // save current channel
	uint16_t adc_dr = read_adc_raw_blocking(ADC_CHSELR_CHSEL17);
	ADC1->IER = adc_ier; // restore interrupts
	//TIM6->CR1 |= TIM_CR1_CEN; // restart timer
	ADC1->CHSELR = adc_chselr; // restore channel
	ADC1->CR |= adc_adstart; // restore adstart flag
	delay_ms(1); // for some weird reason, it won't work without this
	return (uint16_t) (3UL * (uint32_t) (*VREFINT_CAL_ADDR) * 1000UL
			/ (uint32_t) adc_dr); // probably paranoid use of ui32
}

void adc_init() {
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN; // enable adc clock
	ADC1->IER = 0; // disable interrupts
	ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN; // disable dma for calibration
	if (ADC1->CR & ADC_CR_ADEN) {
		ADC1->CR |= ADC_CR_ADDIS; // turn off ADC for calibration
		while (ADC1->CR & ADC_CR_ADEN)
			;
	}
	ADC1->CR |= ADC_CR_ADCAL; // enable calibration
	while (!(ADC1->ISR & ADC_ISR_EOCAL))
		; // wait for calibration to finish

	ADC->CCR |= ADC_CCR_PRESC_0 | ADC_CCR_PRESC_1 | ADC_CCR_PRESC_3; // max. prescaler
	ADC1->SMPR = ADC_SMPR_SMPR; // select max. sample time

	ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE; // select HSI16 clock
	ADC1->CFGR2 |= ADC_CFGR2_OVSE; // enable oversampling
	ADC1->CFGR2 |= OVERSAMPLING_16X; // 16x oversampling
	ADC1->CFGR2 |= SHIFT_4B; // 4b shift
	ADC1->CFGR1 &= ~ADC_CFGR1_AUTOFF; // disable auto-off

	ADC1->CFGR1 &= ~ADC_CFGR1_CONT; // single conversion mode
	ADC1->CFGR1 |= ADC_CFGR1_DISCEN;
	ADC1->CFGR1 &= ~ADC_CFGR1_EXTEN; // software trigger
	ADC1->CFGR1 |= ADC_CFGR1_OVRMOD; // in case of overflow, use latest value

	ADC1->ISR |= ADC_ISR_ADRDY; // clear ADRDY
	ADC1->CR |= ADC_CR_ADEN; // turn on ADC
	while (!(ADC1->ISR & ADC_ISR_ADRDY))
		; // wait for ADR to be ready

	ADC->CCR |= ADC_CCR_TSEN; // wake up temperature sensor
	ADC->CCR |= ADC_CCR_VREFEN; // Enable VREF (do I need to do that?)
	delay_ms(1); // give the sensor some time to wake up, probably way too much
}

void set_EOC_callback(eoc_callback_t callback) {
	eoc_callback = callback;
}

