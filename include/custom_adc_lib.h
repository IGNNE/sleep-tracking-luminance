#include <stm32l0xx.h>
#include <stdlib.h>
#include <delay.h>

/**
 * @file Small library to handle the ADC
 * @author Malte Klas
 */

/**
 * Callback type for the EOC interrupt
 */
typedef void (*eoc_callback_t)(uint16_t);

/**
 * Enable an interrupt on EOC
 *
 * @note  this will call the function set in set_EOC_callback()
 */
void enable_EOC_interrupt();

/**
 * Enable automatic measurement with TIM6
 *
 * @param time_ms time between two measurements
 */
void enable_EOC_interrupt_timer(uint16_t time_ms);

/**
 * Clears all interrupt and timer settings
 */
void disable_adc_interrupts();

/**
 * Read a single value from the ADC in discontinuous, blocking mode
 */
uint16_t read_adc_raw_blocking(uint32_t channel);

/**
 * Start a measurement, but do not wait for it to complete
 * @note to be used with interrupts (but without enable_eoc_interrupt_timer)
 *
 */
void read_adc_interrupt(uint32_t channel);

/**
 * Measure VREFINT (blocking) and calculate VDDA/AVDD/VDD_APPLI
 *
 * @note this function disables the interrupts while it is running, which means that you can safely use it together with interrupts in most cases
 *
 * @return the ADC reference voltage in mV
 */
uint16_t read_vdda();

/**
 * Initialise the ADC, default settings are for discontinuous mode
 */
void adc_init();

/**
 * Register a callback to react on an End-Of-Conversion interrupt
 */
void set_EOC_callback(eoc_callback_t callback);
