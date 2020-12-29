#include <stm32l0xx.h>
#include <stdlib.h>
#include <delay.h>

/**
 * @file custom_adc_lib.h Kleine Library, um den STM32-ADC anzusprechen
 * @author Malte Klas
 */

/**
 * Callback-Typ für den End-of-Calibration (EOC) interrupt
 */
typedef void (*eoc_callback_t)(uint16_t);

/**
 * Aktiviert den Interrupt für EOC
 *
 * @note Der Interrupt wird den Callback aus ::set_EOC_callback aufrufen
 */
void enable_EOC_interrupt();

/**
 * Aktiviert die automatische Messung mit Timer und Interrupt
 *
 * @note Der Prescaler für den Timer wird auf 8000 gesetzt, damit er mit einem Takt von 16 MHz mit einem Intervall von 1 ms zählt.
 *       Dadurch kann der gewünschte Zeitwert einfach in das Reload-Register geladen werden.
 *       Entsprechende Formel im im Datasheet: f / (PSC+1)
 *
 * @todo Warum 8000? Es müsste eigentlich 16000 (bzw 15999...) sein!
 *
 * @param time_ms Zeit zwischen zwei Messungen
 */
void enable_EOC_interrupt_timer(uint16_t time_ms);

/**
 * Deaktivert Interrupts und setzt die entspr. Einstellungen zurück
 */
void disable_adc_interrupts();

/**
 * Liest einen ADC-Wert aus und blockiert, bis das Ergebnis vorliegt
 */
uint16_t read_adc_raw_blocking(uint32_t channel);

/**
 * Startet eine Messung, aber wartet nicht auf das Ergebnis
 *
 * @note Diese Funktion ist nur bei Verwendung der Interrupts sinnvoll (::enable_EOC_interrupt bzw. ::enable_EOC_interrupt_timer)
 *
 */
void read_adc_interrupt(uint32_t channel);

/**
 * Misst VREFINT (blockierend) und berechnet VDDA (auch AVDD/VDD_APPLI im Datasheet)
 *
 * @note Diese Funktion deaktiviert den ADC-Interrupt für die Dauer der Messung
 *
 * @note Der Timer (TIM6) läuft ggf. weiter, während diese Funktion läuft. Es sollte darauf geachtet werden, dass es nicht zu Konflikten kommt!
 *
 * @return ADC-Referenzspannung VDDA in mV
 */
uint16_t read_vdda();

/**
 * Initialisiert den ADC, einschließlich Kalibirerung und 16x Oversampling/Mittelwertbildung
 */
void adc_init();

/**
 * Registriert den Callback für den EOC-Interrupt
 *
 * @param[in] callback Funktionszeiger, der beim Interrupt aufgerufen wird
 */
void set_EOC_callback(eoc_callback_t callback);
