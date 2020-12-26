#ifndef CLOCK_H_
#define CLOCK_H_

#ifdef __cplusplus
extern "C" {
#endif

// sets up system and peripheral clocks
// 16 MHz HSI as system clock and peripheral clocks
// using medium power range
void clock_setup_16MHz(void);

#ifdef __cplusplus
}
#endif

#endif /* CLOCK_H_ */
