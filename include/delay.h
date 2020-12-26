#ifndef DELAY_H_
#define DELAY_H_

#ifdef __cplusplus
#include <cstdint>
extern "C" {
#else
#include <stdint.h>
#endif

// millisecond delay
void delay_ms(uint32_t ms);

// microsecond delay
void delay_us(uint32_t us);

#ifdef __cplusplus
}
#endif

#endif /* DELAY_H_ */
