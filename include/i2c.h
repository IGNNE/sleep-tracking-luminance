#ifndef I2C_H_
#define I2C_H_

#ifdef __cplusplus
#include <cstdint>
extern "C" {
#else
#include <stdint.h>
#include <stdbool.h>
#endif

// sets up I2C1 on PB8 (SCL) and PB9 (SDA)
// connected to LCD shield
// standard mode (100kHz), 7 bit addressing
void i2c_setup(void);

// performs a write/read transfer to the 7 bit slave_address
// performs a repeated start if both write_buffer and read_buffer are given.
// returns true on success and false if a NACK was received.
bool i2c_transfer(uint8_t slave_address, const uint8_t *write_buffer,
		uint8_t bytes_to_write, uint8_t *read_buffer, uint8_t bytes_to_read);

#ifdef __cplusplus
}
#endif

#endif /* I2C_H_ */
