#ifndef SYSTICK_H_
#define SYSTICK_H_

#ifdef __cplusplus
extern "C" {
#endif

// configures the system tick to generate an interrupt every millisecond
void systick_setup(void);

#ifdef __cplusplus
}
#endif

#endif /* SYSTICK_H_ */
