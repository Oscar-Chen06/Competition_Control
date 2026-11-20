#ifndef TIM1_MUX_H
#define TIM1_MUX_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* TIM1 由 MG513（CH1/PA8）和步进（CH3/PA10）分时复用，不能同时运行。
 * 每次开始运动前由对应驱动重新配置 PSC/ARR 并启停通道。
 */
void Tim1Mux_TakeMg513(uint32_t psc, uint32_t arr);
void Tim1Mux_TakeStepper(uint32_t psc, uint32_t arr);
void Tim1Mux_ReleaseStepper(void);

#ifdef __cplusplus
}
#endif

#endif /* TIM1_MUX_H */
