#ifndef _isr_h
#define _isr_h

#ifdef __cplusplus
extern "C" {
#endif

void CSI_IRQHandler(void);
void PIT_IRQHandler(void);
void GPIO2_Combined_0_15_IRQHandler(void);
void GPIO2_Combined_16_31_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif
