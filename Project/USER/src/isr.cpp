extern "C" {
#include "isr.h"

#include "headfile.h"
}  // extern "C"
#include "devices.hpp"
void CSI_IRQHandler(void) {
    rt_interrupt_enter();

    CSI_DriverIRQHandler();  //调用SDK自带的中断函数 这个函数最后会调用我们设置的回调函数
    __DSB();                 //数据同步隔离
    rt_interrupt_leave();
}

void PIT_IRQHandler(void) {
    rt_interrupt_enter();
    if (PIT_FLAG_GET(PIT_CH0)) { PIT_FLAG_CLEAR(PIT_CH0); }

    if (PIT_FLAG_GET(PIT_CH1)) { PIT_FLAG_CLEAR(PIT_CH1); }

    if (PIT_FLAG_GET(PIT_CH2)) { PIT_FLAG_CLEAR(PIT_CH2); }

    if (PIT_FLAG_GET(PIT_CH3)) { PIT_FLAG_CLEAR(PIT_CH3); }

    __DSB();
    rt_interrupt_leave();
}

void GPIO2_Combined_16_31_IRQHandler(void) {
    rt_interrupt_enter();
    if (GET_GPIO_FLAG(C16)) {
        CLEAR_GPIO_FLAG(C16);  //清除中断标志位
    }
    if (GET_GPIO_FLAG(C30)) {
        CLEAR_GPIO_FLAG(C30);
        // imu.readSensor();
        imu.newData();
        // PRINTF("%d\n\r", rt_tick_get());
    }
    rt_interrupt_leave();
}

void GPIO2_Combined_0_15_IRQHandler(void) {
    rt_interrupt_enter();
    if (GET_GPIO_FLAG(MT9V03X_VSYNC_PIN)) {
        //不用清除标志位，标志位在mt9v03x_vsync函数内部会清除
        if (CAMERA_GRAYSCALE == flexio_camera_type) mt9v03x_vsync();
    }
    if (GET_GPIO_FLAG(SCC8660_VSYNC_PIN)) {
        //不用清除标志位，标志位在scc8660_vsync函数内部会清除
        if (CAMERA_COLOR == flexio_camera_type) scc8660_vsync();
    }

    rt_interrupt_leave();
}

/*
GPIO3_Combined_0_15_IRQHandler
此中断函数默认被SD卡检测插入功能占用，如果需要gpio中断建议使用其他IO
或者不使用SD的卡可以将fsl_sdmmc_host.c中的 SDMMCHOST_CARD_DETECT_GPIO_INTERRUPT_HANDLER函数注释掉即可

*/

/*
中断函数名称，用于设置对应功能的中断函数
Sample usage:当前启用了周期定时器中断
void PIT_IRQHandler(void)
{
    //务必清除标志位
    __DSB();
}
记得进入中断后清除标志位
CTI0_ERROR_IRQHandler
CTI1_ERROR_IRQHandler
CORE_IRQHandler
FLEXRAM_IRQHandler
KPP_IRQHandler
TSC_DIG_IRQHandler
GPR_IRQ_IRQHandler
LCDIF_IRQHandler
CSI_IRQHandler
PXP_IRQHandler
WDOG2_IRQHandler
SNVS_HP_WRAPPER_IRQHandler
SNVS_HP_WRAPPER_TZ_IRQHandler
SNVS_LP_WRAPPER_IRQHandler
CSU_IRQHandler
DCP_IRQHandler
DCP_VMI_IRQHandler
Reserved68_IRQHandler
TRNG_IRQHandler
SJC_IRQHandler
BEE_IRQHandler
PMU_EVENT_IRQHandler
Reserved78_IRQHandler
TEMP_LOW_HIGH_IRQHandler
TEMP_PANIC_IRQHandler
USB_PHY1_IRQHandler
USB_PHY2_IRQHandler
ADC1_IRQHandler
ADC2_IRQHandler
DCDC_IRQHandler
Reserved86_IRQHandler
Reserved87_IRQHandler
GPIO1_INT0_IRQHandler
GPIO1_INT1_IRQHandler
GPIO1_INT2_IRQHandler
GPIO1_INT3_IRQHandler
GPIO1_INT4_IRQHandler
GPIO1_INT5_IRQHandler
GPIO1_INT6_IRQHandler
GPIO1_INT7_IRQHandler
GPIO1_Combined_0_15_IRQHandler
GPIO1_Combined_16_31_IRQHandler
GPIO2_Combined_0_15_IRQHandler
GPIO2_Combined_16_31_IRQHandler
GPIO3_Combined_0_15_IRQHandler
GPIO3_Combined_16_31_IRQHandler
GPIO4_Combined_0_15_IRQHandler
GPIO4_Combined_16_31_IRQHandler
GPIO5_Combined_0_15_IRQHandler
GPIO5_Combined_16_31_IRQHandler
WDOG1_IRQHandler
RTWDOG_IRQHandler
EWM_IRQHandler
CCM_1_IRQHandler
CCM_2_IRQHandler
GPC_IRQHandler
SRC_IRQHandler
Reserved115_IRQHandler
GPT1_IRQHandler
GPT2_IRQHandler
PWM1_0_IRQHandler
PWM1_1_IRQHandler
PWM1_2_IRQHandler
PWM1_3_IRQHandler
PWM1_FAULT_IRQHandler
SEMC_IRQHandler
USB_OTG2_IRQHandler
USB_OTG1_IRQHandler
XBAR1_IRQ_0_1_IRQHandler
XBAR1_IRQ_2_3_IRQHandler
ADC_ETC_IRQ0_IRQHandler
ADC_ETC_IRQ1_IRQHandler
ADC_ETC_IRQ2_IRQHandler
ADC_ETC_ERROR_IRQ_IRQHandler
PIT_IRQHandler
ACMP1_IRQHandler
ACMP2_IRQHandler
ACMP3_IRQHandler
ACMP4_IRQHandler
Reserved143_IRQHandler
Reserved144_IRQHandler
ENC1_IRQHandler
ENC2_IRQHandler
ENC3_IRQHandler
ENC4_IRQHandler
TMR1_IRQHandler
TMR2_IRQHandler
TMR3_IRQHandler
TMR4_IRQHandler
PWM2_0_IRQHandler
PWM2_1_IRQHandler
PWM2_2_IRQHandler
PWM2_3_IRQHandler
PWM2_FAULT_IRQHandler
PWM3_0_IRQHandler
PWM3_1_IRQHandler
PWM3_2_IRQHandler
PWM3_3_IRQHandler
PWM3_FAULT_IRQHandler
PWM4_0_IRQHandler
PWM4_1_IRQHandler
PWM4_2_IRQHandler
PWM4_3_IRQHandler
PWM4_FAULT_IRQHandler
Reserved171_IRQHandler
GPIO6_7_8_9_IRQHandler*/
