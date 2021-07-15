#include <zephyr.h>
#include <zephyr/types.h>

#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#define NUM_CHANNEL_ADC 2

#define SAMPLES_IN_BUFFER 50

#define ELEM_BUFF_SAMPLES (NUM_CHANNEL_ADC*SAMPLES_IN_BUFFER)

// ===========================================================================
//  Test point (gpios) definitions
//============================================================================

// The devicetree node identifier for the "tp0"
#define TP0_NODE   DT_ALIAS(tp0)

#if DT_NODE_HAS_STATUS(TP0_NODE, okay)
#define TP0_LAB   DT_GPIO_LABEL(TP0_NODE, gpios)
#define TP0_PIN   DT_GPIO_PIN(TP0_NODE, gpios)
#if DT_PHA_HAS_CELL(TP0_NODE, gpios, flags)
#define TP0_FLG  DT_GPIO_FLAGS(TP0_NODE, gpios)
#else
#define TP0_FLG  0
#endif

#define USE_TP0
extern const struct device *dev_tp0;
#endif

// ===========================================================================
//  Power control (gpios) definitions
//============================================================================

// The devicetree node identifier for the "pwr_sens" alias
#define PWRCTRL0_NODE DT_ALIAS(pwrctrl0)

#if DT_NODE_HAS_STATUS(PWRCTRL0_NODE, okay)
#define PWRCTRL0_LAB  DT_GPIO_LABEL(PWRCTRL0_NODE, gpios)
#define PWRCTRL0_PIN  DT_GPIO_PIN(PWRCTRL0_NODE, gpios)
#if DT_PHA_HAS_CELL(PWRCTRL0_NODE, gpios, flags)
#define PWRCTRL0_FLG  DT_GPIO_FLAGS(PWRCTRL0_NODE, gpios)
#else
#define PWRCTRL0_FLG  0
#endif

#define USE_PWRCTRL0
extern struct device *dev_pwrctrl0;
#endif

// ===========================================================================
//  LEDs (gpios) definitions
//============================================================================

// The devicetree node identifier for the "led-ble" alias
#define LED0_NODE DT_ALIAS(led_ble)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED0_LAB  DT_GPIO_LABEL(LED0_NODE, gpios)
#define LED0_PIN  DT_GPIO_PIN(LED0_NODE, gpios)
#if DT_PHA_HAS_CELL(LED0_NODE, gpios, flags)
#define LED0_FLG  DT_GPIO_FLAGS(LED0_NODE, gpios)
#else
#define LED0_FLG  0
#endif

#define USE_LED0
extern struct device *dev_led0;
#endif

// The devicetree node identifier for the "led-act" alias
#define LED1_NODE DT_ALIAS(led_act)

#if DT_NODE_HAS_STATUS(LED1_NODE, okay)
#define LED1_LAB  DT_GPIO_LABEL(LED1_NODE, gpios)
#define LED1_PIN  DT_GPIO_PIN(LED1_NODE, gpios)
#if DT_PHA_HAS_CELL(LED1_NODE, gpios, flags)
#define LED1_FLG  DT_GPIO_FLAGS(LED1_NODE, gpios)
#else
#define LED1_FLG  0
#endif

#define USE_LED1
extern struct device *dev_led1;
#endif

// The devicetree node identifier for the "led-err" alias
#define LED2_NODE DT_ALIAS(led_err)

#if DT_NODE_HAS_STATUS(LED2_NODE, okay)
#define LED2_LAB  DT_GPIO_LABEL(LED2_NODE, gpios)
#define LED2_PIN  DT_GPIO_PIN(LED2_NODE, gpios)
#if DT_PHA_HAS_CELL(LED2_NODE, gpios, flags)
#define LED2_FLG  DT_GPIO_FLAGS(LED2_NODE, gpios)
#else
#define LED2_FLG  0
#endif

#define USE_LED2
extern struct device *dev_led2;
#endif

// ===========================================================================
//  ADC channel definitions
//============================================================================

// The devicetree node identifier for the "adc-ch0" alias
#define ADCCH0_NODE DT_ALIAS(adc_ch0)
#if DT_NODE_EXISTS(ADCCH0_NODE)
#define USE_ADCCH0
#define ADCCH0_PIN  DT_PROP(ADCCH0_NODE, channel)
#endif

// The devicetree node identifier for the "adc-ch1" alias
#define ADCCH1_NODE DT_ALIAS(adc_ch1)
#if DT_NODE_EXISTS(ADCCH1_NODE)
#define USE_ADCCH1
#define ADCCH1_PIN  DT_PROP(ADCCH1_NODE, channel)
#endif

// The devicetree node identifier for the "adc-ch2" alias
#define ADCCH2_NODE DT_ALIAS(adc_ch2)
#if DT_NODE_EXISTS(ADCCH2_NODE)
#define USE_ADCCH2
#define ADCCH2_PIN  DT_PROP(ADCCH2_NODE, channel)
#endif

// The devicetree node identifier for the "adc-ch3" alias
#define ADCCH3_NODE DT_ALIAS(adc_ch3)
#if DT_NODE_EXISTS(ADCCH3_NODE)
#define USE_ADCCH3
#define ADCCH3_PIN  DT_PROP(ADCCH3_NODE, channel)
#endif
