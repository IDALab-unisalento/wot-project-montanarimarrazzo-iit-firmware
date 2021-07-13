/*
 * Test adc - by LF
 */
//------------------------------------------------------------------

#include <zephyr.h>
#include <zephyr/types.h>

#include <drivers/adc.h>
#include <hal/nrf_saadc.h>

#include <stddef.h>
//#include <string.h>

#include "gpio_map.h"

//------------------------------------------------------------------
// buffer memory to be used for max 2 adc channels
static s16_t sample_buff[ADC_BUFF_SIZE];

//------------------------------------------------------------------
// generic adc drive configuration
#define ADC_DEVICE_NAME		DT_LABEL(DT_INST(0, nordic_nrf_saadc))
#define ADC_RESOLUTION		10

//------------------------------------------------------------------
// each adc channel can have different settings
// we define here default ones
#define ADC_GAIN		ADC_GAIN_1_3        // max 0.6V*3 = 1.8V
#define ADC_REFERENCE		ADC_REF_INTERNAL    // 0.6Vref
#define ADC_ACQUISITION_TIME	ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40)

//------------------------------------------------------------------
// uncomment used channels and fill their adc_channel_cfg struct
#define ADC_CHANNEL1_ID         0
#define ADC_CHANNEL1_INPUT	ADCCH0_PIN
#define ADC_CHANNEL2_ID         1
#define ADC_CHANNEL2_INPUT	ADCCH1_PIN
//#define ADC_CHANNEL3_ID	2
//#define ADC_CHANNEL3_INPUT	ADCCH2_PIN
//#define ADC_CHANNEL4_ID	3
//#define ADC_CHANNEL4_INPUT	ADCCH3_PIN

//------------------------------------------------------------------
#if defined(ADC_CHANNEL1_ID)
static const struct adc_channel_cfg channel1_cfg =
{
	.gain             = ADC_GAIN,
	.reference        = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id       = ADC_CHANNEL1_ID,
        .differential     = NRF_SAADC_MODE_SINGLE_ENDED,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
	.input_positive   = ADC_CHANNEL1_INPUT,
	.input_negative   = NRF_SAADC_INPUT_DISABLED,
#endif
};
#endif // defined(ADC_CHANNEL1_ID)

//------------------------------------------------------------------
#if defined(ADC_CHANNEL2_ID)
static const struct adc_channel_cfg channel2_cfg =
{
	.gain             = ADC_GAIN,
	.reference        = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id       = ADC_CHANNEL2_ID,
        .differential     = NRF_SAADC_MODE_SINGLE_ENDED,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
	.input_positive   = ADC_CHANNEL2_INPUT,
	.input_negative   = NRF_SAADC_INPUT_DISABLED,
#endif
};
#endif // defined(ADC_CHANNEL2_ID)

//------------------------------------------------------------------
#if defined(ADC_CHANNEL3_ID)
static const struct adc_channel_cfg channel3_cfg =
{
	.gain             = ADC_GAIN,
	.reference        = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id       = ADC_CHANNEL3_ID,
        .differential     = NRF_SAADC_MODE_SINGLE_ENDED,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
	.input_positive   = ADC_CHANNEL3_INPUT,
	.input_negative   = NRF_SAADC_INPUT_DISABLED,
#endif
};
#endif // defined(ADC_CHANNEL3_ID)

//------------------------------------------------------------------
#if defined(ADC_CHANNEL4_ID)
static const struct adc_channel_cfg channel4_cfg =
{
	.gain             = ADC_GAIN,
	.reference        = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id       = ADC_CHANNEL4_ID,
        .differential     = NRF_SAADC_MODE_SINGLE_ENDED,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
	.input_positive   = ADC_CHANNEL4_INPUT,
	.input_negative   = NRF_SAADC_INPUT_DISABLED,
#endif
};
#endif // defined(ADC_CHANNEL4_ID)

//------------------------------------------------------------------
// Functions
//------------------------------------------------------------------

static u32_t channel_mask;

const struct device *get_adc_device(void)
{
  return device_get_binding(ADC_DEVICE_NAME);
}

//------------------------------------------------------------------
// initialize adc - return adc_dev pointer if success otherwise null
const struct device *adc_init(void)
{
  int ret = 0;
  
  channel_mask = 0;

  const struct device *adc_dev = device_get_binding(ADC_DEVICE_NAME);
  if (!adc_dev)
    return 0;

#if defined(ADC_CHANNEL1_ID)
  ret = adc_channel_setup(adc_dev, &channel1_cfg);
  if (!ret)
    channel_mask |= BIT(ADC_CHANNEL1_ID);
#endif // defined(ADC_CHANNEL1_ID)
  
#if defined(ADC_CHANNEL2_ID)
  ret = adc_channel_setup(adc_dev, &channel2_cfg);
  if (!ret)
    channel_mask |= BIT(ADC_CHANNEL2_ID);
#endif // defined(ADC_CHANNEL2_ID)

#if defined(ADC_CHANNEL3_ID)
  ret = adc_channel_setup(adc_dev, &channel3_cfg);
  if (!ret)
    channel_mask |= BIT(ADC_CHANNEL3_ID);
#endif // defined(ADC_CHANNEL3_ID)

#if defined(ADC_CHANNEL4_ID)
  ret = adc_channel_setup(adc_dev, &channel4_cfg);
  if (!ret)
    channel_mask |= BIT(ADC_CHANNEL4_ID);
#endif // defined(ADC_CHANNEL4_ID)

  if (!channel_mask)
    return 0;

  // clear sample buffer
//(void)memset(sample_buff, 0, sizeof(sample_buff));

  return adc_dev;
}

// sample all adc channels - return data count or 0 if error
int adc_sample(const struct device *adc_dev)
{
  int ret;

  // set adc conversion sequence
  const struct adc_sequence sequence =
  {
  	.options     = NULL,                // no extra samples and callback
	.channels    = channel_mask,        // bit mask of channels to read
	.buffer      = sample_buff,         // where to put samples read
	.buffer_size = sizeof(sample_buff),
	.resolution  = ADC_RESOLUTION,      // desired resolution
	.oversampling = 0,                  // don't oversample
	.calibrate = 0                      // don't calibrate
  };

  // read adc
  ret = adc_read(adc_dev, &sequence);
  if (ret)
    return 0;

  // prepare buffer for conversion
  for (int i=0; i < ADC_BUFF_SIZE; i++)
  {

        s32_t milliv = sample_buff[i];
    
        // get millivolt
        adc_raw_to_millivolts(adc_ref_internal(adc_dev), ADC_GAIN, ADC_RESOLUTION, &milliv);
        
        sample_buff[i] = (s16_t)milliv;    
         
  }

  return ADC_BUFF_SIZE;
}

s16_t *adc_get_millivolts(void)
{
  return sample_buff;
}
