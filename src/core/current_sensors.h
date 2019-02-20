

#ifndef CURR_SENSOR
#define CURR_SENSOR

#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"

#define SAMPLES_IN_BUFFER 5
static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];

void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
	int temp;
	char tempStr[100];
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        int avg_sample = 0;
        float steinhart;
        float sampleAVG;
        int i;
        ret_code_t err_code;
		
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        //APP_ERROR_CHECK(err_code);

        for (i = 0; i < SAMPLES_IN_BUFFER; i++)	
        {
              avg_sample += p_event->data.done.p_buffer[i]; // take N samples in a row
        }
		temp = avg_sample/i;
		if (avg_sample/i >200){
			nrf_gpio_pin_set(23);
			nrf_gpio_pin_clear(22);
		}
		else{
			nrf_gpio_pin_set(22);
			nrf_gpio_pin_clear(23);
		}
		nrf_delay_ms(20);
		
		sprintf(tempStr,"code is: %d\n",temp);
//		SEGGER_RTT_WriteString(0, tempStr);
        
    }
}

void saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
	
    channel_config.gain = NRF_SAADC_GAIN1_6;

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0],SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1],SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}

void saadc_sampling_trigger(void)
{
    ret_code_t err_code;
	//Event handler is called immediately after conversion is finished.
	err_code = nrf_drv_saadc_sample(); // Check error
	APP_ERROR_CHECK(err_code);
}

#endif //CURR_SENSOR
