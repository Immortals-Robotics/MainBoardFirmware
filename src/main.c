/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 * @defgroup uart_example_main main.c
 * @{
 * @ingroup uart_example
 * @brief UART Example Application main file.
 *
 * This file contains the source code for a sample application using UART.
 *
 */
 
 
#define NRF_ESB_LEGACY
#include "nrf_esb.h"
#include "my_UART.h"

#include "nrf_drv_spi.h"


#include "nrf_esb_error_codes.h"
#include "nrf_gpio.h"

#define CONFIG_NFCT_PINS_AS_GPIOS


#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "boards.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//#include "nrf_drv_saadc.h"

#include "my_esb_RF.h"//stack of code (SO MESSED UP!!!)
#include "my_ADC.h"
//#include "my_UART.h"


//#define SAMPLES_IN_BUFFER 5
//static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];


#define RX_MCU_PIN_NUMBER 7
#define TX_MCU_PIN_NUMBER 8
#define RTS_MCU_PIN_NUMBER 9
#define CTS_MCU_PIN_NUMBER 10

#define UART_MCU_RX_BUF_SIZE 64
#define UART_MCU_TX_BUF_SIZE 64

bool received_NRF;
bool HALTED = false;
int warming_up_count_down = 10;
uint32_t FRQ_channel = -1;
uint8_t IDnumber;

#define CHANNEL_COUNT 5
#define DATA_SAMPLE_BUFFERS 40
#define SAMPLES_IN_BUFFER DATA_SAMPLE_BUFFERS*CHANNEL_COUNT

volatile uint8_t state = 1;

static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(0);
static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t     m_ppi_channel;
static uint32_t              m_adc_evt_counter;

//Omid:
unsigned char errorBuff = 0x00;
unsigned char LOW_BATT = 0x01;
bool NEW_ADC_SAMPLE = false;
//unsigned char temp,temp2,temp3;
unsigned char BATT_ADC,
				MOTOR_1_ADC,
				MOTOR_2_ADC,
				MOTOR_3_ADC,
				MOTOR_4_ADC;
char tempStr[100];
int halt_motor[4] = {0,0,0,0};
#define MIN_MOTOR_ADC_RED_LINE 110

//#define I_RANGE 5;
//unsigned char Integral_Buffer[4][I_RANGE];
//int T_PNT = 0;
	
void send_halt_to_FPGA(){
	unsigned char data[3]={0x00,0x03,0x06};//HALT COMMAND CODE
	
	if(app_uart_tx_fifo_len() == 0){
		for(int i=0; i < 3; i++){
			while (app_uart_put(data[i]) != NRF_SUCCESS);
			nrf_delay_us(50);
		}
	}
}

void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
	
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        unsigned int avg_sample = 0;
        float steinhart;
        float sampleAVG;
        int i;
		
        ret_code_t err_code;
		
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        //APP_ERROR_CHECK(err_code);
		avg_sample = 0;
        for (i = 2; i < SAMPLES_IN_BUFFER; i+=CHANNEL_COUNT)	
        {
              avg_sample += (unsigned char) p_event->data.done.p_buffer[i]; // take N samples in a row
        }
		BATT_ADC = avg_sample/DATA_SAMPLE_BUFFERS;
		
		avg_sample = 0;
		for (i = 4; i < SAMPLES_IN_BUFFER; i+=CHANNEL_COUNT)	
        {
              avg_sample += (unsigned char) p_event->data.done.p_buffer[i]; // take N samples in a row
        }
		MOTOR_1_ADC = avg_sample/DATA_SAMPLE_BUFFERS;
		if(MOTOR_1_ADC > MIN_MOTOR_ADC_RED_LINE){
			halt_motor[0] = (halt_motor[0] > 0) ? (halt_motor[0] - 1) : 0;
		}else{
			halt_motor[0]++;	
		}
		
		avg_sample = 0;
		for (i = 1; i < SAMPLES_IN_BUFFER; i+=CHANNEL_COUNT)	
        {
              avg_sample += (unsigned char) p_event->data.done.p_buffer[i]; // take N samples in a row
        }
		MOTOR_2_ADC = avg_sample/DATA_SAMPLE_BUFFERS;
		if(MOTOR_2_ADC > MIN_MOTOR_ADC_RED_LINE){
			halt_motor[1] = (halt_motor[1] > 0) ? (halt_motor[1] - 1) : 0;
		}else{
			halt_motor[1]++;	
		}
		
		avg_sample = 0;
		for (i = 0; i < SAMPLES_IN_BUFFER; i+=CHANNEL_COUNT)	
        {
              avg_sample += (unsigned char) p_event->data.done.p_buffer[i]; // take N samples in a row
        }
		MOTOR_3_ADC = avg_sample/DATA_SAMPLE_BUFFERS;
		if(MOTOR_3_ADC > MIN_MOTOR_ADC_RED_LINE){
			halt_motor[2] = (halt_motor[2] > 0) ? (halt_motor[2] - 1) : 0;
		}else{
			halt_motor[2]++;	
		}
		
		avg_sample = 0;
		for (i = 3; i < SAMPLES_IN_BUFFER; i+=CHANNEL_COUNT)	
        {
              avg_sample += (unsigned char) p_event->data.done.p_buffer[i]; // take N samples in a row
        }
		MOTOR_4_ADC = avg_sample/DATA_SAMPLE_BUFFERS;
		if(MOTOR_4_ADC > MIN_MOTOR_ADC_RED_LINE){
			halt_motor[3] = (halt_motor[3] > 0) ? (halt_motor[3] - 1) : 0;
		}else{
			halt_motor[3]++;	
		}
		
		warming_up_count_down --;
        NEW_ADC_SAMPLE = true;
    }
}


void saadc_init(void)
{
    ret_code_t err_code;
	
    nrf_saadc_channel_config_t channel_0_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);//MOTOR 3
    channel_0_config.gain = NRF_SAADC_GAIN1_6;
    
	nrf_saadc_channel_config_t channel_1_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);//MOTOR 2
    channel_1_config.gain = NRF_SAADC_GAIN1_6;
	
	nrf_saadc_channel_config_t channel_2_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN4);//ADC_BATT
    channel_2_config.gain = NRF_SAADC_GAIN1_6;
	channel_2_config.reference = NRF_SAADC_REFERENCE_VDD4;
	
	nrf_saadc_channel_config_t channel_3_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN6);//MOTOR 4
    channel_3_config.gain = NRF_SAADC_GAIN1_6;
	
	nrf_saadc_channel_config_t channel_4_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN7);//MOTOR 1
    channel_4_config.gain = NRF_SAADC_GAIN1_6;
	
	
	
	err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_0_config);
    APP_ERROR_CHECK(err_code);
	
	err_code = nrf_drv_saadc_channel_init(1, &channel_1_config);
    APP_ERROR_CHECK(err_code);
	
	err_code = nrf_drv_saadc_channel_init(2, &channel_2_config);
    APP_ERROR_CHECK(err_code);
	
	err_code = nrf_drv_saadc_channel_init(3, &channel_3_config);
    APP_ERROR_CHECK(err_code);
	
	err_code = nrf_drv_saadc_channel_init(4, &channel_4_config);
    APP_ERROR_CHECK(err_code);
	
	
	
    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0],SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1],SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}
//char tempStr[64];
void send_ADC_MOTOR_CURRENT(){
	nrf_esb_stop_rx();
	esb_init_as_TX();
	
	//getting the current ADC data:
	tx_payload.length = 32;
	tx_payload.noack = false;
	nrf_delay_us(2000);
	while(1){
		
		tx_payload.noack = false;
		tx_payload.data[1] = BATT_ADC;
		tx_payload.data[2] = MOTOR_1_ADC;//halt_motor[0];
		tx_payload.data[3] = MOTOR_2_ADC;//halt_motor[1];
		tx_payload.data[4] = MOTOR_3_ADC;//halt_motor[2];
		tx_payload.data[5] = MOTOR_4_ADC;//halt_motor[3];
		if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
        {
			sprintf(tempStr,"%u -- %u -- %u -- %u -- %u\n", BATT_ADC, MOTOR_1_ADC, MOTOR_2_ADC, MOTOR_3_ADC, MOTOR_4_ADC);
            SEGGER_RTT_WriteString(0, tempStr);
			break;
        }
        else
        {
			//SEGGER_RTT_WriteString(0,tempStr);
        }
	}
	nrf_delay_us(2000);
	
	esb_init_as_RX();	
    nrf_esb_start_rx();
}

void send_data_TX(unsigned char TXdata[32]){
	nrf_esb_stop_rx();
	esb_init_as_TX();
	
	//getting the current ADC data:
	tx_payload.length = 32;
	nrf_delay_us(2000);
	int i;
	while(1){
		tx_payload.noack = false;
		for(i=0; i<31; i++)
			tx_payload.data[i+1] = TXdata[i];
		
		if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
        {
			break;
        }
	}
	nrf_delay_us(2000);
	
	esb_init_as_RX();	
    nrf_esb_start_rx();
}

void send_error_MSG(unsigned char errorCode){
	unsigned char TXdata[32];
	TXdata[0] = IDnumber;
	TXdata[1] = 0x01;//This message is an error
	TXdata[2] = errorCode;
	send_data_TX(TXdata);
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    uint32_t err_code;

    //bsp_board_leds_init();
	nrf_gpio_cfg_output(24);//BUZZ
	nrf_gpio_cfg_output(23);
	nrf_gpio_cfg_output(22);
	nrf_gpio_cfg_output(21);//LED reset
	nrf_gpio_pin_clear(22);
	nrf_gpio_cfg_output(6);//Found signal
	nrf_gpio_cfg_output(5);//POW_EN
	nrf_gpio_cfg_input(20,NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(19,NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(18,NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(17,NRF_GPIO_PIN_PULLUP);
	saadc_init();
	
	nrf_gpio_pin_set(22);
	nrf_delay_ms(200);
	nrf_gpio_pin_clear(22);
	nrf_delay_ms(200);
	nrf_gpio_pin_set(22);
	nrf_delay_ms(200);
	nrf_gpio_pin_clear(22);
	nrf_delay_ms(200);
	
	
	
//	//-------ADC-BATTERY-CHECK----------
	while(1){
		saadc_sampling_trigger();
		if(NEW_ADC_SAMPLE){
			if(BATT_ADC > 80)
				break;
			else{
				nrf_gpio_pin_set(21);
				nrf_delay_ms(100);
				nrf_gpio_pin_clear(21);
				nrf_delay_ms(100);
				nrf_gpio_pin_set(21);
				nrf_delay_ms(100);
				nrf_gpio_pin_clear(21);
				nrf_delay_ms(100);
			}
			nrf_delay_ms(2);
			NEW_ADC_SAMPLE = false;
		}
	}
	
//	//-----------END-OF-ADC-------------
	nrf_gpio_pin_set(5);// Fire up all the robot
	nrf_gpio_pin_clear(6);
	//--------------UART----------------
    const app_uart_comm_params_t comm_params =
      {
          RX_MCU_PIN_NUMBER,
          TX_MCU_PIN_NUMBER,
          RTS_MCU_PIN_NUMBER,
          CTS_MCU_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_ENABLED,
          false,
          UART_BAUDRATE_BAUDRATE_Baud31250
      };
		
	  
	  
    APP_UART_FIFO_INIT(&comm_params,
                         UART_MCU_RX_BUF_SIZE,
                         UART_MCU_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);
	//-------------END_OF_UART-------------
//	//TESTING:
//	  nrf_gpio_pin_set(6);//Signal that we found it!!!
//	  nrf_delay_ms(2000);
//	  unsigned char tmpDATA[20]={0,0,0,0,0,
//		  0,0,0,0,0,
//		  0,0,0,0,0,
//		  0,0,0,0,0};
//	tmpDATA[1]=0x0F;
//	tmpDATA[2]=11;
//	tmpDATA[3]=0x11;
//	tmpDATA[7]=0x37;
//	tmpDATA[8]=0x73;
//	tmpDATA[9]=0x55;
//	tmpDATA[10]=0x22;		  
//	tmpDATA[11]=0x22;
//	tmpDATA[12]=0x50;
//	  
//	while(1){
//		nrf_gpio_pin_set(22);
//		if(app_uart_tx_fifo_len() == 0){
//				for(int i=0; i < 20 ;i++){
//					
//					while (app_uart_put(tmpDATA[i]) != NRF_SUCCESS);
//				}
//			}
//		nrf_gpio_pin_clear(22);
//		nrf_delay_ms(200);
//			
//		nrf_gpio_pin_set(22);
//		if(app_uart_tx_fifo_len() == 0){
//				for(int i=0; i < 20 ;i++){
//					
//					while (app_uart_put(tmpDATA[i]) != NRF_SUCCESS);
//					nrf_delay_us(50);
//				}
//			}
//		nrf_gpio_pin_clear(22);
//		nrf_delay_ms(20000);
//	}
//	  
	  
	//----------------RADIO----------------
	
	IDnumber=0x00;

	if(nrf_gpio_pin_read(17) == 0)
		IDnumber |= 0x01;
	
	if(nrf_gpio_pin_read(20) == 0)
		IDnumber |= 0x02;
	
	if(nrf_gpio_pin_read(19) == 0)
		IDnumber |= 0x04;
	
	if(nrf_gpio_pin_read(18) == 0)
		IDnumber |= 0x08;
	
	nrf_delay_ms(10);

	clocks_start();	
	esb_init_as_RX();	
    nrf_esb_start_rx();
	nrf_esb_stop_rx();
	
//	// TODO comment this part!!!
//	nrf_esb_set_rf_channel(55);
//	nrf_esb_get_rf_channel(&FRQ_channel);
	
	//TODO test: (uncomment this part for the real firmware)
	while(!received_NRF){//search for the right frq
		nrf_gpio_pin_set(23);
		nrf_delay_ms(100);
		nrf_gpio_pin_clear(23);
		nrf_delay_ms(100);
		
		nrf_gpio_pin_set(24);
		nrf_delay_ms(20);
		nrf_gpio_pin_clear(24);
		
		received_NRF = false;
		for(int i=0;i<=100;i+=5){
			nrf_esb_stop_rx();
			nrf_esb_set_rf_channel(i);
			nrf_esb_start_rx();
			nrf_delay_ms(20);
			if(received_NRF){//got it!!!
				//nrf_gpio_pin_set(22);
				nrf_esb_stop_rx();
				nrf_esb_get_rf_channel(&FRQ_channel);
				
				uint8_t new_base_addr[4];
				new_base_addr[0] = 110;
				new_base_addr[1] = IDnumber;
				new_base_addr[2] = 110;
				new_base_addr[3] = 110;
				nrf_delay_ms(20);
				nrf_gpio_pin_set(24);
				nrf_delay_ms(20);
				nrf_gpio_pin_clear(24);
				nrf_delay_ms(100);
				nrf_gpio_pin_set(24);
				nrf_delay_ms(20);
				nrf_gpio_pin_clear(24);
				
				nrf_gpio_pin_set(6);//Signal that we found it!!!
				
				nrf_esb_set_base_address_0(new_base_addr);
				nrf_esb_start_rx();
				break;
			}
			
			
			
		}
	}
	//-------------END_OF_RADIO---------------
	  
    int CNT=0;
	SEGGER_RTT_WriteString(0, "ADC test is starting!\n");
	bool motors_are_off = false;
//	nrf_gpio_pin_set(6);
	nrf_gpio_pin_set(23);
	nrf_delay_ms(100);
	nrf_gpio_pin_clear(23);
    while (true)
    {
//        saadc_sampling_trigger();
////		send_ADC_MOTOR_CURRENT();
////		nrf_gpio_pin_set(22);
////		nrf_delay_ms(100);
////		nrf_gpio_pin_clear(22);
////		nrf_delay_ms(100);

//		errorBuff = 0x00;
//		if(halt_motor[0] >= 2)
//			errorBuff |= 0x01;
//		if(halt_motor[1] >= 2)
//			errorBuff |= 0x02;
//		if(halt_motor[2] >= 2)
//			errorBuff |= 0x04;
//		if(halt_motor[3] >= 2)
//			errorBuff |= 0x08;
//		send_error_MSG(errorBuff);
//		if(errorBuff){
//			send_halt_to_FPGA();
//			send_error_MSG(errorBuff);
//			
//			nrf_gpio_pin_set(24);
//			nrf_delay_ms(30);
//			nrf_gpio_pin_clear(24);
//			nrf_delay_ms(100);
//			nrf_gpio_pin_set(24);
//			nrf_delay_ms(30);
//			nrf_gpio_pin_clear(24);
//			nrf_delay_ms(100);
//			nrf_gpio_pin_set(24);
//			nrf_delay_ms(70);
//			nrf_gpio_pin_clear(24);
//			nrf_delay_ms(200);
//		}
//		motors_are_off = abs(MOTOR_1_ADC-190) < 10 && 
//								abs(MOTOR_2_ADC-190) < 10  && 
//								abs(MOTOR_3_ADC-190) < 10  && 
//								abs(MOTOR_4_ADC-190) < 10;
//		if((HALTED | motors_are_off) && warming_up_count_down == 0 && BATT_ADC <= 85)
//		{
//			send_error_MSG(LOW_BATT);
//			
//			nrf_gpio_pin_set(24);
//			nrf_delay_ms(20);
//			nrf_gpio_pin_clear(24);
//			nrf_delay_ms(100);
//			nrf_gpio_pin_set(24);
//			nrf_delay_ms(20);
//			nrf_gpio_pin_clear(24);
//			nrf_delay_ms(100);
//			nrf_gpio_pin_set(24);
//			nrf_delay_ms(20);
//			nrf_gpio_pin_clear(24);
//			nrf_delay_ms(100);
//			nrf_gpio_pin_set(24);
//			nrf_delay_ms(20);
//			nrf_gpio_pin_clear(24);
//			nrf_delay_ms(200);
//		}

////		sprintf(tempStr,"%u --\n", BATT_ADC);
////        SEGGER_RTT_WriteString(0, tempStr);
//			
//		
//		nrf_delay_ms(10);
		__WFE();
    }

}

//Complete firmware
/** @} */
