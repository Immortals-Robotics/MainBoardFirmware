


//RADIO:
uint8_t led_nr;

nrf_esb_payload_t rx_payload;

/*lint -save -esym(40, BUTTON_1) -esym(40, BUTTON_2) -esym(40, BUTTON_3) -esym(40, BUTTON_4) -esym(40, LED_1) -esym(40, LED_2) -esym(40, LED_3) -esym(40, LED_4) */
bool received_NRF;
nrf_esb_payload_t nrf_packet_payload;

uint32_t FRQ_channel = -1;
extern uint8_t IDnumber;
extern bool HALTED;

void nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
	//nrf_gpio_pin_set(23);
  	//nrf_gpio_pin_toggle(23);
	switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
            NRF_LOG_DEBUG("TX SUCCESS EVENT");
            break;
        case NRF_ESB_EVENT_TX_FAILED:
            NRF_LOG_DEBUG("TX FAILED EVENT");
            (void) nrf_esb_flush_tx();
            (void) nrf_esb_start_tx();
            break;
        case NRF_ESB_EVENT_RX_RECEIVED:
            NRF_LOG_DEBUG("RX RECEIVED EVENT");
            if (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS)
            {
				//nrf_gpio_pin_set(22);
				received_NRF=true;
				
				if(app_uart_tx_fifo_len() == 0){
					for(int i=0; i < rx_payload.data[1] && i < rx_payload.length ;i++){
						
						while (app_uart_put(rx_payload.data[i]) != NRF_SUCCESS);
						//nrf_delay_us(50);
					}
				}
				
				//Storing the packets to process it later
				nrf_packet_payload = rx_payload;
				
				HALTED = (rx_payload.data[2] == 0x06);
				nrf_esb_flush_rx();
                //nrf_gpio_pin_clear(22);				
            }
            break;
    }
	nrf_esb_flush_rx();
}


void clocks_start( void )
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}


void gpio_init( void )
{
    bsp_board_leds_init();
}


uint32_t esb_init_as_RX( void )
{
    uint32_t err_code;
	uint8_t base_addr_0[4] = {110, 25, 110, 110};
	if(FRQ_channel != -1)//we have found the senders signal
		base_addr_0[1] =  IDnumber;
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t addr_prefix[8] = {110, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 };
    nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;
    nrf_esb_config.payload_length           = 30;
    nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
    nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_2MBPS;
    nrf_esb_config.mode                     = NRF_ESB_MODE_PRX;
    nrf_esb_config.event_handler            = nrf_esb_event_handler;
    nrf_esb_config.selective_auto_ack       = false;
	
	
    err_code = nrf_esb_init(&nrf_esb_config);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_0(base_addr_0);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(addr_prefix, 8);
    VERIFY_SUCCESS(err_code);
	
	if(FRQ_channel != -1){
		nrf_esb_set_rf_channel(FRQ_channel);
	}else{
		nrf_esb_set_rf_channel(2);
	}
	
	
    return err_code;
}

//TX:
static nrf_esb_payload_t        tx_payload = NRF_ESB_CREATE_PAYLOAD(0, 0x01, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00);

uint32_t esb_init_as_TX( void )
{
    uint32_t err_code;
    uint8_t base_addr_0[4] = {110, 30, 110, 110};//TX address for all robots are 30
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t addr_prefix[8] = {110, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 };

    nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;
    nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
    nrf_esb_config.retransmit_delay         = 600;
    nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_2MBPS;
    nrf_esb_config.event_handler            = nrf_esb_event_handler;
    nrf_esb_config.mode                     = NRF_ESB_MODE_PTX;
    nrf_esb_config.selective_auto_ack       = false;

    err_code = nrf_esb_init(&nrf_esb_config);

    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_0(base_addr_0);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(addr_prefix, 8);
    VERIFY_SUCCESS(err_code);
	
	if(FRQ_channel != -1)
		nrf_esb_set_rf_channel(FRQ_channel);
	else
		nrf_esb_set_rf_channel(2);
	

    return err_code;
}

//-----------------------END-OF-RADIO--------------------------