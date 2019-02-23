
#ifndef PROCESS_H
#define PROCESS_H

nrf_esb_payload_t new_packet_payload;

extern bool HALTED;

void handle_NRF_RX_packet(){
	switch(new_packet_payload.data[2]){
		case 0x06:
			HALTED = true;
			break;
		case 
		
	}
}


#endif //PROCESS_H