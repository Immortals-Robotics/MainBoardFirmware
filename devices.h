//
//	Software Platform Generated File
//	--------------------------------
//

#ifndef _DEVICES_H
#define _DEVICES_H


 // instance id macros
#define DRV_I2CM_1			0
#define DRV_DEBUGIO			0
#define DRV_MOTORVEL			1
#define DRV_KICKIO			2
#define DRV_MOTORDIR			3
#define DRV_SERVOIO			4
#define DRV_M25PX0_1			0
#define DRV_PWM8_1			0
#define DRV_PWMX_1			0
#define DRV_PWMX_2			1
#define DRV_PWMX_3			2
#define DRV_PWMX_4			3
#define DRV_PWMX_D			4
#define DRV_SPI_2			0
#define DRV_SPI_1			1
#define WB_I2CM_1			0
#define DEBUGIO			0
#define MOTORVEL			1
#define KICK			2
#define SERVO			3
#define MOTORDIR			4
#define WB_PWM8_1			0
#define WB_PWMX_3			0
#define WB_PWMX_4			1
#define WB_PWMX_1			2
#define WB_PWMX_2			3
#define WB_PWMX_D			4
#define WB_SPI_2			0
#define SPI_FLASH			1

 // peripheral base address and interrupt macros
#define WB_I2CM_1_BASE_ADDRESS			0xFF0C0000
#define WB_I2CM_1_INTERRUPT_A 			10
#define DEBUGIO_BASE_ADDRESS			0xFF000000
#define MOTORVEL_BASE_ADDRESS			0xFF030000
#define KICK_BASE_ADDRESS			0xFF070000
#define SERVO_BASE_ADDRESS			0xFF080000
#define MOTORDIR_BASE_ADDRESS			0xFF090000
#define WB_PWM8_1_BASE_ADDRESS			0xFF0E0000
#define WB_PWM8_1_INTERRUPT_A 			12
#define WB_PWMX_3_BASE_ADDRESS			0xFF020000
#define WB_PWMX_3_INTERRUPT_A 			7
#define WB_PWMX_4_BASE_ADDRESS			0xFF040000
#define WB_PWMX_4_INTERRUPT_A 			8
#define WB_PWMX_1_BASE_ADDRESS			0xFF050000
#define WB_PWMX_1_INTERRUPT_A 			5
#define WB_PWMX_2_BASE_ADDRESS			0xFF060000
#define WB_PWMX_2_INTERRUPT_A 			6
#define WB_PWMX_D_BASE_ADDRESS			0xFF0A0000
#define WB_PWMX_D_INTERRUPT_A 			9
#define WB_SPI_2_BASE_ADDRESS			0xFF010000
#define SPI_FLASH_BASE_ADDRESS			0xFF0B0000

#endif
