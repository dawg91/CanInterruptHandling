/*
 * SystemStatus.c
 *
 *  Created on: 12.04.2014
 *      Author:
 */
#define STM32F1

#include <libopencm3/stm32/can.h>
// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"
#include "portable.h"
#include "semphr.h"
//Library
#include "can_handling.h"
#include "SystemStatus.h"






xSemaphoreHandle xSemaphoreWaitingForCanReceive;



//Creates Semaphore, runs in main method before tasks created
void can_semaphore_init(){
	vSemaphoreCreateBinary(xSemaphoreWaitingForCanReceive);

}


//FreeRtos task to read CAN1
void SC_readCan1Data(void* pvPars){
	uint8_t canDataLength; //length of the data that will receive by can
	uint32_t canId = 0;
	uint8_t data[8]; //data that will receive by can

	portTickType now = xTaskGetTickCount();
	portTickType lastBmsReceive0 = now;






	can_enable_irq(CAN1, CAN_IER_FMPIE1);
	while(1){
		if(xSemaphoreTake(xSemaphoreWaitingForCanReceive, 100) == pdTRUE){
			while(getWaitingCanMessagesPort(CAN1) > 0){
				u32 fmi;
				bool ext, rtr;
				u32 dataLength;

				can_receive(CAN1, 1, true, &canId, &ext, &rtr, &fmi, (u8 *)&dataLength, data);
				canDataLength = (uint8_t)dataLength;
				switch(canId){
				//System Status Message
				case 0x1BC:
					if ((data[0] & 0xFE != 0) || (data[2] & 0x8 != 0)){
						isBms_error = 1;
					}
					else{
						isBms_error = 0;
					}

					lastBmsReceive0 = xTaskGetTickCount();
					break;

				case 0x666:
					if (BMS_Workaround == 0){
						BMS_Workaround = 1;
					}
					else{
						BMS_Workaround = 0;
					}
					break;

				}

			}
		}
		now = xTaskGetTickCount();
		if((now - lastBmsReceive0 > 7000)){
			isBms_error = 1;
		}
		can_enable_irq(CAN1, CAN_IER_FMPIE1);
	}
	//enable interrupt

}
//FreeRtos Task to read CAN2
void SC_readCan2Data(void* pvPars){
	uint8_t canDataLength; //length of the data that will receive by can
	uint32_t canId;
	uint8_t data[8]; //data that will receive by can








	can_enable_irq(CAN2, CAN_IER_FMPIE0);
	while(1){
		if(xSemaphoreTake(xSemaphoreWaitingForCanReceive, 100) == pdTRUE){
			while(getWaitingCanMessages > 0){
				readCan(&canDataLength, data, &canId);
				switch(canId){
				//System Status Message



				case 0x239:
					isBrakeLight = data[0];
					if(isBrakeLight != isBrakeLightBefore){
						if(isBrakeLight){
							gpio_set(GPIOA, GPIO6);
						} else {
							gpio_clear(GPIOA, GPIO6);
						}

					}
					isBrakeLightBefore = isBrakeLight;
					break;

				}

				//enable interrupt
				can_enable_irq(CAN_PORT, CAN_IER_FMPIE0);

				//vTaskDelay(1);
			}
		}
	}


}

//Interrupt Handler for CAN1
void usb_lp_can_rx0_isr(void)
{
	long xHigherPriorityTaskWoken = pdFALSE;
	//disable interrupt
	can_disable_irq(CAN1, CAN_IER_FMPIE1);
	//unlock Log Task
	xSemaphoreGiveFromISR(xSemaphoreWaitingForCanReceive,&xHigherPriorityTaskWoken);
	/* If xHigherPriorityTaskWoken was set to true you
	we should yield.  The actual macro used here is
	port specific. */
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

//Interrupt Handler for CAN2
void can2_rx0_isr(void)
{
	long xHigherPriorityTaskWoken = pdFALSE;
	//disable interrupt
	can_disable_irq(CAN_PORT, CAN_IER_FMPIE0);
	//unlock Log Task
	xSemaphoreGiveFromISR(xSemaphoreWaitingForCanReceive,&xHigherPriorityTaskWoken);
	/* If xHigherPriorityTaskWoken was set to true you
	we should yield.  The actual macro used here is
	port specific. */
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}



