/**
 *  @file can_handling.c
 *
 *  @author Tobias Groll
 */
#define STM32F1
//libopenstm32 includes
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/nvic.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/scb.h>
//config include
#include "can_config.h"
//FreeRTOS includes
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include "task.h"
//can_handling include
#include "can_handling.h"
#include "gpio_ext.h"

xSemaphoreHandle xSemaphoreCan;
/**
 * Init the CAN System
 */
void can_setup(void)
{
	/* Enable peripheral clocks. */
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_AFIOEN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPEN_MASK);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_CAN1EN);
#if (CAN_PORT == CAN2 || USE_BOTH_CAN == 1)
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_CAN2EN);
#endif

	/*Remap Pins */
#ifdef REMAP_CAN
		/* Remap CAN PINS*/
		gpio_port_remap(CAN_REMAP_OPTION);
#endif

#if (REMAP_CAN2 == 1)
		gpio_port_remap(CAN2_REMAP_OPTION);
#endif

	/*Set GPIO Modes */
#if (CAN_PORT == CAN1 || USE_BOTH_CAN == 1)
	/* Configure CAN pin: RX (input pull-up). */
	gpio_set_mode(CAN1_GPIO_PORT, GPIO_MODE_INPUT,
				      GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_CAN_RX_PIN);

	/* Configure CAN pin: TX. */
	gpio_set_mode(CAN1_GPIO_PORT, GPIO_MODE_OUTPUT_50_MHZ,
				      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_CAN_TX_PIN);
#endif
#if (CAN_PORT == CAN2 || USE_BOTH_CAN == 1)
	/* Configure CAN pin: RX (input pull-up). */
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
				      GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_CAN2_RX_PIN);

	/* Configure CAN pin: TX. */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
				      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_CAN2_TX_PIN);
#endif

	/*enable Interrupt*/
#if (ENABLE_CAN_RECEIVE_INTERRUPT == 1)
	//set the nvic priority group
	//scb_set_aircr_prigroup(SCB_AIRCR_PRIGROUP_GROUP16_NOSUB); /* TODO: What happens here? Function was removed from library */
	uint32_t scb_aircr = SCB_AIRCR;
	scb_aircr = ~(0xFFFF << 16) & scb_aircr;
	scb_aircr = scb_aircr | SCB_AIRCR_VECTKEY | (SCB_AIRCR_PRIGROUP_GROUP16_NOSUB << SCB_AIRCR_PRIGROUP_LSB);
	SCB_AIRCR = scb_aircr;
	/* NVIC setup. */
	//set priority, configMAX_SYSCALL_INTERRUPT_PRIORITY is the highest priority interrupts can have which calls freertos functions
#if (CAN_PORT == CAN1 || USE_BOTH_CAN == 1)
	nvic_set_priority(NVIC_USB_LP_CAN_RX0_IRQ, configMAX_SYSCALL_INTERRUPT_PRIORITY);
	nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
#endif
#if (CAN_PORT == CAN2 || USE_BOTH_CAN == 1)
	nvic_set_priority(NVIC_CAN2_RX0_IRQ, configMAX_SYSCALL_INTERRUPT_PRIORITY);
	//enable interrupt
	nvic_enable_irq(NVIC_CAN2_RX0_IRQ);

#endif
#endif

	/*initialize CAN Ports */
#if (CAN_PORT == CAN1 || USE_BOTH_CAN == 1)
	/* Reset CAN. */
	can_reset(CAN1);

	/* CAN cell init. */
	if (can_init(CAN1,
		     false,           /* TTCM: Time triggered comm mode? */
		     true,            /* ABOM: Automatic bus-off management? */
		     false,           /* AWUM: Automatic wakeup mode? */
		     false,           /* NART: No automatic retransmission? */
		     false,           /* RFLM: Receive FIFO locked mode? */
		     false,           /* TXFP: Transmit FIFO priority? */
		     CAN_BTR_SJW_4TQ,
		     CAN_BTR_TS1_12TQ,
		     CAN_BTR_TS2_5TQ,
		     PRESCALER_CAN1))             /* BRP+1: Baud rate prescaler */
	{

		/* Die because we failed to initialize. */
		while (1)
			__asm__("nop");
	}

	/* CAN filter 0 init. */
	can_filter_id_mask_32bit_init(CAN1,
				0,     /* Filter ID */
				0,     /* CAN ID */
				0,     /* CAN ID mask */
				1,     /* FIFO assignment (here: FIFO1) */
				true); /* Enable the filter. */
#endif
#if (CAN_PORT == CAN2 || USE_BOTH_CAN == 1)
	/* Reset CAN. */
		can_reset(CAN2);

		/* CAN cell init. */
		if (can_init(CAN2,
			     false,           /* TTCM: Time triggered comm mode? */
			     true,            /* ABOM: Automatic bus-off management? */
			     false,           /* AWUM: Automatic wakeup mode? */
			     false,           /* NART: No automatic retransmission? */
			     false,           /* RFLM: Receive FIFO locked mode? */
			     false,           /* TXFP: Transmit FIFO priority? */
			     CAN_BTR_SJW_4TQ,
			     CAN_BTR_TS1_12TQ,
			     CAN_BTR_TS2_5TQ,
			     PRESCALER_CAN2))             /* BRP+1: Baud rate prescaler */
		{

			/* Die because we failed to initialize. */
			while (1)
				__asm__("nop");
		}

	uint8_t can2FilterStart = 16;
	CAN_FMR(CAN1) &= ~(CAN_FMR_CAN2SB_MASK);
	CAN_FMR(CAN1) |= can2FilterStart << 8;
	uint32_t canfmr = CAN_FMR(CAN1);
	/* CAN filter 0 init. */
	can_filter_id_mask_32bit_init(CAN2,
			can2FilterStart,     /* Filter ID */
				0,     /* CAN ID */
				0,     /* CAN ID mask */
				0,     /* FIFO assignment (here: FIFO0) */
				true); /* Enable the filter. */
#endif
//	xSemaphoreCan = xSemaphoreCreateCounting(1,0);
//	xSemaphoreGive(xSemaphoreCan);
}

/**
 * reads the data from can
 *
 * \param length 	filled with the length of the received data
 * \param data		filled with the received data
 * \param canId		filled with the canId of the receiver
 */
void readCan(uint8_t *length, uint8_t *data, uint32_t *canId){
	 readCanPort(CAN_PORT, length, data, canId);
}

void sendCan(uint8_t length, uint8_t *data, uint32_t canId, bool waitForFreeMessageBox){
	 sendCanPort(CAN_PORT, length, data, canId, waitForFreeMessageBox);
}

/**
 * reads the data from can2
 *
 * \param length 	filled with the length of the received data
 * \param data		filled with the received data
 * \param canId		filled with the canId of the receiver
 */
void readCanPort(u32 can_port, uint8_t *length, uint8_t *data, uint32_t *canId){
//	data = (uint8_t *)"Test123";
//	*canId = (uint32_t)37;
//	*length = 7;
//	while(xSemaphoreTake(xSemaphoreCan, 2) != pdTRUE){
//		taskYIELD();
//	}
	u32 fmi;
	bool ext, rtr;
	u32 dataLength;

	can_receive(can_port,   //CAN Interface
				0, 		//FIFO queue id (0 or 1)
				true,   //release after receiving the message decrease the waiting message register
				canId,  //CAN ID of the sending knot
				&ext,   //shows if the can id is an extended one
				&rtr,   //shows if the is transmit request flag set
				&fmi,   //gets filter match id
				(u8 *)&dataLength, //length of the received data
				data);  //the data
	*length = (uint8_t)dataLength;
//	xSemaphoreGive(xSemaphoreCan);
}

void sendCanPort(u32 can_port, uint8_t length, uint8_t *data, uint32_t canId, bool waitForFreeMessageBox){
//	while(xSemaphoreTake(xSemaphoreCan, 2) != pdTRUE){
//		taskYIELD();
//	}
	int ret = -1;
	int tsr;
	int can = 0;
	uint32_t msr = CAN_MSR(can_port);
	uint32_t esr = CAN_ESR(can_port);
	if(can_port == CAN1){
		can = 1;
	} else if(can_port == CAN2){
		can = 2;
	}
	ret = can_transmit(can_port, canId, false, false, length,data);
	if(waitForFreeMessageBox){
		while(ret == -1){
			ret = can_transmit(can_port, canId, false, false, length,data);
			tsr = CAN_TSR(can_port);
			msr = CAN_MSR(can_port);
			esr = CAN_ESR(can_port);
			if(ret == -1){
				ret = -1;
			}
		}
	}

//	xSemaphoreGive(xSemaphoreCan);
}

uint8_t getWaitingCanMessages(){
	return getWaitingCanMessagesPort(CAN_PORT);
}

uint8_t getWaitingCanMessagesPort(u32 can_port){
	return (uint8_t)(CAN_RF0R(can_port) & CAN_RF0R_FMP0_MASK);
//	return 1;
}

