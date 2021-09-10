#include "CAN.h"



//                   private data
//*******************************************************
static uint8_t interruptHandler[3]={39,40,41};
static CAN_handler *canhandler[3];
//  *****************************************************

//                   Private functions
static void CAN_Gpio_configure (uint8_t canNumber);
static void CAN_BitTime_Set (CAN0_Type* canx_BASE,	CAN_bitiming BitTime);

static void CAN_Configure_Message_Transmit (CAN_handler *canHandler );
static void CAN_Configure_Message_Recieve (CAN_handler *canHandler );

static void CAN_IF1_Mask_ID_set (CAN_handler *canHandler, uint8_t Msg_number , uint8_t Transmit);
static void CAN_IF2_Mask_ID_set (CAN_handler *canHandler, uint8_t Msg_number , uint8_t Transmit);

static uint16_t CAN_Interrupt_ID_Get (CAN_handler *canHandler);
static void CAN_StatusInterruptHandling (CAN_handler *canHandler);

//*******************************************************************************
//                                       Driver functions
//*******************************************************************************


void CAN_Init(CAN_handler *canHandler)
{
	// This function configures the basic parameters (Bit timing / peripheral number/ interface parameters) from the parameters passed to it from CAN_handler struct
  // This function enables the can module 
	// parameters: canHandler : it is struct that contains base address of the can module and the parameters needed to be configured 
	// returns : None 
	
	(SYSCTL->RCGCCAN |= (1<<(canHandler->PeripherlaNumber)));               // ENABLE clock gating control to the can module
	CAN_Gpio_configure(canHandler->PeripherlaNumber);            // configure gpio pins related to the can module
	canHandler->CANx->CTL |= CAN_INIT;                  // enabling initialization and configuration change
	canHandler->CANx->CTL |= CAN_CCE; 
	CAN_BitTime_Set(canHandler->CANx , canHandler->config.BitTime);      // configuring bit time 
	
	CAN_Configure_Message_Transmit (canHandler);                                  // configuring interface 1 for transmitting and interface 2 for recieving
	CAN_Configure_Message_Recieve (canHandler);   
	
	canhandler[canHandler->PeripherlaNumber]=canHandler;

	canHandler->CANx->CTL &= ~CAN_INIT;
	canHandler->CANx->CTL &= ~CAN_CCE;                     // disable initialization and configuration change to return to normal operation
}


void CAN_Enable (CAN_handler *canHandler)
{
  // This function enables the can module 
	// parameters: canHandler : it is struct that contains base address of the can module and the parameters needed to be configured 
	// returns : None 
	
	canHandler->CANx->CTL &= ~CAN_INIT;
}

void CAN_Disable (CAN_handler *canHandler)
{
	// This function disables the can module 
	// parameters: canHandler : it is struct that contains base address of the can module and the parameters needed to be configured 
	// returns : None 
	
	canHandler->CANx->CTL |= CAN_INIT;
}

void CAN_Interrupt_Enable  (CAN_handler *canHandler)
{
	// This function enables the can interrupts enter the interrupt handler  
	// parameters: canHandler : it is struct that contains base address of the can module and the parameters needed to be configured 
	// returns : None 
	
	switch (interruptHandler[canHandler->PeripherlaNumber])
	{

		case 39:
			NVICEN1 |= (1<<7);
		break;
		
		case 40:
			NVICEN1 |= (1<<8);
		break; 
		
		case 41:
			NVICEN1 |= (1<<9);
		break;
		
		default:
			break;
		}		
	canHandler->CANx->CTL |= CAN_IE ;                               // enable interrupts
	canHandler->CANx->CTL |= CAN_EIE ;                              // enable error interrupts
	canHandler->CANx->CTL |= CAN_SIE ;                              // enable status interrupts
}

void CAN_Interrupt_Disable (CAN_handler *canHandler)
{
		// This function disables the can interrupts from entering the interrupt handler  
	// parameters: canHandler : it is struct that contains base address of the can module and the parameters needed to be configured 
	// returns : None 
switch (interruptHandler[canHandler->PeripherlaNumber])
	{
		
		case 39:
			NVICDIS1 |= (1<<7);
		break;
		
		case 40:
			NVICDIS1 |= (1<<8);
		break; 
		
		case 41:
			NVICDIS1 |= (1<<9);
		break;
		
		default:
			break;
		}
	canHandler->CANx->CTL &= ~CAN_IE ; 
}




void CAN_Transmit(CAN_handler *canHandler ,uint16_t* pTxBuffer,uint8_t dataLength, uint8_t Msg_number )
{
		// this function inserts the data needed to the data registers and initiates the transmission process 
	// parameters are:
	//             canhandler: the can handler that includes the base address of the can controllers and general configuration parameters
	//             pTxBuffer: is the data buffer needs to be transmitted
	//             dataLength: is the length of the data put in the pTxBuffer to be divided into frames and sent
	//             msg number: is the number of the message that has ID associated with the message content
	// returns : None 
	
	
	int counter = 0;
	while (counter < dataLength){
			
	    while (canHandler->CANx->IF1CRQ & CAN_IF_Busy);                    // wait till interface finish current operation
	    CAN_IF1_Mask_ID_set (canHandler , Msg_number, 1 );                        // set the ID and the mask
			canHandler->CANx->IF1CMSK |= CAN_IF_Write_Not_Read;                         // configuring the message object to transmit
		canHandler->CANx->IF1CMSK |= CAN_IF_ACC_DATAA | CAN_IF_ACC_DATAB;                  // enable access to arbitration bits
		
		  if (!pTxBuffer[counter]) { break; }                               
		 while(canHandler->CANx->IF1MCTL & CAN_IF_MCTL_TXRQST );                   //check whether transmission completed or not
		
		  if(pTxBuffer[counter]){                                           // checking existence of the next bytes then writing it in data registers
		   canHandler->CANx->IF1DA1 = pTxBuffer[counter];
			 counter ++;
		  }
		  if(pTxBuffer[counter]){
		    canHandler->CANx->IF1DA2 = pTxBuffer[counter];
			  counter ++;
		  }
		  if(pTxBuffer[counter]){
		    canHandler->CANx->IF1DB1 = pTxBuffer[counter];
			  counter ++;
		  }
		  if(pTxBuffer[counter]){
		    canHandler->CANx->IF1DB2 = pTxBuffer[counter];
			  counter ++;
		  }
		 
		  canHandler->CANx->IF1CRQ = Msg_number;                                        //set message number 
		  canHandler->CANx->IF1MCTL |= CAN_IF_MCTL_TXRQST;                      // start transmission
  }
				canHandler->DataTransmitted_CallBack(&canHandler,pTxBuffer);         // call the data transmitted callback function to indicate that transmission process is completed
 }			


void CAN_Recieve(CAN_handler *canHandler ,uint16_t* pRxBuffer,uint8_t* dataLength )
{
	// this function reads the data in the message recieved and put it data buffer 
	// parameters are:
	//             canhandler: the can handler that includes the base address of the can controllers and general configuration parameters
	//             pRxBuffer: is the data buffer that stores the messages data
	//             dataLength: is the length of the data recieved
	// NOTE:   All the fifo message objects take the ID and Mask of the first message objects configured in the handler configuration struct 
	// returns : None 
	
 while (1){     
  uint16_t INTID = CAN_Interrupt_ID_Get (canHandler)	; 
	 if( INTID == 0x0000 || INTID == 0x8000 ) {      // stop recieving when there is no new message or there is  Status Interrupt 
	  break;
	 }
	 
			int counter = 0;

	uint8_t Msg_number =  canHandler->CANx->INT;                                // assigning interrupt number to the message number         
	
		while (1){
			while (canHandler->CANx->IF2CRQ & CAN_IF_Busy);                    // wait till interface finish current operation
			CAN_IF2_Mask_ID_set (canHandler , Msg_number, 0 );                                             // set the ID and the mask
	
			canHandler->CANx->IF2CMSK = 0x007f ;  
			canHandler->CANx->IF2CRQ = Msg_number;                                 //  load the Message object parameters in the interface registers
	
			if (!(canHandler->CANx->IF2MCTL & CAN_IF_MCTL_NEWDAT)){                   // if there is no new data recieved?
				break;                                                                // stop and wait for another interrupt
			}
		
			*dataLength = (canHandler->CANx->IF2MCTL & (0xf));                    // assigns the number of bytes sent
	
			if(counter <= *dataLength){                                           // checking existence of the next bytes then writing it in data registers
				pRxBuffer[counter] =  canHandler->CANx->IF2DA1 ; 
				counter ++;
				}
			if(counter <= *dataLength){                                           // checking existence of the next bytes then writing it in data registers
				pRxBuffer[counter] =  canHandler->CANx->IF2DA2 ; 
			  counter ++;
		  }
		  if(counter <= *dataLength){                                           // checking existence of the next bytes then writing it in data registers
				pRxBuffer[counter] =  canHandler->CANx->IF2DB1 ; 
				counter ++;
		  }
		  if(counter <= *dataLength){                                           // checking existence of the next bytes then writing it in data registers
				pRxBuffer[counter] =  canHandler->CANx->IF2DB2 ; 
				counter ++;
				}
			if (!(canHandler->CANx->IF2MCTL & CAN_IF_MCTL_EOB)){                   // if this message is the end of buffer
				break;                                                                // stop and wait for another interrupt
			}else{
			Msg_number ++;
			}

		}
	}
	if( canHandler->CANx->INT == 0x0000)
	{
		canHandler->DataReceived_CallBack(&canHandler, pRxBuffer);            // call the data recieved callback function to indicate that reception process is completed
	}
}

void CAN_Configure_FIFO(CAN_handler *canHandler ,uint8_t buffer_Length, uint8_t Msg_number )
{                      
	// this function initiate and configure the fifo buffer
	// parameters are:
	//             canhandler: the can handler that includes the base address of the can controllers and general configuration parameters
	//             buffer length: is the length of the fifo needed
	//             msg number: is the number of the first message needed in the fifo
	// NOTE:   All the fifo message objects take the ID and Mask of the first message objects configured in the handler configuration struct 
	// returns : None 
	
	for (uint8_t i=0; i < buffer_Length; i++){
	                              
    CAN_Configure_Message_Recieve (canHandler);                              // load recieve configuration to the message
		CAN_IF2_Mask_ID_set (canHandler, Msg_number , 0);                         // setting the ID and mask of the first message in this 
		if (i != buffer_Length-1){
	  canHandler->CANx->IF2MCTL &= ~CAN_IF_MCTL_EOB;                              // end of bufffer disable for all the fifo buffer except the last one
		}
		
		canHandler->CANx->IF2CMSK = 0x00ff ;		                //  load the parameters in the interface registers to the message objects
		canHandler->CANx->IF2CRQ = Msg_number + i;  
  
	}
}



void CAN_txDone_RegisterCB(CAN_handler *canHandler ,void (*DataTransmitted_CallBack) (void* canhandler,uint16_t* ptr) )
{
	// this function assigns callback function to be called in this layer when data transmitted
	// parameters are:
	//             canhandler: the can handler that includes the base address of the can controllers and general configuration parameters
	//             DataTransmitted_CallBack: is pointer to the function that should be implemented in the upper layers and assigned in this function 
	// returns : None 
	
canHandler->DataTransmitted_CallBack = DataTransmitted_CallBack;
}

void CAN_rxDone_RegisterCB(CAN_handler *canHandler ,void (*DataReceived_CallBack) (void* canhandler,uint16_t* rxBuff) )
{
		// this function assigns callback function to be called in this layer when data received
	// parameters are:
	//             canhandler: the can handler that includes the base address of the can controllers and general configuration parameters
	//             DataReceived_CallBack: is pointer to the function that should be implemented in the upper layers and assigned in this function 
	// returns : None 
	
	canHandler->DataReceived_CallBack = DataReceived_CallBack;
}

void CAN_txDone_UnRegisterCB(CAN_handler *canHandler  )
{
		// this function unregister callback function from being called in this layer when data transmitted
	// parameters are:
	//             canhandler: the can handler that includes the base address of the can controllers and general configuration parameters
	// returns : None 
	
canHandler->DataTransmitted_CallBack=( (void*)0);
}

void CAN_rxDone_UnRegisterCB(CAN_handler *canHandler  )
{
			// this function unregister callback function from being called in this layer when data received
	// parameters are:
	//             canhandler: the can handler that includes the base address of the can controllers and general configuration parameters
	// returns : None 
canHandler->DataReceived_CallBack = ( (void*)0);
}

void CAN_SilentMode_Enable (CAN_handler *canHandler)
{
	canHandler->CANx->CTL |= CAN_Test;            // enable test mode 
	canHandler->CANx->TST = CAN_SILENT;            // enable silent mode 
}

void CAN_SilentMode_disable (CAN_handler *canHandler)
{
	canHandler->CANx->TST &= ~CAN_SILENT;            // disable silent mode
  canHandler->CANx->CTL &= ~CAN_Test;            // disable test mode 	
}

void CAN_LoopbackMode_Enable (CAN_handler *canHandler)
{
	canHandler->CANx->CTL |= CAN_Test;            // enable test mode 
	canHandler->CANx->TST = CAN_LoopBack;            // enable LoopBack mode 
}

void CAN_LoopbackMode_disable (CAN_handler *canHandler)
{
	canHandler->CANx->TST &= ~CAN_LoopBack;            // disable silent mode
  canHandler->CANx->CTL &= ~CAN_Test;            // disable test mode 	
}
void CAN_Loopback_Silent_Enable (CAN_handler *canHandler)
{
	canHandler->CANx->CTL |= CAN_Test;            // enable test mode 
	canHandler->CANx->TST = CAN_LoopBack | CAN_SILENT;            // enable LoopBack and silent modes 
}
void CAN_Loopback_Silent_disable (CAN_handler *canHandler)
{
	canHandler->CANx->TST &= ~(CAN_LoopBack | CAN_SILENT);            // disable LoopBack and silent modes 
	canHandler->CANx->CTL &= ~CAN_Test;                            // disable test mode 
}
void CAN_BasicMode_Enable (CAN_handler *canHandler)
{
	canHandler->CANx->CTL |= CAN_Test;            // enable test mode 
	canHandler->CANx->TST = CAN_BASIC;            // enable basic mode 
}
void CAN_BasicMode_disable (CAN_handler *canHandler)
{
	canHandler->CANx->TST &= ~CAN_BASIC;            // disable basic mode
  canHandler->CANx->CTL &= ~CAN_Test;            // disable test mode 	
}

void CAN_BasicMode_Recieve (CAN_handler *canHandler ,uint16_t* pRxBuffer,uint8_t* dataLength )
{
	// this function reads the data recieved in the interface registers in basic Mode and put it data buffer 
	// parameters are:
	//             canhandler: the can handler that includes the base address of the can controllers and general configuration parameters
	//             pRxBuffer: is the data buffer that stores the messages data
	//             dataLength: is the length of the data recieved
	// returns : None 
	
	if (!(canHandler->CANx->TST & CAN_BASIC))                          // exit the function if the can module is not in basic mode
	{
		return; 
	}
	
	 while (1){     
  uint16_t INTID = CAN_Interrupt_ID_Get (canHandler)	; 
	 if( INTID == 0x0000 || INTID == 0x8000 ) {      // stop recieving when there is no new message or there is  Status Interrupt 
	  break;
	 }
	 
			int counter = 0;	
		while (1){
			while (canHandler->CANx->IF2CRQ & CAN_IF_Busy);                    // wait till interface finish current operation
	
			if (!(canHandler->CANx->IF2MCTL & CAN_IF_MCTL_NEWDAT)){                   // if there is no new data recieved?
				break;                                                                // stop and wait for another interrupt
			}
		
			*dataLength = (canHandler->CANx->IF2MCTL & (0xf));                    // assigns the number of bytes sent
	
			if(counter <= *dataLength){                                           // checking existence of the next bytes then writing it in data registers
				pRxBuffer[counter] =  canHandler->CANx->IF2DA1 ; 
				counter ++;
				}
			if(counter <= *dataLength){                                           // checking existence of the next bytes then writing it in data registers
				pRxBuffer[counter] =  canHandler->CANx->IF2DA2 ; 
			  counter ++;
		  }
		  if(counter <= *dataLength){                                           // checking existence of the next bytes then writing it in data registers
				pRxBuffer[counter] =  canHandler->CANx->IF2DB1 ; 
				counter ++;
		  }
		  if(counter <= *dataLength){                                           // checking existence of the next bytes then writing it in data registers
				pRxBuffer[counter] =  canHandler->CANx->IF2DB2 ; 
				counter ++;
				}

		}
	}
	if( canHandler->CANx->INT == 0x0000)
	{
		canHandler->DataReceived_CallBack(&canHandler, pRxBuffer);            // call the data recieved callback function to indicate that reception process is completed
	}
}


void CAN_BasicMode_Transmit (CAN_handler *canHandler ,uint16_t* pTxBuffer,uint8_t dataLength )
{
		// this function inserts the data needed to the data registers and initiates the transmission process in basic mode
	// parameters are:
	//             canhandler: the can handler that includes the base address of the can controllers and general configuration parameters
	//             pTxBuffer: is the data buffer needs to be transmitted
	//             dataLength: is the length of the data put in the pTxBuffer to be divided into frames and sent
	// returns : None 
	
		if (!(canHandler->CANx->TST & CAN_BASIC))                          // exit the function if the can module is not in basic mode
	{
		return; 
	}
	
	
	int counter = 0;
	while (counter < dataLength){
			
	    while (canHandler->CANx->IF1CRQ & CAN_IF_Busy);                    // wait till interface finish current operation
	    
		
		  if (!(counter < dataLength)) { break; }                               
		 while(canHandler->CANx->IF1MCTL & CAN_IF_MCTL_TXRQST );                   //check whether transmission completed or not
		
		  if(counter < dataLength){                                           // checking existence of the next bytes then writing it in data registers
		   canHandler->CANx->IF1DA1 = pTxBuffer[counter];
			 counter ++;
		  }
		  if(counter < dataLength){
		    canHandler->CANx->IF1DA2 = pTxBuffer[counter];
			  counter ++;
		  }
		  if(counter < dataLength){
		    canHandler->CANx->IF1DB1 = pTxBuffer[counter];
			  counter ++;
		  }
		  if(counter < dataLength){
		    canHandler->CANx->IF1DB2 = pTxBuffer[counter];
			  counter ++;
		  }
		 
		  canHandler->CANx->IF1MCTL |= CAN_IF_MCTL_TXRQST;                      // start transmission
  }
				canHandler->DataTransmitted_CallBack(&canHandler,pTxBuffer);         // call the data transmitted callback function to indicate that transmission process is completed
}			

// *********************************************************************
//********************************** INTERUPT SERVICE ROUTINE FUNCTIONS *********************
void CAN0_Handler (void)
{
		uint16_t INTID = CAN_Interrupt_ID_Get (canhandler[0]);
		switch(INTID)
	{
		
		case 0x8000:
			CAN_StatusInterruptHandling (canhandler[0]);
			break;
	
		case 0x000:
			break;
		
		default:
					CAN_Recieve(canhandler[0] ,canhandler[0]->pRx,&canhandler[0]->Rx_length );         // recieve the recieved message
			break;
	}
}

void CAN1_Handler (void)
{
		uint16_t INTID = CAN_Interrupt_ID_Get (canhandler[1]);
		switch(INTID)
	{
		
		case 0x8000:
			CAN_StatusInterruptHandling (canhandler[1]);
			break;
	
		case 0x000:
			break;
		
		default:
					CAN_Recieve(canhandler[1] ,canhandler[1]->pRx,&canhandler[1]->Rx_length );         // recieve the recieved message
			break;
	}
}

void CAN2_Handler (void)
{
		uint16_t INTID = CAN_Interrupt_ID_Get (canhandler[2]);
		switch(INTID)
	{
		
		case 0x8000:
			CAN_StatusInterruptHandling (canhandler[2]);
			break;
	
		case 0x000:
			break;
		
		default:
					CAN_Recieve(canhandler[2] ,canhandler[2]->pRx,&canhandler[2]->Rx_length );         // recieve the recieved message
			break;
	}
}
// *********************************************************************
//********************************** PRIVATE FUNCTIONS IMPLEMENTATION *********************

static void CAN_Gpio_configure (uint8_t canNumber)
	{	
	switch(canNumber)
	{
		
		case 0:
			
		SYSCTL->RCGCGPIO |= (1<<1);     //CLOCK ENABLE TO PORT B
		GPIOD->AFSEL |= (1<<4)|(1<<5);    //SETTING GPIO PINS TO ALTERNATE FUNTION
		GPIOD->PCTL |= ((0x8<<16) | (0x8<<20));
		GPIOD->DEN |= (1<<4)|(1<<5);
			break;
	
		case 1:
		SYSCTL->RCGCGPIO |= (1<<0);     //CLOCK ENABLE TO PORT A
		GPIOA->AFSEL |= (1<<0) | (1<<1);    //SETTING GPIO PINS TO ALTERNATE FUNTION
		GPIOA->PCTL |= ((0x8<<0) | (0x8<<4));		
		GPIOA->DEN |= (1<<0) | (1<<1);
			break;
		
		default:
			break;
	}
}
	
static void CAN_BitTime_Set(CAN0_Type* canx_BASE,	CAN_bitiming BitTime)
{
	
	//Baud Rate prescalar Set
	canx_BASE->BIT |= BitTime.BaudRatePrescaler & (0x3f);               // assinging the LSBs to the BIT register 
	canx_BASE->BRPE |= BitTime.BaudRatePrescaler & (0x3c0);               // assinging the MSBs to the BRPE register 
  //Synchronization Jump Width set
	canx_BASE->BIT |= ((BitTime.SJW & (0x3)) << 6);                
	//TSEG1 set
	canx_BASE->BIT |= ((BitTime.TSEG1 & (0xf)) << 8);
	//TSEG2 set
	canx_BASE->BIT |= ((BitTime.TSEG2 & (0x7)) << 12);

}

static void CAN_Configure_Message_Transmit(CAN_handler *canHandler )
	{
	
	canHandler->CANx->IF1CMSK |= CAN_IF_Write_Not_Read;                         // configuring the message object to transmit
	canHandler->CANx->IF1CMSK |= CAN_IF_ACC_MASK;                              // enable access to mask bits 
	canHandler->CANx->IF1CMSK |= CAN_IF_ACC_ARB;                               // enable access to arbitration bits
	canHandler->CANx->IF1CMSK |= CAN_IF_ACC_CTRL;                              // enable access to control bits
	canHandler->CANx->IF1CMSK |= CAN_IF_ACC_CLRINTPND;                               // enable access to arbitration bits
	canHandler->CANx->IF1CMSK |= CAN_IF_ACC_NEWDAT_TXRQST;                               // enable access to arbitration bits
	canHandler->CANx->IF1CMSK |= CAN_IF_ACC_DATAA | CAN_IF_ACC_DATAB;                               // enable access to arbitration bits
	 
	canHandler->CANx->IF1MCTL |= CAN_IF_MCTL_UMASK;                              // enable the acceptance mask 	
	canHandler->CANx->IF1MCTL |= CAN_IF_MCTL_TXIE;                              // enable the INTPND bit to be set after a successful transmission
	canHandler->CANx->IF1MCTL |= CAN_IF_MCTL_RMTEN;                            // enable automatic transmission using remote transmit request
	canHandler->CANx->IF1MCTL |= CAN_IF_MCTL_EOB;                              // end of bufffer enable
  canHandler->CANx->IF1MCTL |= (canHandler->config.DataFrameSize & (0xf));    // set the data frame size
}

static void CAN_Configure_Message_Recieve (CAN_handler *canHandler)
{
	
	canHandler->CANx->IF2CMSK |= CAN_IF_ACC_MASK;                              // enable access to mask bits 
	canHandler->CANx->IF2CMSK |= CAN_IF_ACC_ARB;                               // enable access to arbitration bits
	canHandler->CANx->IF2CMSK |= CAN_IF_ACC_CTRL;                              // enable access to control bits
	canHandler->CANx->IF2CMSK |= CAN_IF_ACC_CLRINTPND;                               // enable access to arbitration bits
	canHandler->CANx->IF2CMSK |= CAN_IF_ACC_NEWDAT_TXRQST;                               // enable access to arbitration bits
	canHandler->CANx->IF2CMSK |= CAN_IF_ACC_DATAA | CAN_IF_ACC_DATAB;                               // enable access to arbitration bits

	canHandler->CANx->IF2MCTL |= CAN_IF_MCTL_UMASK;                              // enable the acceptance mask 	
	canHandler->CANx->IF2MCTL |= CAN_IF_MCTL_RXIE;                              // enable the INTPND bit to be set after a successful Reception
	canHandler->CANx->IF2MCTL &= ~CAN_IF_MCTL_RMTEN;                            // to leave the TXRQST bit unchanged
	canHandler->CANx->IF2MCTL |= CAN_IF_MCTL_EOB;                              // end of bufffer enable
  canHandler->CANx->IF2MCTL |= (canHandler->config.DataFrameSize & (0xf));    // set the data frame size
}


static void CAN_IF1_Mask_ID_set (CAN_handler *canHandler, uint8_t Msg_number , uint8_t Transmit)
{
		if( canHandler->config.idxtd == bit11) {                                     // if 11-bit identifier used
		canHandler->CANx->IF1MSK2 |= (( canHandler->config.mask[Msg_number] & (0x7ff)) << 2);     // take the first 11 bit in the mask and put it in CAN IF1 Mask 2 register
		canHandler->CANx->IF1MSK2 |= CAN_MASK_MXTD | CAN_MASK_MDIR;                   // The extended identifier bit XTD and DIR are used for acceptance filtering
		
		canHandler->CANx->IF1ARB2 |= (( canHandler->config.ID[Msg_number] & (0x7ff)) << 2);     // take the first 11 bit in the ID and put it in CAN IF1 Arbitration 2 register
		canHandler->CANx->IF1ARB2 &= ~CAN_ARB_XTD;                              // indicating not extended id
			
			if(Transmit == 1){
		canHandler->CANx->IF1ARB2 |= CAN_ARB_DIR;                                 // transmitting direction
			}else if (Transmit == 0){
		canHandler->CANx->IF1ARB2 &= ~CAN_ARB_DIR;                                 // Recieving direction
			}
			
		canHandler->CANx->IF1ARB2 |= CAN_ARB_MSGVAL;                               // message is valid       

	}else if ( canHandler->config.idxtd == bit29){
		canHandler->CANx->IF1MSK1 |= (( canHandler->config.mask[Msg_number] & (0xffff)) << 0);        // take the first 16 bit in the mask and put it in CAN IF1 Mask 1 register
		canHandler->CANx->IF1MSK2 |= ( canHandler->config.mask[Msg_number] << 16) & (0x1fff);         // take the second 13 bit in  the mask and put it in CAN IF1 Mask 2 register
		canHandler->CANx->IF1MSK2 |= CAN_MASK_MXTD | CAN_MASK_MDIR;
		
		canHandler->CANx->IF1ARB1 |= (( canHandler->config.ID[Msg_number] & (0xffff)) << 0);        // take the first 16 bit in the ID and put it in CAN IF1 Arbitration 1 register
		canHandler->CANx->IF1ARB2 |= ( canHandler->config.ID[Msg_number] << 16) & (0x1fff);         // take the second 13 bit in  the ID and put it in CAN IF1 Arbitration 2 register
		canHandler->CANx->IF1ARB2 |= CAN_ARB_XTD;                              // indicating extended id
		
		if(Transmit == 1){
		canHandler->CANx->IF1ARB2 |= CAN_ARB_DIR;                                 // transmitting direction
			}else if (Transmit == 0){
		canHandler->CANx->IF1ARB2 &= ~CAN_ARB_DIR;                                 // Recieving direction
			}
			
		canHandler->CANx->IF1ARB2 |= CAN_ARB_MSGVAL;                               // message is valid       
		}
}

static void CAN_IF2_Mask_ID_set (CAN_handler *canHandler, uint8_t Msg_number_ID , uint8_t Transmit)
{
	
		if( canHandler->config.idxtd == bit11) {                                     // if 11-bit identifier used
		canHandler->CANx->IF2MSK2 |= (( canHandler->config.mask[Msg_number_ID] & (0x7ff)) << 2);     // take the first 11 bit in the mask and put it in CAN IF1 Mask 2 register
		canHandler->CANx->IF2MSK2 |= CAN_MASK_MXTD | CAN_MASK_MDIR;                   // The extended identifier bit XTD and DIR are used for acceptance filtering
		
		canHandler->CANx->IF1ARB2 |= (( canHandler->config.ID[Msg_number_ID] & (0x7ff)) << 2);     // take the first 11 bit in the ID and put it in CAN IF1 Arbitration 2 register
		canHandler->CANx->IF1ARB2 &= ~CAN_ARB_XTD;                              // indicating not extended id
			
			if(Transmit == 1){
		canHandler->CANx->IF1ARB2 |= CAN_ARB_DIR;                                 // transmitting direction
			}else if (Transmit == 0){
		canHandler->CANx->IF1ARB2 &= ~CAN_ARB_DIR;                                 // Recieving direction
			}
			
		canHandler->CANx->IF1ARB2 |= CAN_ARB_MSGVAL;                               // message is valid       

	}else if ( canHandler->config.idxtd == bit29){
		canHandler->CANx->IF2MSK1 |= (( canHandler->config.mask[Msg_number_ID] & (0xffff)) << 0);        // take the first 16 bit in the mask and put it in CAN IF1 Mask 1 register
		canHandler->CANx->IF2MSK2 |= ( canHandler->config.mask[Msg_number_ID] << 16) & (0x1fff);         // take the second 13 bit in  the mask and put it in CAN IF1 Mask 2 register
		canHandler->CANx->IF2MSK2 |= CAN_MASK_MXTD | CAN_MASK_MDIR;
		
		canHandler->CANx->IF2ARB1 |= (( canHandler->config.ID[Msg_number_ID] & (0xffff)) << 0);        // take the first 16 bit in the ID and put it in CAN IF1 Arbitration 1 register
		canHandler->CANx->IF2ARB2 |= ( canHandler->config.ID[Msg_number_ID] << 16) & (0x1fff);         // take the second 13 bit in  the ID and put it in CAN IF1 Arbitration 2 register
		canHandler->CANx->IF2ARB2 |= CAN_ARB_XTD;                              // indicating extended id
		
		if(Transmit == 1){
		canHandler->CANx->IF2ARB2 |= CAN_ARB_DIR;                                 // transmitting direction
			}else if (Transmit == 0){
		canHandler->CANx->IF2ARB2 &= ~CAN_ARB_DIR;                                 // Recieving direction
			}
			
		canHandler->CANx->IF2ARB2 |= CAN_ARB_MSGVAL;                               // message is valid       
		}
	}


static uint16_t CAN_Interrupt_ID_Get (CAN_handler *canHandler)
{
		return canHandler->CANx->INT ;
}	

static void CAN_StatusInterruptHandling (CAN_handler *canHandler)
{
   uint8_t Status_Int = canHandler->CANx->STS;                        // read CAN status register 
	
	if (Status_Int & CAN_BOFF)                             // if there is buss off error     
	{
		// implementation to deal with buss off error
	}
	if (Status_Int & CAN_EWARN)                             // if there is Warning Status   
	{
				// implementation to deal with Warning Status error
	}
	if (Status_Int & CAN_EPASS)                             // if there is Error Passive   
	{
				// implementation to deal with Error Passive
	}
	if (Status_Int & CAN_RXOK)                             // if there is message recieved successfully   
	{
		CAN_Recieve(canHandler ,canHandler->pRx,&canHandler->Rx_length );         // recieve the recieved message
		canHandler->CANx->STS &= ~CAN_RXOK;                                       // clear the rxok bit
	}
	if (Status_Int & CAN_TXOK)                             // if there is message transmitted successfully   
	{
		canHandler->CANx->STS &= ~CAN_TXOK;                                    // clear the rxok bit
	}
	canHandler->CANx->STS &= 0x000;                                          // clear the lec field in the status register
}

