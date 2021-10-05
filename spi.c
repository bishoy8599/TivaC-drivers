#include "spi.h"



//                   private data
//*******************************************************
static uint8_t interruptHandler[4]={7,34,57,58};
static SPI_handler *spihandler[4];
//  *****************************************************


//                   Static functions

static void SPI_Gpio_configure (uint8_t spiNumber);
static void SPI_MasterSlave_Set(SSI0_Type* SSIx_BASE,uint8_t MasterSlaveMode);
static void SPI_ClockSource_Set (SSI0_Type* SSIx,uint8_t source);
static void SPI_ClockDivider_Set (SSI0_Type* SSIx,uint8_t PrescalerValue);
static void SPI_FrameFormat_Set(SSI0_Type* SSIx,uint8_t FrameFormat);
static void SPI_ClockPolarityPhase_set(SSI0_Type* SSIx,uint8_t clockPolarityPhaseMode);
static void SPI_DataSize_set(SSI0_Type* SSIx,uint8_t datasize);
static void spi_delay(void);
static uint8_t SPI_InterruptStatus(SSI0_Type* SSIx,uint8_t interrupt);
static uint8_t SPI_MaskedInterruptStatus(SSI0_Type* SSIx,uint8_t interrupt);
static void SPI_InterruptClear(SSI0_Type* SSIx,uint8_t interrupt);
static void SPI_InterruptMask(SSI0_Type* SSIx,uint8_t interrupt);
static void SPI_InterruptUnMask(SSI0_Type* SSIx,uint8_t interrupt);



//************************************************************
//                   Driver functions


void SPI_Init(SPI_handler *spiHandler)
{
	(SYSCTL->RCGCSSI|=(1<<(spiHandler->PeripherlaNumber)));
	SPI_Gpio_configure(spiHandler->PeripherlaNumber);  
	SPI_Disable(spiHandler); 
	SPI_MasterSlave_Set(spiHandler->SSIx,spiHandler->config.MasterSlaveMode);
	SPI_ClockSource_Set(spiHandler->SSIx,spiHandler->config.ClkSource); 
	SPI_ClockDivider_Set(spiHandler->SSIx,spiHandler->config.PreScaler);  
	SPI_FrameFormat_Set(spiHandler->SSIx,spiHandler->config.FrameFormat);
  SPI_ClockPolarityPhase_set(spiHandler->SSIx,spiHandler->config.ClkPolarityPhaseMode);
  SPI_DataSize_set(spiHandler->SSIx,spiHandler->config.DataSize);
	spihandler[spiHandler->PeripherlaNumber]=spiHandler;
  SPI_Enable(spiHandler);
}

void SPI_Disable(SPI_handler *spiHandler){
	spiHandler->SSIx->CR1 &= ~SPI_EnableDisable_Bit;
}

void SPI_Enable(SPI_handler *spiHandler){
	spiHandler->SSIx->CR1 |= SPI_EnableDisable_Bit;
}
//___________________________________________________________//

void SPI_Transmit (SPI_handler *spiHandler,char* pTxBuffer,uint8_t dataLength)
{
	for(int i=0;i<dataLength;i++)
	{
while((spiHandler->SSIx->SR & (1<<1)) == 0);       
spiHandler->SSIx->DR = pTxBuffer[i];               
while(spiHandler->SSIx->SR & (1<<4));		
	}
}

void SPI_Receive   (SPI_handler *spiHandler,char* pRxBuffer,uint8_t dataLength)
{
	for(int i=0;i<dataLength;i++)
	{
while((spiHandler->SSIx->SR & (1<<2))==0);
pRxBuffer[spiHandler->RxCounter]=spiHandler->SSIx->DR;
spiHandler->RxCounter++;
spi_delay();		
	}
	while(((spiHandler->SSIx->SR & (1<<4) ) == 1));
}

//______________________________________________________________//

void SPI_Start_Trasnmit(SPI_handler *spiHandler,char* pTxBuffer,uint8_t dataLength)
{
	spiHandler->pTx=pTxBuffer;
	spiHandler->Tx_length=dataLength;
	spiHandler->Txcounter=0;
	SPI_Interrupt_Enable(spiHandler);
	if(dataLength<=FifoSize)
	{
	 for(int i=0;i<dataLength;i++)
		{
		spiHandler->SSIx->DR=pTxBuffer[i];
		spiHandler->Txcounter++;
		}
	}
	else if(dataLength>FifoSize)
	{
	for(int i=0;i<FifoSize;i++)
		{
		spiHandler->SSIx->DR=pTxBuffer[i];
		spiHandler->Txcounter++;
		}
	}
SPI_InterruptUnMask(spiHandler->SSIx,SPI_TxFifo_bit);
}


void TransmitFromInterrupt(SPI_handler *spiHandler)
{	
	if(SPI_InterruptStatus(spiHandler->SSIx,SPI_TxFifo_InterruptStatus))
	{
	uint8_t remainingBits=spiHandler->Tx_length-spiHandler->Txcounter;
	if(remainingBits!=0)
	{
	spiHandler->SSIx->DR=spiHandler->pTx[spiHandler->Txcounter];
	spiHandler->Txcounter++;
 }

else if(remainingBits==0)
	 { spiHandler->Txcounter=0;
	   SPI_InterruptMask(spiHandler->SSIx,SPI_TxFifo_bit);
		//add call back here
		 spiHandler->DataTransmitted_CallBack(&spiHandler,spiHandler->pTx);
	 }
 } 
}

void SPI_Start_Receive (SPI_handler *spiHandler,char* pRxBuffer,uint8_t dataLength)
{
    spiHandler->pRx=pRxBuffer;
	spiHandler->Rx_length=dataLength;
	spiHandler->RxCounter=0;
	SPI_InterruptUnMask(spiHandler->SSIx,SPI_RxFifo_bit);
	SPI_Interrupt_Enable(spiHandler);
}

void ReceiveFromInterrupt(SPI_handler *spiHandler) 
	{
 if(SPI_InterruptStatus(spiHandler->SSIx,SPI_RxFifo_InterruptStatus))
 {
	uint8_t bit2Rx;
	//rx fifo half full 
  for(int i=0;i<FifoSize/2;i++)
  {	
	spiHandler->pRx[spiHandler->RxCounter]=spiHandler->SSIx->DR;
	spiHandler->RxCounter++;
	}
	
bit2Rx=spiHandler->Rx_length-spiHandler->RxCounter;
	if(bit2Rx==0)
	{
	spiHandler->RxCounter=0;
	SPI_InterruptMask(spiHandler->SSIx,SPI_RxFifo_bit);
	spiHandler->DataReceived_CallBack(&spiHandler,spiHandler->pRx);
	}
}
}

void SPI_txDone_RegsiterCB(SPI_handler *spiHandler ,void (*DataTransmitted_CallBack) (void* spihandler,char* ptr) )
{
spiHandler->DataTransmitted_CallBack=DataTransmitted_CallBack;
}
void SPI_rxDone_RegisterCB(SPI_handler *spiHandler ,void (*DataReceived_CallBack) (void* spihandler,char* ptr) )
{
spiHandler->DataReceived_CallBack=DataReceived_CallBack;
}


void SPI_txDone_UnRegsiterCB(SPI_handler *spiHandler  )
{
spiHandler->DataTransmitted_CallBack=( (void*)0);
}
void SPI_rxDone_UnRegisterCB(SPI_handler *spiHandler  )
{
spiHandler->DataReceived_CallBack=( (void*)0);
}

// *********************************************************************
//********************************** INTERUPT SERVICE ROUTINE FUNCTIONS *********************

void SSI0_Handler (void)
{
	if(SPI_InterruptStatus(spihandler[0]->SSIx,SPI_TxFifo_InterruptStatus))
	{
    TransmitFromInterrupt(spihandler[0]);	
	}
	
	else if(SPI_InterruptStatus(spihandler[0]->SSIx,SPI_RxFifo_InterruptStatus))
	{
ReceiveFromInterrupt(spihandler[0]);	
	}
	
}

void SSI1_Handler (void)
{
	if(SPI_InterruptStatus(spihandler[1]->SSIx,SPI_TxFifo_InterruptStatus))
	{
    TransmitFromInterrupt(spihandler[1]);	
	}
	
	else if(SPI_InterruptStatus(spihandler[1]->SSIx,SPI_RxFifo_InterruptStatus))
	{
ReceiveFromInterrupt(spihandler[1]);	
	}
	
}

void SSI2_Handler (void)
{
	if(SPI_InterruptStatus(spihandler[2]->SSIx,SPI_TxFifo_InterruptStatus))
	{
    TransmitFromInterrupt(spihandler[2]);	
	}
	
	else if(SPI_InterruptStatus(spihandler[2]->SSIx,SPI_RxFifo_InterruptStatus))
	{
ReceiveFromInterrupt(spihandler[2]);	
	}
	
}

void SSI3_Handler (void)
{
	if(SPI_InterruptStatus(spihandler[3]->SSIx,SPI_TxFifo_InterruptStatus))
	{
    TransmitFromInterrupt(spihandler[3]);	
	}
	
	else if(SPI_InterruptStatus(spihandler[3]->SSIx,SPI_RxFifo_InterruptStatus))
	{
ReceiveFromInterrupt(spihandler[3]);	
	}
	
}
// *********************************************************************
//********************************** STATIC FUNCTIONS *********************

	static void SPI_Gpio_configure (uint8_t spiNumber)
{	
	switch(spiNumber)
	{
		case SPI_0:
			//gpio pins is set by default as spi
		SYSCTL->RCGCGPIO |= (1<<0);     //CLOCK ENABLE TO PORT A
		GPIOA->DEN |= ((1<<2)|(1<<3)|(1<<4)|(1<<5));    // PORT A DIGITAL ENABLE
			break;
	
		case SPI_1:
			
		SYSCTL->RCGCGPIO |= (1<<3);     //CLOCK ENABLE TO PORT D
		GPIOD->AFSEL |= (1<<0)|(1<<1)|(1<<2)|(1<<3);    //SETTING GPIO PINS TO ALTERNATE FUNTION
		GPIOD->PCTL |= ((0x2<<0) | (0x2<<4) | (0x2<<8) | (0x2<<12));
		GPIOD->DEN |= ((1<<0) | (1<<1) | (1<<2) | (1<<3)) ;
			break;
	
		case SPI_2:
		SYSCTL->RCGCGPIO |= (1<<1);     //CLOCK ENABLE TO PORT A
		GPIOA->AFSEL |= (1<<4) | (1<<5) | (1<<6) | (1<<7);    //SETTING GPIO PINS TO ALTERNATE FUNTION
		GPIOA->PCTL |= ((0x2<<16) | (0x2<<20) | (0x2<<24) | (0x2<<28));		
		GPIOA->DEN |= ((1<<4) | (1<<5) | (1<<6) | (1<<7)) ; 
			break;
		
		case SPI_3:
		SYSCTL->RCGCGPIO |= (1<<3);            //CLOCK ENABLE TO PORT D
		GPIOD->AFSEL |= (1<<0)|(1<<1)|(1<<2)|(1<<3);    //SETTING GPIO PINS TO ALTERNATE FUNTION
GPIOD->PCTL |= ((0x1<<0) | (0x1<<4) | (0x1<<8) | (0x1<<12));		
		GPIOD->DEN |= ((1<<0) | (1<<1) | (1<<2) | (1<<3)) ;
			break;
		
		default:
			break;
	}
}
static void SPI_MasterSlave_Set(SSI0_Type* SSIx,uint8_t MasterSlaveMode)
{
if(MasterSlaveMode==SPI_Mode_Master)
{
	SSIx->CR1 &= ~SPI_MasterSlave_Bit;
}
else if(MasterSlaveMode==SPI_Mode_Slave)
{
	SSIx->CR1 |= SPI_MasterSlave_Bit;
}
}

static void SPI_ClockSource_Set (SSI0_Type* SSIx,uint8_t source)
{
if(source==SPI_sysclk_Source)
{
SSIx->CC=SPI_sysclk_Source;
}
else if(source==SPI_Piosc_Source)
{
SSIx->CC=SPI_Piosc_Source;
}
}


static void SPI_ClockDivider_Set (SSI0_Type* SSIx,uint8_t PrescalerValue)
{
	//SSInClk = SysClk / (CPSDVSR * (1 + SCR))
 if((PrescalerValue&0x1)==0)
 {
 if(PrescalerValue>=2 && PrescalerValue<=254)
 {
 SSIx->CPSR=PrescalerValue;
 }
else 
	{
  PrescalerValue=2;
	SSIx->CPSR=PrescalerValue;
	}		
}
 
 else if((PrescalerValue&0x1)==1)
 {
 	PrescalerValue+=1;
	SSIx->CPSR=PrescalerValue;
 }
}

static void SPI_FrameFormat_Set(SSI0_Type* SSIx,uint8_t FrameFormat)
{
switch(FrameFormat)
{
	case SPI_Freescale_Foramt:
		SSIx->CR0 &= ~SPI_Format_Bits;
	  SSIx->CR0 |= SPI_Freescale_Foramt_set;
		break;
	
case SPI_TI_Foramt:
	  SSIx->CR0 &= ~SPI_Format_Bits;
	  SSIx->CR0 |= SPI_TI_Foramt_set;
		break;
	
case SPI_MicroWire_Foramt:
   	SSIx->CR0 &= ~SPI_Format_Bits;
	  SSIx->CR0 |= SPI_MicroWire_Foramt_set;
		break;
	
default:
	break;
}
}

static void SPI_ClockPolarityPhase_set(SSI0_Type* SSIx,uint8_t clockPolarityPhaseMode)
{
switch(clockPolarityPhaseMode)
{

	case SPI_ClkPolarityPhase_Mode_1:
		SSIx->CR0&=~(SPI_ClockPhase_Bit|SPI_ClockPolarity_Bit);
		break;
	
	
	case SPI_ClkPolarityPhase_Mode_2:
	SSIx->CR0&=~(SPI_ClockPhase_Bit|SPI_ClockPolarity_Bit);
	SSIx->CR0|=SPI_ClkPolarityPhase_Mode_2_set;
		break;
	
	case SPI_ClkPolarityPhase_Mode_3:
	SSIx->CR0&=~(SPI_ClockPhase_Bit|SPI_ClockPolarity_Bit);
	SSIx->CR0|=SPI_ClkPolarityPhase_Mode_3_set;
		break;
	
	case SPI_ClkPolarityPhase_Mode_4:
	SSIx->CR0&=~(SPI_ClockPhase_Bit|SPI_ClockPolarity_Bit);
	SSIx->CR0|=SPI_ClkPolarityPhase_Mode_4_set;
		break;
	
	default:
		break;
}
}

static void SPI_DataSize_set(SSI0_Type* SSIx,uint8_t datasize)
{
SSIx->CR0&=~(0x0000000F);
SSIx->CR0|=datasize;	
}

static void spi_delay(void)
{
for(int i=0;i<100000;i++)
	{}
}



static uint8_t SPI_InterruptStatus(SSI0_Type* SSIx,uint8_t interrupt)
{
return (SSIx->RIS&interrupt);
}
static void SPI_InterruptClear(SSI0_Type* SSIx,uint8_t interrupt)
{
SSIx->ICR&=~interrupt;
}

static void SPI_InterruptMask(SSI0_Type* SSIx,uint8_t interrupt)
{
SSIx->IM&=~interrupt;  /*mask interrupt*/
}

static void SPI_InterruptUnMask(SSI0_Type* SSIx,uint8_t interrupt)
{
SSIx->IM|=interrupt;  /*mask interrupt*/
}


static uint8_t SPI_MaskedInterruptStatus(SSI0_Type* SSIx,uint8_t interrupt)
{

return (SSIx->MIS&interrupt);
}

void SPI_Interrupt_Enable  (SPI_handler *spiHandler)
{
	switch (interruptHandler[spiHandler->PeripherlaNumber])
	{
		case 4:
			NVICEN0 |= (1<<4);
		break;
		
		case 34:
			NVICEN1 |= (1<<2);
		break;
		
		case 57:
			NVICEN1 |= (1<<24);
		break; 
		
		case 58:
			NVICEN1 |= (1<<25);
		break;
		
		default:
			break;
		}		
}

void SPI_Interrupt_Disable (SPI_handler *spiHandler)
{
switch (interruptHandler[spiHandler->PeripherlaNumber])
	{
		case 4:
			NVICDIS0 |= (1<<4);
		break;
		
		case 34:
			NVICDIS1 |= (1<<2);
		break;
		
		case 57:
			NVICDIS1 |= (1<<24);
		break; 
		
		case 58:
			NVICDIS1 |= (1<<25);
		break;
		
		default:
			break;
		}		
}