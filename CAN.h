#ifndef CAN_H
#define CAN_H

#include <TM4C123.h>

//*******************************************************************
//            (1) Macros for CAN module
//*******************************************************************

										//					CONTROLLER

// _-------------- CAN Control CANCTL Register-------------------

#define CAN_Test (1<<7)                          // test Mode Enable
#define CAN_CCE (1<<6)                          // Configuration Change Enable
#define CAN_DAR (1<<5)                          // Disable Automatic-Retransmission
#define CAN_EIE (1<<3)                          // Error Interrupt Enable
#define CAN_SIE (1<<2)                          // Status Interrupt Enable
#define CAN_IE (1<<1)                          //  Interrupt Enable
#define CAN_INIT (1<<0)                          //  Initialization

// _-------------- CAN Status (CANSTS) Register-------------------

  // READ ONLY 
#define CAN_BOFF (1<<7)                          // Bus-Off Status
#define CAN_EWARN (1<<6)                          // Warning Status
#define CAN_EPASS (1<<5)                          // Error Passive

// READ AND WRITE
#define CAN_RXOK (1<<4)                          // Received a Message Successfully
#define CAN_TXOK (1<<3)                          // Transmitted a Message Successfully

// _-------------- CAN Error Counter (CANERR) Register-------------------

#define CAN_Received_Error_Passive (1<<3)                          // Received Error Passive  (READ ONLY)

// _-------------- CAN Test (CANTST) Register-------------------

#define CAN_Recieve_observation (1<<7)                          // READ ONLY
#define CAN_LoopBack (1<<4)                          // Loopback Mode
#define CAN_SILENT (1<<3)                          // SILENT Mode
#define CAN_BASIC (1<<2)                          // basic Mode

//_______________________________________________________________________________
										//					INTERFACE

// _-------------- CAN IF Command Request (CANIF#CRQ) Register-------------------

 #define CAN_IF_Busy (1<<15)                          // Busy Flag
         // 5:0 bits are the message number needed to be transferred

// _-------------- CAN IFn Command Mask (CANIFnCMSK) Register-------------------
	

 #define CAN_IF_Write_Not_Read (1<<7)                  // specifies the direction of data transfer
 #define CAN_IF_ACC_MASK     (1<<6)                  // Access Mask Bits
 #define CAN_IF_ACC_ARB     (1<<5)                  // Access Arbitration Bits
 #define CAN_IF_ACC_CTRL     (1<<4)                  // Access Control Bits
 #define CAN_IF_ACC_CLRINTPND     (1<<3)                  // Clear Interrupt Pending Bit
 #define CAN_IF_ACC_NEWDAT_TXRQST      (1<<2)                  
 #define CAN_IF_ACC_DATAA     (1<<1)                  // Access Data Byte 0 to 3
 #define CAN_IF_ACC_DATAB     (1<<0)                  // Access Data Byte 4 to 7

// _-------------- CAN IF# Mask 2 (CANIF1MSK2) Register-------------------
 
  #define CAN_MASK_MXTD    (1<<15)                  // Mask Extended Identifier
  #define CAN_MASK_MDIR    (1<<14)                  // Mask Message Direction


 // _-------------- CAN IF# Arbitration 2 (CANIF1ARB2) Register-------------------

  #define CAN_ARB_MSGVAL    (1<<15)                  // Message Valid
  #define CAN_ARB_XTD    (1<<14)                  // Extended Identifier
  #define CAN_ARB_DIR    (1<<13)                  // Message Direction

 // _-------------- CAN IF# Message Control (CANIF1MCTL) Register-------------------

#define CAN_IF_MCTL_NEWDAT    (1<<15)                  //New Data
#define CAN_IF_MCTL_MSGLST    (1<<14)                  //Message Lost
#define CAN_IF_MCTL_INTPND    (1<<13)                  //Interrupt Pending
#define CAN_IF_MCTL_UMASK    (1<<12)                  // Use Acceptance Mask
#define CAN_IF_MCTL_TXIE    (1<<11)                  // Transmit Interrupt Enable
#define CAN_IF_MCTL_RXIE   (1<<10)                  // Receive Interrupt Enable
#define CAN_IF_MCTL_RMTEN    (1<<9)                  // Remote Enable
#define CAN_IF_MCTL_TXRQST    (1<<8)                  // Transmit Request
#define CAN_IF_MCTL_EOB    (1<<7)                  // End of Buffer

// define peripheral interrupts registers

#define Core_Peripherals_Base 0xE000E000

#define NVICEN0 *((volatile uint32_t*)(Core_Peripherals_Base + 0x100))
#define NVICEN1 *((volatile uint32_t*)(Core_Peripherals_Base + 0x104))
#define NVICDIS0 *((volatile uint32_t*)(Core_Peripherals_Base + 0x180))
#define NVICDIS1 *((volatile uint32_t*)(Core_Peripherals_Base + 0x184))
	
//________________________________________________________________________________________


//******************************************************************
//      (2)Data structures for CAN module
//******************************************************************


typedef struct 
{
	uint16_t BaudRatePrescaler;
	uint8_t  SJW;
	uint8_t  TSEG1;
	uint16_t  TSEG2;

}CAN_bitiming;


typedef enum
{
bit11 = 0,
bit29 = 1,
}ID_Extension;


typedef struct 
{
	
	CAN_bitiming BitTime;
	ID_Extension  idxtd;
	uint32_t mask[32]; 
	uint32_t ID [32]; 
	uint8_t DataFrameSize;
	
}CAN_configParameters;

typedef struct 
{
CAN0_Type *CANx;
uint8_t   PeripherlaNumber;
CAN_configParameters config;
uint16_t pTx[64];
uint8_t Txcounter;
uint8_t Tx_length;
uint16_t pRx[64];
uint8_t Rx_length;
uint8_t RxCounter;
void (*DataTransmitted_CallBack) (void* canhandler,uint16_t* txBuff);
void (*DataReceived_CallBack)    (void* canhandler,uint16_t* rxBuff);
}CAN_handler;


//**********************************************************************
//                      (3) CAN API
//**********************************************************************

void CAN_Init(CAN_handler *canHandler);

void CAN_Enable (CAN_handler *canHandler);
void CAN_Disable (CAN_handler *canHandler);

void CAN_Interrupt_Enable  (CAN_handler *canHandler);
void CAN_Interrupt_Disable (CAN_handler *canHandler);

void CAN_Transmit(CAN_handler *canHandler ,uint16_t* pTxBuffer,uint8_t dataLength, uint8_t Msg_number );
void CAN_Recieve(CAN_handler *canHandler ,uint16_t* pRxBuffer,uint8_t* dataLength );

void CAN_Configure_FIFO(CAN_handler *canHandler ,uint8_t buffer_Length, uint8_t Msg_number );

void CAN_txDone_RegisterCB(CAN_handler *canHandler ,void (*DataTransmitted_CallBack) (void* canhandler,uint16_t* txBuff) );
void CAN_rxDone_RegisterCB(CAN_handler *canHandler ,void (*DataReceived_CallBack) (void* canhandler,uint16_t* rxBuff) );

void CAN_txDone_UnRegisterCB(CAN_handler *canHandler  );
void CAN_rxDone_UnRegisterCB(CAN_handler *canHandler  );

//                   Testing Modes
void CAN_SilentMode_Enable (CAN_handler *canHandler);
void CAN_SilentMode_disable (CAN_handler *canHandler);

void CAN_LoopbackMode_Enable (CAN_handler *canHandler);
void CAN_LoopbackMode_disable (CAN_handler *canHandler);

void CAN_Loopback_Silent_Enable (CAN_handler *canHandler);
void CAN_Loopback_Silent_disable (CAN_handler *canHandler);

void CAN_BasicMode_Enable (CAN_handler *canHandler);
void CAN_BasicMode_disable (CAN_handler *canHandler);
void CAN_BasicMode_Recieve (CAN_handler *canHandler ,uint16_t* pRxBuffer,uint8_t* dataLength );
void CAN_BasicMode_Transmit(CAN_handler *canHandler ,uint16_t* pTxBuffer,uint8_t dataLength);


//     _____________________________________________


//________________________CAN Interrupt service routine functions_______________________

void CAN0_Handler (void);
void CAN1_Handler (void);
void CAN2_Handler (void);
//_______________________________________________________________________________
#endif
