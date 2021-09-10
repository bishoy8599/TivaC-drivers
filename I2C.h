  
#ifndef I2C_H
#define I2C_H
#include <TM4C123.h>


//*******************************************************************
//            (1) Macros for I2C module

//Spi peripheral number
#define I2C_0  (0)
#define I2C_1  (1)
#define I2C_2  (2)
#define I2C_3  (3)

// _______________________MASTER______________________________________


// _--------------MCS (RO)  control/status Register-------------------

#define I2C_BUSY	    (1 << 0 )     
#define I2C_ERROR     (1 << 1 )
#define I2C_ADRACK	  (1 << 2 )		// Acknowlegde address
#define I2C_DATACK    (1 << 3 )   // Acknowlegde data
#define I2C_ARBLST    (1 << 4 )   // Arbitration Lost
#define I2C_IDLE      (1 << 5 )   
#define I2C_BUSBSY    (1 << 6 )   // Bus Busy
#define I2C_CLKTO     (1 << 7 )   // Clock Timeout Error

//_--------------MCS (WO) control/status Register-------------------

#define I2C_RUN	    (1 << 0)		// I2C Master Enable
#define I2C_START	  (1 << 1)		// Generate START
#define I2C_STOP	  (1 << 2)		// Generate STOP
#define I2C_ACK	    (1 << 3)		// Data Acknowledge Enable
#define I2C_HS	    (1 << 4)		// High-Speed Enable

//_---------------- Time Period (MTPR) Register-------------

#define I2C_MTPR_HS	    (1 << 7)		// High-Speed Enable

//_-------------- Interrupt Registers-------------------

#define I2C_CLKTO_INT	    (1 << 0)		// Clock Timeout Interrupt Mask
#define I2C_MASTER_INT	    (1 << 1)		// Master Interrupt Mask

/******************** Configuration (MCR) Register*********************/
#define I2C_LOOPBACK						(1 << 0 )				// Loopback mode enable
#define I2C_MASTER							(1 << 4 )				// Master mode enable
#define I2C_SLAVE								(1 << 5 )				// Slave mode enable
#define I2C_GLF_EN							(1 << 6 )				// Glitch filter enable

/******************** Bus Monitor (MBMON) Register*********************/
#define I2C_MASTER_SCL							(1 << 0 )				// SCL signal
#define I2C_MASTER_SDA							(1 << 1 )				// SDA signal

/******************** Configuration2 (MCR2) Register*********************/
#define I2C_GFPW_BYPASS							( 0 << 4 )				// Bypass
#define I2C_GFPW_1_CLK							( 1 << 4 )				// 1 clock wide
#define I2C_GFPW_2_CLK							( 2 << 4 )				// 2 clocks wide
#define I2C_GFPW_3_CLK							( 3 << 4 )				// 3 clocks wide
#define I2C_GFPW_4_CLK							( 4 << 4 )				// 4 clocks wide
#define I2C_GFPW_8_CLK							( 5 << 4 )				// 8 clocks wide
#define I2C_GFPW_16_CLK							( 6 << 4 )				// 16 clocks wide
#define I2C_GFPW_31_CLK							( 7 << 4 )				// 31 clocks wide


// _______________________SLAVE______________________________________

// _--------------SCSR (RO)  control/status Register-------------------

#define I2C_RREQ									( 1 << 0 )				// Receive request
#define I2C_TREQ									( 1 << 1 )				// Transmit request
#define I2C_FBR							      ( 1 << 2 )				// First byte receive
#define I2C_OAR2SEL				    		(1 << 3 )				// OAR2 Address Matched

/****************** SCSR (WO) Control/Status Register*********************/
#define I2C_SLAVE_ACTIVE						( 1 << 0 )				// Device Slave operation active

/***************************** Interrupts******************************/
#define I2C_INT_DATA								( 1 << 0 )				// Slave data interrupt
#define I2C_INT_START								( 1 << 1 )				// Start condition interrupt
#define I2C_INT_STOP								( 1 << 2 )				// Stop condition interrupt

/*****************************Slave ACK Control******************************/
#define I2C_ACKOEN								( 1 << 0 )				// ACK Override enable
#define I2C_ACKOVAL								( 1 << 1 )				// ACK Override value
//________________________________________________________________________________________________


#define I2C_PC_HS										( (uint32_t) 1 << 0 ) 			// Peripheral Configuration high speed




#define Core_Peripherals_Base 0xE000E000

#define NVICEN0 *((volatile uint32_t*)(Core_Peripherals_Base + 0x100))
#define NVICEN1 *((volatile uint32_t*)(Core_Peripherals_Base + 0x104))
#define NVICEN2 *((volatile uint32_t*)(Core_Peripherals_Base + 0x108))

#define NVICDIS0 *((volatile uint32_t*)(Core_Peripherals_Base + 0x180))
#define NVICDIS1 *((volatile uint32_t*)(Core_Peripherals_Base + 0x184))
#define NVICDIS2 *((volatile uint32_t*)(Core_Peripherals_Base + 0x188))


//*******************************************************************
//            (2) Data structures for I2C module

typedef struct {
	uint32_t MasterSlaveMode;						// Master or Slave
	uint32_t Loopback;					// Loop back mode
	uint32_t Clock_period;			// Clock period
	uint32_t High_speed;				// High speed mode enable
	uint32_t Glitch_filter;			// Glitch filter enable
	uint32_t GF_width;					// Filter width
	uint32_t Slave_own_address;	// Address of slave. Not set if master is selected
	uint32_t Dual_address_en;		// Dual address for slave enable
	uint32_t Dual_address;			// Dual address of slave
} i2c_Config;


typedef struct
{
	I2C0_Type		 *I2Cx;  /*!< I2C registers base address     */
	uint8_t      PeripherlaNumber;
	i2c_Config   config;       /*!< I2C communication parameters   */
	uint32_t		 Slave_addr; /*!< Slave address to transmit data   */
	char *pTx;
uint8_t Txcounter;
uint8_t Tx_length;
char *pRx;
uint8_t Rx_length;
uint8_t Rxcounter;
	uint32_t 				ErrorCode;  
void (*DataTransmitted_CallBack) (void* i2c_handler,char* txBuff);
void (*DataReceived_CallBack)    (void* i2c_handler,char* rxBuff);	
}i2c_handler;


//**********************************************************************
//                      (3) I2C API
//**********************************************************************

void i2c_init(i2c_handler *i2cx);

void i2c_Master_Transmit(i2c_handler *i2cx);
void i2c_Slave_Transmit(i2c_handler *i2cx);
void i2c_Master_Recieve(i2c_handler *i2cx);
void i2c_Slave_Recieve(i2c_handler *i2cx);

void i2c_Interrupt_Enable  (i2c_handler *i2cHandler);
void i2c_Interrupt_Disable (i2c_handler *i2cHandler);

void i2c_Slave_Interrupt_Enable(i2c_handler *i2cx);                //This function unmasks I2C slave interrupt request.
void i2c_irq_handler(i2c_handler *i2cx);                          // This function handles SPI interrupt request.

void i2c_txDone_RegsiterCB(i2c_handler *i2cHandler ,void (*DataTransmitted_CallBack) (void* i2chandler,char* ptr));
void i2c_rxDone_RegisterCB(i2c_handler *i2cHandler ,void (*DataReceived_CallBack) (void* i2chandler,char* ptr) );

void i2c_txDone_UnRegsiterCB(i2c_handler *i2cHandler  );
void i2c_rxDone_UnRegisterCB(i2c_handler *i2cHandler  );

#endif
