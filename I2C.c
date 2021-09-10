#include "i2c.h"



//                   private data
//*******************************************************
static uint8_t interruptHandler[4]={8,37,68,69};
//  *****************************************************

//                   Static functions prototype 
//___________________________________________________________________________________

  static void I2C_CLK_ENABLE(uint8_t  module);
	static void I2C_Gpio_configure (uint8_t spiNumber);
	static void i2c_check_error(i2c_handler *i2cx);
	static void i2c_exception_handler(i2c_handler *i2cx);
  static void i2c_Master_HS_Transmit(i2c_handler *i2cx);

//______________________________________________________________________________________________

//************************* Driver functions*********************************

void i2c_init(i2c_handler *i2cx) {
	
	I2C_CLK_ENABLE(i2cx->PeripherlaNumber);
	I2C_Gpio_configure (i2cx->PeripherlaNumber);
	
	if(i2cx->config.MasterSlaveMode == I2C_MASTER) {                  //module configured as master
		
		i2cx->I2Cx->MCR |= I2C_MASTER;                              //enable master 
		i2cx->I2Cx->MTPR |= i2cx->config.Clock_period;               //// Set clock period
		
		if(i2cx->config.Loopback == I2C_LOOPBACK){
			i2cx->I2Cx->MCR |= I2C_LOOPBACK;
		}
	} else if(i2cx->config.MasterSlaveMode == I2C_SLAVE){                        //module configured as slave
	
	  i2cx->I2Cx->MCR |= I2C_MASTER;
	 	i2cx->I2Cx->SCSR |= I2C_SLAVE_ACTIVE;              //enable slave operation
	  i2cx->I2Cx->SOAR = i2cx->config.Slave_own_address;  //setting address of slave
		
		if(i2cx->config.Dual_address_en){                             // If dual address is needed, enable the functionality
		 i2cx->I2Cx->SOAR2 = i2cx->config.Dual_address;               // setting alternative address
			i2cx->I2Cx->SOAR2 |= (1<<7) ;                               //enable dual address          
		} else{
			i2cx->I2Cx->SOAR2 &= ~(1<<7);                                  //Disable dual address 
		}

	}
	
		if(i2cx->config.High_speed) {
			if(i2cx->I2Cx->PP & I2C_PC_HS) {                              // check the capability of high speed
				i2cx->I2Cx->PC |= I2C_PC_HS;                                     // enable high speed
		} else { 
			return;
		}
		
		if(i2cx->config.MasterSlaveMode == I2C_MASTER){
			i2cx->I2Cx->MTPR |= I2C_MTPR_HS;           
			i2cx->I2Cx->MCS |= I2C_HS;                         // set the master control register to enable high speed
		}
		
	}
}
			

void i2c_Master_Transmit(i2c_handler *i2cx){
	
	i2cx->I2Cx->MSA = ( i2cx->Slave_addr << 1 );              // Write slave address to MSA
	i2cx->I2Cx->MSA &= ~0x1;                                  //  set mode to transmit
	
	if ( i2cx->config.High_speed){													// enable high speed mode if needed
		i2c_Master_HS_Transmit(i2cx);
	}
	
	i2cx->I2Cx->MCS &= ~I2C_HS;                               // disable high speed mode bit

	
	i2cx->I2Cx->MDR = (*i2cx->pTx);                        // write the first byte of data
	i2cx->pTx ++;                                          // increment data array
	i2cx->Txcounter ++;                                    // increment data counter
	
	while(i2cx->I2Cx->MCS & I2C_BUSBSY);                  // Wait until i2c bus is available
	
	if(i2cx->Tx_length == 1){                                     // if there is only one byte send it then stop the process
		i2cx->I2Cx->MCS |= I2C_RUN; 
		i2cx->I2Cx->MCS |= I2C_START;
		i2cx->I2Cx->MCS |= I2C_STOP;
		
		while(i2cx->I2Cx->MCS & I2C_BUSY);                          // check for error
		i2c_check_error(i2cx);
		return;
	}                                                 // if there is more than one byte start and transmit without stopping
	
	
		i2cx->I2Cx->MCS |= I2C_RUN;
	  i2cx->I2Cx->MCS |= I2C_START;
	  i2cx->I2Cx->MCS &= ~I2C_STOP;
		
		do  {
			while(i2cx->I2Cx->MCS & I2C_BUSY);                      // wait to finish the byte 
			i2c_check_error(i2cx);                             // check for error
			
			i2cx->I2Cx->MDR = (*i2cx->pTx);                        // write the first byte of data
	    i2cx->pTx ++;                                          // increment data array
	    i2cx->Txcounter ++;                                    // increment data counter
			
			i2cx->I2Cx->MCS |= I2C_RUN;                             
			i2cx->I2Cx->MCS &= ~I2C_START; 
			i2cx->I2Cx->MCS &= ~I2C_STOP;
			}
		while (i2cx->Tx_length != i2cx->Txcounter) ;
		
			i2cx->I2Cx->MCS &= ~I2C_RUN;                             
			i2cx->I2Cx->MCS &= ~I2C_START;                               
			i2cx->I2Cx->MCS |= I2C_STOP;                          // Stop the transmission because all data transmitted
		
		while(i2cx->I2Cx->MCS & I2C_BUSY);
		i2c_check_error(i2cx);
		i2cx->DataTransmitted_CallBack(&i2cx, i2cx->pTx);	
}


void i2c_Slave_Transmit(i2c_handler *i2cx){
	
	while(i2cx->I2Cx->SCSR & I2C_TREQ){                       // if there is transmit request from master 
		i2cx->I2Cx->MDR = (*i2cx->pTx);                        // write the first byte of data
	    i2cx->pTx ++;                                          // increment data array
	    i2cx->Txcounter ++;
		
	if (i2cx->Tx_length == i2cx->Txcounter) {                   // if the transmit buffer is empty break the loop
		break; 
		}
	}
}

void i2c_Master_Recieve(i2c_handler *i2cx) {
	
	i2cx->I2Cx->MSA = ( i2cx->Slave_addr << 1 );              // Write slave address to MSA
	i2cx->I2Cx->MSA |= 0x1;                                  //  set mode to receive
	
	while(i2cx->I2Cx->MCS & I2C_BUSBSY);                         // Wait until i2c bus is available
	
	if(i2cx->Rx_length == 1){                       // If there is only one byte to recieve, send stop right after
		i2cx->I2Cx->MCS |= I2C_RUN;
		i2cx->I2Cx->MCS |= I2C_START;
		i2cx->I2Cx->MCS |= I2C_STOP;
		i2cx->I2Cx->MCS |= I2C_ACK;
		i2cx->I2Cx->MCS &= ~I2C_HS;
		
		while(i2cx->I2Cx->MCS & I2C_BUSY);                 // Wait until  I2Cx finish
		i2c_check_error(i2cx);								   					// Check for errors
		
		if (!(i2cx->I2Cx->MCS & I2C_ERROR))                     // if there is no error
				(*i2cx->pRx) = i2cx->I2Cx->MDR;             // Read the data byte from MDR

			return;
		}
	
		i2cx->I2Cx->MCS |= I2C_RUN;
		i2cx->I2Cx->MCS |= I2C_START;
		i2cx->I2Cx->MCS &= ~I2C_STOP;
		i2cx->I2Cx->MCS |= I2C_ACK;
		i2cx->I2Cx->MCS &= ~I2C_HS;
	
		do  {
			while(i2cx->I2Cx->MCS & I2C_BUSY);                      // wait to finish the byte 
			i2c_check_error(i2cx);
			
			i2cx->I2Cx->MDR = (*i2cx->pRx);                        // write the first byte of data
	    i2cx->pRx ++;                                          // increment data array
	    i2cx->Rxcounter ++;                                    // increment data counter
			
			i2cx->I2Cx->MCS |= I2C_RUN;
		  i2cx->I2Cx->MCS &= ~I2C_START;
	  	i2cx->I2Cx->MCS &= ~I2C_STOP;
	  	i2cx->I2Cx->MCS |= I2C_ACK;
	  	i2cx->I2Cx->MCS &= ~I2C_HS;
			
		} while (i2cx->Tx_length != i2cx->Txcounter);
			
		  i2cx->I2Cx->MCS &= ~I2C_RUN;
		  i2cx->I2Cx->MCS &= ~I2C_START;
	  	i2cx->I2Cx->MCS |= I2C_STOP;
	  	i2cx->I2Cx->MCS &= ~I2C_ACK;
	  	i2cx->I2Cx->MCS &= ~I2C_HS;
		
		while(i2cx->I2Cx->MCS & I2C_BUSY);
		i2c_check_error(i2cx);
		i2cx->DataReceived_CallBack(&i2cx, i2cx->pRx);
}


void i2c_Slave_Recieve(i2c_handler *i2cx) { 
	while(i2cx->I2Cx->SCSR & I2C_RREQ){                       // if there is transmit request from master 
		i2cx->I2Cx->MDR = (*i2cx->pRx);                        // write the first byte of data
	    i2cx->pRx ++;                                          // increment data array
	    i2cx->Rxcounter ++;

	if (i2cx->Rx_length == i2cx->Rxcounter) {                   // if the transmit buffer is empty break the loop
		break; 
		}
	}
}




	void i2c_Interrupt_Enable  (i2c_handler *i2cHandler)
{
	switch (interruptHandler[i2cHandler->PeripherlaNumber])
	{
		case 8:
			NVICEN0 |= (1<<8);
		break;
		
		case 37:
			NVICEN1 |= (1<<5);
		break;
		
		case 68:
			NVICEN1 |= (1<<4);
		break; 
		
		case 69:
			NVICEN1 |= (1<<3);
		break;
		
		default:
			break;
		}		
}

void i2c_Interrupt_Disable (i2c_handler *i2cHandler)
{
switch (interruptHandler[i2cHandler->PeripherlaNumber])
	{
		case 8:
			NVICDIS0 |= (1<<8);
		break;
		
		case 37:
			NVICDIS1 |= (1<<2);
		break;
		
		case 68:
			NVICDIS1 |= (1<<24);
		break; 
		
		case 69:
			NVICDIS1 |= (1<<25);
		break;
		
		default:
			break;
		}		
}


void i2c_Slave_Interrupt_Enable(i2c_handler *i2cx){
	i2cx->I2Cx->SIMR = 0x7;
}
	
void i2c_irq_handler(i2c_handler *i2cx){
	if(i2cx->I2Cx->SMIS & (I2C_INT_DATA | I2C_INT_START)){       // Response when there is data interrupt after start
		if(i2cx->I2Cx->SCSR & I2C_RREQ){
			i2c_Slave_Transmit(i2cx);
		} else if(i2cx->I2Cx->SCSR & I2C_TREQ) {
			i2c_Slave_Transmit(i2cx);
		}
		
	}
	
	if(i2cx->I2Cx->SMIS & I2C_INT_STOP){
		// Stop detected, everything is finished, clear interrupt
		i2cx->I2Cx->SICR |= (1<<0) | (1<<1) | (1<<2);
	}
}

void i2c_txDone_RegsiterCB(i2c_handler *i2cHandler ,void (*DataTransmitted_CallBack) (void* i2chandler,char* ptr) )
{
i2cHandler->DataTransmitted_CallBack=DataTransmitted_CallBack;
}
void i2c_rxDone_RegisterCB(i2c_handler *i2cHandler ,void (*DataReceived_CallBack) (void* i2chandler,char* ptr) )
{
i2cHandler->DataReceived_CallBack=DataReceived_CallBack;
}


void i2c_txDone_UnRegsiterCB(i2c_handler *i2cHandler  )
{
i2cHandler->DataTransmitted_CallBack=( (void*)0);
}
void i2c_rxDone_UnRegisterCB(i2c_handler *i2cHandler  )
{
i2cHandler->DataReceived_CallBack=( (void*)0);
}
//****************************************************************************************************************
//                   STATIC FUNCTIONS IMPLEMENTATION
//******************************************************************************************************************

static void I2C_CLK_ENABLE(uint8_t  module) {
	SYSCTL->RCGCI2C |= (0x01 << module);
}

static void I2C_Gpio_configure (uint8_t i2cNumber)
{	
	switch(i2cNumber)
	{
		case I2C_0:
		SYSCTL->RCGCGPIO |= (1<<1);     //CLOCK ENABLE TO PORT B
		GPIOB->AFSEL |= (1<<2) | (1<<3);    //SETTING GPIO PINS TO ALTERNATE FUNTION
		GPIOB->ODR |= (1<<3);               // settin SDA pin to open drain select
		GPIOB->PCTL |= ((0x3<<4) | (0x3<<8));		//choosing alternate function 3 which includes i2c0 pins
		GPIOB->DEN |= ((1<<2) | (1<<3));
		
			break;
	
		case I2C_1:
			
		SYSCTL->RCGCGPIO |= (1<<0);     //CLOCK ENABLE TO PORT A
		GPIOA->AFSEL |= (1<<6)|(1<<7);    //SETTING GPIO PINS TO ALTERNATE FUNTION
		GPIOA->ODR |= (1<<7);               // settin SDA pin to open drain select
		GPIOA->PCTL |= ((0x3<<24) | (0x3<<28));   //choosing alternate function 3 which includes i2c1 pins
		GPIOA->DEN |= ((1<<6) | (1<<7)) ;
			break;
	
		case I2C_2:
	SYSCTL->RCGCGPIO |= (1<<4);                    //CLOCK ENABLE TO PORT E
		GPIOE->AFSEL |= (1<<4)|(1<<5);                 //SETTING GPIO PINS TO ALTERNATE FUNTION
		GPIOE->ODR |= (1<<5);               // settin SDA pin to open drain select
    GPIOE->PCTL |= ((0x3<<16) | (0x3<<20));		//choosing alternate function 3 which includes i2c2 pins
		GPIOE->DEN |= ((1<<4)|(1<<5));
			break;
		
		case I2C_3:
		SYSCTL->RCGCGPIO |= (1<<3);                    //CLOCK ENABLE TO PORT D
		GPIOD->AFSEL |= (1<<0)|(1<<1);                 //SETTING GPIO PINS TO ALTERNATE FUNTION
		GPIOD->ODR |= (1<<1);               // settin SDA pin to open drain select
    GPIOD->PCTL |= ((0x3<<0) | (0x3<<4) | (0x3<<8) | (0x3<<12));		//choosing alternate function 3 which includes i2c3 pins
		GPIOD->DEN |= ((1<<0)|(1<<1));
			break;
		
		default:
			break;
	}
}

static void i2c_check_error(i2c_handler *i2cx){
	if(i2cx->I2Cx->MCS & I2C_ERROR){
		i2cx->ErrorCode = I2C_ERROR;  // Error code to be modified
		i2c_exception_handler(i2cx);
	}
}
static void i2c_exception_handler(i2c_handler *i2cx) {
	// Stop transmission, stop I2C
			i2cx->I2Cx->MCS &= ~I2C_RUN;
			i2cx->I2Cx->MCS &= ~I2C_START;
			i2cx->I2Cx->MCS |= I2C_STOP;
}

static void i2c_Master_HS_Transmit(i2c_handler *i2cx) { 
	i2cx->I2Cx->MCS |= I2C_RUN;
	i2cx->I2Cx->MCS |= I2C_START;
	i2cx->I2Cx->MCS &= ~I2C_STOP;
	i2cx->I2Cx->MCS &= ~I2C_ACK;
	i2cx->I2Cx->MCS |= I2C_HS;
} 

