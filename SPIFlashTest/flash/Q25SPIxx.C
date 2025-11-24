
#include "main.h"
#include "Q25SPIxx.h"


/**
 * Porting SPI port config
 */
#define SPI_FLASH_PORT_CS		GPIOA
#define SPI_FLASH_PORT_CS_PIN	GPIO_PIN_2 	//Flash CS Pin
#define FLASH_SPI_PORT			hspi1 		//Link with SPI Port handler, For ex: SPI_HandleTypeDef hspi1;
#define GPIO_WRITE_PIN(Port,Pin,State)	(HAL_GPIO_WritePin(Port, Pin, State))

#define SPI_IS_BUSY 	(HAL_GPIO_ReadPin(SPI_FLASH_PORT_CS, SPI_FLASH_PORT_CS_PIN)==GPIO_PIN_RESET)
#define Flash_Receive(data,dataSize)	(HAL_SPI_Receive (&FLASH_SPI_PORT , data, dataSize, HAL_MAX_DELAY))
#define Flash_Polling_Transmit(data,dataSize) 	(HAL_SPI_Transmit(&FLASH_SPI_PORT , data, dataSize, HAL_MAX_DELAY))
//Finis Porting


extern SPI_HandleTypeDef FLASH_SPI_PORT;



#define Flash_Select() do \
{							\
	GPIO_WRITE_PIN(SPI_FLASH_PORT_CS, SPI_FLASH_PORT_CS_PIN, GPIO_PIN_RESET); 	\
}while(0);


#define Flash_UnSelect() do \
{\
	GPIO_WRITE_PIN(SPI_FLASH_PORT_CS, SPI_FLASH_PORT_CS_PIN, GPIO_PIN_SET);  \
}while(0);




void Flash_Transmit(uint8_t* data, uint16_t dataSize){

		HAL_SPI_Transmit(&FLASH_SPI_PORT , data, dataSize, HAL_MAX_DELAY);
#ifndef	EXT_FLASH_SPI_POLLING_MODE
		HAL_SPI_Transmit_DMA(&EXT_FLASH_SPI_PORT , data, dataSize);	
#endif

}


void Flash_WaitForWritingComplete(void){
uint8_t buffer[1];
	Flash_Select();
	buffer[0] = W25_R_SR1;
	Flash_Transmit(buffer, 1);
	do {
		Flash_Receive(buffer, 1);  //SR1 is repeteadly sent until Flash is selected
	} while (buffer[0] & SR1_BIT_BUSY);
	Flash_UnSelect();
}



void Flash_Read(uint32_t addr, uint8_t* data, uint32_t dataSize){
uint16_t data_to_transfer;
uint8_t buffer[5];

	buffer[0] = FLASH_READ_COMMAND;
	buffer[1] = (addr >> 16) & 0xFF;
	buffer[2] = (addr >> 8) & 0xFF;
	buffer[3] = addr & 0xFF;
	buffer[4] = W25_DUMMY;
	Flash_Select();
	Flash_Transmit(buffer, (FLASH_READ_COMMAND == W25_READ ? 4 : 5));  // "normal/slow" read command doesn't need sending dummy byte

	// dataSize is 32 bit, spi_receive handles 16bit transfers, so I have to loop...
	while (dataSize) {
		data_to_transfer = ((dataSize>0xFFFF) ? 0xFFFF : (uint16_t)dataSize);
		Flash_Receive(data, data_to_transfer);
		data+=data_to_transfer;
		dataSize-=data_to_transfer;
	}
	Flash_UnSelect();
}


void Flash_SimpleWriteAPage(uint32_t addr, uint8_t* data, uint16_t dataSize){
uint8_t buffer[4];
	buffer[0] = W25_PAGE_P;
	buffer[1] = (addr >> 16) & 0xFF;
	buffer[2] = (addr >> 8) & 0xFF;
	buffer[3] = addr & 0xFF;
	Flash_Select();
	Flash_Transmit(buffer, 4);
	Flash_Transmit(data, dataSize);
	Flash_UnSelect();
}



void Flash_Write(uint32_t addr, uint8_t* data, uint32_t dataSize){
uint8_t buffer[4];
uint16_t quota;
uint32_t inpage_addr;

	if (dataSize==0)
		return;

	// quota is the data size trasferred until now
	quota=0;

	// define the starting write position inside the first Flash page to write...
	inpage_addr=addr & (EXT_FLASH_PAGE_SIZE-1);

	// ... so I can detect if more than 1 Flash page has still to be written
	while ((dataSize-quota+inpage_addr)>EXT_FLASH_PAGE_SIZE){
	//loop here inside, until more than 1 Flash page...

		Flash_Select();
		buffer[0] = W25_W_ENABLE;
		Flash_Transmit(buffer, 1);
		Flash_UnSelect();
		Flash_SimpleWriteAPage(addr+quota,data+quota,(EXT_FLASH_PAGE_SIZE-inpage_addr));
		quota+=(EXT_FLASH_PAGE_SIZE-inpage_addr);
		// having aligned data to page border on the first writing
		// next writings start from 0 position inside a page
		inpage_addr=0;
		Flash_WaitForWritingComplete();
	}
	// now just the final Flash page...
	if (dataSize-quota) {
		Flash_Select();
		buffer[0] = W25_W_ENABLE;
		Flash_Transmit(buffer, 1);
		Flash_UnSelect();
		Flash_SimpleWriteAPage(addr+quota,data+quota,dataSize-quota);
		Flash_WaitForWritingComplete();
	}
}


void Flash_SErase4k(uint32_t addr){
uint8_t buffer[4];
	Flash_Select();
	buffer[0] = W25_W_ENABLE;
	Flash_Transmit(buffer, 1);
	Flash_UnSelect();

	buffer[0] = W25_S_ERASE4K;
	buffer[1] = (addr >> 16) & 0xFF;
	buffer[2] = (addr >> 8) & 0xFF;
	buffer[3] = addr & 0xFF;
	Flash_Select();
	Flash_Transmit(buffer, 4);
	Flash_UnSelect();
	Flash_WaitForWritingComplete();
}




void Flash_BErase32k(uint32_t addr){
uint8_t buffer[4];
	Flash_Select();
	buffer[0] = W25_W_ENABLE;
	Flash_Transmit(buffer, 1);
	Flash_UnSelect();

	buffer[0] = W25_B_ERASE32K;
	buffer[1] = (addr >> 16) & 0xFF;
	buffer[2] = (addr >> 8) & 0xFF;
	buffer[3] = addr & 0xFF;
	Flash_Select();
	Flash_Transmit(buffer, 4);
	Flash_UnSelect();
	Flash_WaitForWritingComplete();
}




void Flash_BErase64k(uint32_t addr){
uint8_t buffer[4];
	Flash_Select();
	buffer[0] = W25_W_ENABLE;
	Flash_Transmit(buffer, 1);
	Flash_UnSelect();

	buffer[0] = W25_B_ERASE64K;
	buffer[1] = (addr >> 16) & 0xFF;
	buffer[2] = (addr >> 8) & 0xFF;
	buffer[3] = addr & 0xFF;
	Flash_Select();
	Flash_Transmit(buffer, 4);
	Flash_UnSelect();
	Flash_WaitForWritingComplete();
}




void Flash_ChipErase(void){
uint8_t buffer[4];
	Flash_Select();
	buffer[0] = W25_W_ENABLE;
	Flash_Transmit(buffer, 1);
	Flash_UnSelect();

	buffer[0] = W25_CH_ERASE;
	Flash_Select();
	Flash_Transmit(buffer, 1);
	Flash_UnSelect();
	Flash_WaitForWritingComplete();
}



void Flash_PowerDown(void){
uint8_t buffer[4];

	buffer[0] = W25_POWERDOWN;
	Flash_Select();
	Flash_Transmit(buffer, 1);
	Flash_UnSelect();
}


void Flash_PowerUp(void){
uint8_t buffer[4];

	buffer[0] = W25_POWERUP_ID;
	Flash_Select();
	Flash_Transmit(buffer, 1);
	Flash_UnSelect();
	HAL_Delay(1);
}


uint8_t Flash_ReadDevID(void){
uint8_t buffer[4];
uint8_t data;

	buffer[0] = W25_POWERUP_ID;
	buffer[1] = W25_DUMMY;
	buffer[2] = W25_DUMMY;
	buffer[3] = W25_DUMMY;
	Flash_Select();
	Flash_Transmit(buffer, 4);
	Flash_Receive(&data, 1);
	Flash_UnSelect();
	return data;
}





uint16_t Flash_ReadManufactutrerAndDevID(void) {
uint8_t buffer[4];
uint16_t data;

	buffer[0] = W25_POWERUP_ID;
	buffer[1] = W25_DUMMY;
	buffer[2] = W25_DUMMY;
	buffer[3] = W25_DUMMY;
	Flash_Select();
	Flash_Transmit(buffer, 4);
	Flash_Receive((uint8_t*)&data, 2);
	Flash_UnSelect();
	return data;
}




/******************************************************************
 * @RETURN	32bit value divided in 4 bytes:
 * 			(1)MSB	dummy
 * 			(2)		Jedec Manufacturer ID
 * 			(3)		Memory Type
 * 			(4)		Capacity
 ******************************************************************
 * Memory Capacity code:
 * 			10H ->	 5Mb		11H ->  10Mb		12H ->  20Mb
 * 			13H ->  40Mb		14H ->  80Mb		15H ->  16Mb
 * 			16H ->  32Mb		17H ->  64Mb		18H -> 128Mb
 * 			19H -> 256Mb		20H -> 512Mb		21H ->   1Gb
 ******************************************************************/
uint32_t Flash_ReadJedecID(void) {
uint8_t buffer[4];
uint8_t data[3];
uint32_t result;

	buffer[0] = W25_JEDEC_ID;
	Flash_Select();
	Flash_Transmit(buffer, 1);
	Flash_Receive(data, 3);
	Flash_UnSelect();
	result=((data[0]<<16) | (data[1] <<8) | data[2]);
	return result;
}




/*********************************
 * @RETURN	256byte SFDP register content:
 *********************************/
void Flash_ReadSFDP(uint8_t* data) {
uint8_t buffer[5];
	buffer[0] = W25_R_SFPD_REG;
	for (uint8_t k=1;k<5;k++)
		buffer[k]=0;
	Flash_Select();
	Flash_Transmit(buffer, 5);
	Flash_Receive(data, 256);
	Flash_UnSelect();
}





/*********************************
 * @BRIEF	testing chip alive and kicking
 * 			reading SFDP record, it must return
 * 			a string beginning with "SFDP"
 * @RETURN	1 	test passed
 * 			0	no
 *********************************/
uint8_t Flash_TestAvailability(void) {
uint8_t data[256];
uint8_t test=1;

	for (uint8_t k=0;k!=254;k++)
		  data[k]=0xFF;
	Flash_ReadSFDP(data);
	if (data[0]!='S')
		test=0;
	if (data[1]!='F')
		test=0;
	if (data[2]!='D')
		test=0;
	if (data[3]!='P')
		test=0;
	return test;
}




/******************************************************************
 * @BRIEF	reading manufacutrer and device ID
 * 			checking if connected device is a Winbond Flash
 ******************************************************************/
uint8_t Flash_Init(void){
uint32_t JedecID;
	HAL_Delay(6);	// supposing init is called on system startup: 5 ms (tPUW) required after power-up to be fully available
	Flash_Reset();
	if (!Flash_TestAvailability())
		return 0;
	JedecID=Flash_ReadJedecID() ;	//select the memSize byte
	if (((JedecID >> 16) & 0XFF) != 0xEF)  // if ManufacturerID is not Winbond (0xEF)
		return 0;
	return 1;  //return memSize as per table in Flash_ReadJedecID() definition
}





void Flash_Reset(void)
{
	uint8_t command;
	command = W25_RESET_EN;
	Flash_Select();
	Flash_Transmit(&command, 1);
	Flash_UnSelect();
	command = W25_RESET;
	Flash_Select();
	Flash_Transmit(&command, 1);
	Flash_UnSelect();
	HAL_Delay(1);	// 30us needed by resetting
}


void DataReader_ReadData(uint32_t address24, uint8_t* buffer, uint32_t length)
{
	Flash_Read(address24, buffer, length);
}


void DataReader_StartDMAReadData(uint32_t address24, uint8_t* buffer, uint32_t length)
{
	Flash_Read(address24, buffer, length);
}

