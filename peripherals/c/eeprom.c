#include "eeprom.h"

//*****************************************************************************
// Used to determine if the EEPROM is busy writing the last transaction to 
// non-volatile storage
//
// Paramters
//    i2c_base:   a valid base address of an I2C peripheral
//
// Returns
// I2C_OK is returned one the EEPROM is ready to write the next byte
//*****************************************************************************
static 
i2c_status_t eeprom_wait_for_write( int32_t  i2c_base)
{
  
  i2c_status_t status;
  
  if( !i2cVerifyBaseAddr(i2c_base) )
  {
    return  I2C_INVALID_BASE;
  }

  // Set the I2C address to be the EEPROM and in Write Mode
  status = i2cSetSlaveAddr(i2c_base, MCP24LC32AT_DEV_ID, I2C_WRITE);

  // Poll while the device is busy.  The  MCP24LC32AT will not ACK
  // writing an address while the write has not finished.
  do 
  {
    // The data we send does not matter.  This has been set to 0x00, but could
    // be set to anything
    status = i2cSendByte( i2c_base, 0x00, I2C_MCS_START | I2C_MCS_RUN | I2C_MCS_STOP);
    
    // Wait for the address to finish transmitting
    while ( I2CMasterBusy(i2c_base)) {};
    
    // If the address was not ACKed, try again.
  } while (I2CMasterAdrAck(i2c_base) == false);

  return  status;
}
  
  
//*****************************************************************************
// Writes a single byte of data out to the  MCP24LC32AT EEPROM.  
//
// Paramters
//    i2c_base:   a valid base address of an I2C peripheral
//
//    address:    16-bit address of the byte being written.  Only the lower
//                12 bits is used by the EEPROM
//
//    data:       Data written to the EEPROM.
//
// Returns
// I2C_OK if the byte was written to the EEPROM.
//*****************************************************************************
i2c_status_t eeprom_byte_write
( 
  uint32_t  i2c_base,
  uint16_t  address,
  uint8_t   data
)
{
	i2c_status_t status;
  
	// Before doing anything, make sure the I2C device is idle
	while ( I2CMasterBusy(i2c_base)) {};

	//==============================================================
	// Set the I2C address to be the EEPROM
	// ADD CODE
	//==============================================================
	i2cSetSlaveAddr(i2c_base, MCP24LC32AT_DEV_ID, I2C_WRITE);

	// If the EEPROM is still writing the last byte written, wait
	eeprom_wait_for_write(i2c_base);

	//==============================================================
	// Send the Upper byte of the address
	// ADD CODE	
	//==============================================================
	status = i2cSendByte(
		i2c_base,
		(uint8_t)(address >> 8),
		I2C_MCS_START | I2C_MCS_RUN
	);
	//==============================================================
	// Send the Lower byte of the address
	// ADD CODE
	//==============================================================
	status = i2cSendByte(i2c_base, (uint8_t)address, I2C_MCS_RUN);
	//==============================================================
	// Send the Byte of data to write
	// ADD CODE
	//==============================================================
	status = i2cSendByte(i2c_base, data, I2C_MCS_RUN | I2C_MCS_STOP);
	
	return status;
}

i2c_status_t eeprom_seq_write( 
	uint32_t     i2c_base,
	uint16_t     address,
	uint8_t      *data,
	unsigned int bytes
)
{
	short i;
	i2c_status_t status;
	
	// make sure the write address is on a page boundary
	if ((address % EEPROM_PAGE_SIZE) != 0) return I2C_INVALID_PARAM;
	
	// wait for control of the I2C hardware
	while(I2CMasterBusy(i2c_base));
	
	// write to the EEPROM, waiting for each page to be written
	while (bytes > EEPROM_PAGE_SIZE) {
		eeprom_wait_for_write(i2c_base);
		
		// set EEPROM I2C address
		i2cSetSlaveAddr(i2c_base, MCP24LC32AT_DEV_ID, I2C_WRITE);
		eeprom_wait_for_write(i2c_base);
		
		// send base address to EEPROM
		status = i2cSendByte(i2c_base, (uint8_t)(address >> 8), I2C_MCS_START | I2C_MCS_RUN);
		if (status != I2C_OK) goto STOP;
		status = i2cSendByte(i2c_base, (uint8_t)address, I2C_MCS_RUN);
		if (status != I2C_OK) goto STOP;
		
		// write all but the last bytes without generating a stop condition
		for (i = 0; i < EEPROM_PAGE_SIZE-1; i++) {
			status = i2cSendByte(i2c_base, *data, I2C_MCS_RUN);
			data++;
			if (status != I2C_OK) goto STOP;
		}
		// generate a stop condition on the last byte write
		status = i2cSendByte(i2c_base, *data, I2C_MCS_RUN | I2C_MCS_STOP);
		if (status != I2C_OK) return status;
		data++;
		
		// keep track of pages written
		bytes -= EEPROM_PAGE_SIZE;
		address += EEPROM_PAGE_SIZE;
	}
	// write all remaining bytes after writing full pages
	if (bytes > 0) {
		// set EEPROM I2C address
		i2cSetSlaveAddr(i2c_base, MCP24LC32AT_DEV_ID, I2C_WRITE);
		eeprom_wait_for_write(i2c_base);
		
		// send base address to EEPROM
		status = i2cSendByte(i2c_base, (uint8_t)(address >> 8), I2C_MCS_START | I2C_MCS_RUN);
		if (status != I2C_OK) goto STOP;
		status = i2cSendByte(i2c_base, (uint8_t)address, I2C_MCS_RUN);
		if (status != I2C_OK) goto STOP;
		
		// write all but the last byte without sending a stop condition
		while (bytes > 1) {
			status = i2cSendByte(i2c_base, *data, I2C_MCS_RUN);
			data++;
			bytes--;
			if (status != I2C_OK) goto STOP;
		}
		// write the last byte with a stop condition
		status = i2cSendByte(i2c_base, *data, I2C_MCS_RUN | I2C_MCS_STOP);
		if (status != I2C_OK) return status;
	}
	
	return I2C_OK;
	
	STOP:
	i2cSendByte(i2c_base, 0, I2C_MCS_STOP);
	return status;
}

//*****************************************************************************
// Reads a single byte of data from the  MCP24LC32AT EEPROM.  
//
// Paramters
//    i2c_base:   a valid base address of an I2C peripheral
//
//    address:    16-bit address of the byte being read.  Only the lower
//                12 bits is used by the EEPROM
//
//    data:       data read from the EEPROM is returned to a uint8_t pointer.
//
// Returns
// I2C_OK if the byte was read from the EEPROM.
//*****************************************************************************
i2c_status_t eeprom_byte_read
( 
  uint32_t  i2c_base,
  uint16_t  address,
  uint8_t   *data
)
{
	// Before doing anything, make sure the I2C device is idle
	while ( I2CMasterBusy(i2c_base)) {};

	// If the EEPROM is still writing the last byte written, wait
	eeprom_wait_for_write(i2c_base);

	//==============================================================
	// Set the I2C slave address to be the EEPROM and in Write Mode
	// ADD CODE
	//==============================================================
	i2cSetSlaveAddr(i2c_base, MCP24LC32AT_DEV_ID, I2C_WRITE);
	//==============================================================
	// Send the Upper byte of the address
	// ADD CODE
	//==============================================================
	i2cSendByte(
		i2c_base,
		(uint8_t)(address >> 8),
		I2C_MCS_START | I2C_MCS_RUN
	);
	//==============================================================
	// Send the Lower byte of the address
	// ADD CODE
	//==============================================================
	i2cSendByte(i2c_base, (uint8_t)(address), I2C_MCS_RUN);
	//==============================================================
	// Set the I2C slave address to be the EEPROM and in Read Mode
	// ADD CODE
	//==============================================================
	i2cSetSlaveAddr(i2c_base, MCP24LC32AT_DEV_ID, I2C_READ);
	//==============================================================
	// Read the data returned by the EEPROM
	// ADD CODE
	//==============================================================
	i2cGetByte(i2c_base, data, I2C_MCS_START | I2C_MCS_RUN | I2C_MCS_STOP);

	return I2C_OK;
}

i2c_status_t eeprom_seq_read( 
	uint32_t     i2c_base,
	uint16_t     address,
	uint8_t      *data,
	unsigned int bytes)
{
	i2c_status_t status;
	
	// wait for hardware to become available
	while (I2CMasterBusy(i2c_base));
	eeprom_wait_for_write(i2c_base);
	
	// address the EEPROM
	i2cSetSlaveAddr(i2c_base, MCP24LC32AT_DEV_ID, I2C_WRITE);
	status = i2cSendByte(i2c_base, (uint8_t)(address >> 8), I2C_MCS_START | I2C_MCS_RUN);
	if (status != I2C_OK) goto STOP;
	status = i2cSendByte(i2c_base, (uint8_t)address, I2C_MCS_RUN);
	if (status != I2C_OK) goto STOP;
	
	// initiate read with I2C restart
	i2cSetSlaveAddr(i2c_base, MCP24LC32AT_DEV_ID, I2C_READ);
	status = i2cGetByte(i2c_base, data, I2C_MCS_START | I2C_MCS_RUN | I2C_MCS_DATACK);
	if (status != I2C_OK) goto STOP;
	data++;
	bytes--;
	
	// get and ack all but the last byte without generating a stop condition
	while (bytes > 1) {
		status = i2cGetByte(i2c_base, data, I2C_MCS_RUN | I2C_MCS_DATACK);
		if (status != I2C_OK) goto STOP;
		data++;
		bytes--;
	}
	// generate a stop condition after getting the last byte and do not ack
	status = i2cGetByte(i2c_base, data, I2C_MCS_RUN | I2C_MCS_STOP);
	return status;
	
	STOP:
	i2cGetByte(i2c_base, 0, I2C_MCS_STOP);
	return status;
}

//*****************************************************************************
// Test the EEPROM
//*****************************************************************************
void test_eeprom(void)
{
  
  uint8_t write_data[EEPROM_TEST_NUM_BYTES];
  uint8_t read_data[EEPROM_TEST_NUM_BYTES];
  bool passed = true;
  int i;
  
		printf("==== Starting EEPROM Test ====\n\r");
	
  // Write data to the EEPROM
  for(i = 0; i < EEPROM_TEST_NUM_BYTES; i++)
  {
    write_data[i] = rand();
    eeprom_byte_write(EEPROM_I2C_BASE,i,write_data[i]);
  }

  // Read data back from the EEPROM
  for(i = 0; i < EEPROM_TEST_NUM_BYTES; i++)
  {
    eeprom_byte_read(EEPROM_I2C_BASE,i,&(read_data[i]));
  }
  
  // Verify that the bytes written match the bytes read
  for(i = 0; i < EEPROM_TEST_NUM_BYTES; i++)
  {
		printf("\tAddr 0x%04x.  write: 0x%02x read: 0x%02x\n\r", i, write_data[i], read_data[i]);
    if( write_data[i] != read_data[i])
    {
      passed = false;
    }
  }
  
  // Print if the test passed or failed.
  if ( passed == true)
  {
      printf("\tEEPROM Test Passed\n\r");
  }
  else
  {
      printf("\tEEPROM Test Failed\n\r");
  }
  
		printf("==== Stopping EEPROM Test ====\n\n\r");
}

