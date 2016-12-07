#include "io_expander.h"
#include "i2c.h"

void init_io_expander(void) {
	// wait for the I2C master to be idle
	while(I2CMasterBusy(I2C1_BASE));
	
	// LED configuration
	// configure bank A as output
	i2cSetSlaveAddr(I2C1_BASE, MCP23017_DEV_ID, I2C_WRITE);
	i2cSendByte(I2C1_BASE, MCP23017_CTL_IODIRA, I2C_MCS_START | I2C_MCS_RUN);
	i2cSendByte(I2C1_BASE, 0x00, I2C_MCS_RUN | I2C_MCS_STOP);
	
	// push button configuration
	// configure bank B pins 0-3 as input with pull-up
	// input is defualt, pull-up must be set
	i2cSetSlaveAddr(I2C1_BASE, MCP23017_DEV_ID, I2C_WRITE);
	i2cSendByte(I2C1_BASE, MCP23017_CTL_GPPUB, I2C_MCS_START | I2C_MCS_RUN);
	i2cSendByte(I2C1_BASE, 0x0F, I2C_MCS_RUN | I2C_MCS_STOP);
}

uint8_t get_buttons(void) {
	uint8_t data;
	
	// wait for the I2C master to be idle
	while(I2CMasterBusy(I2C1_BASE));
	
	// read the GPIOB register
	i2cSetSlaveAddr(I2C1_BASE, MCP23017_DEV_ID, I2C_WRITE);
	i2cSendByte(I2C1_BASE, MCP23017_CTL_GPIOB, I2C_MCS_START | I2C_MCS_RUN);
	i2cSetSlaveAddr(I2C1_BASE, MCP23017_DEV_ID, I2C_READ);
	i2cGetByte(I2C1_BASE, &data, I2C_MCS_START | I2C_MCS_RUN | I2C_MCS_ACK | I2C_MCS_STOP);
	
	return data;
}

void set_leds(uint8_t led_mask) {
	// wait for the I2C master to be idle
	while(I2CMasterBusy(I2C1_BASE));
	
	// write to the GPIOA register
	i2cSetSlaveAddr(I2C1_BASE, MCP23017_DEV_ID, I2C_WRITE);
	i2cSendByte(I2C1_BASE, MCP23017_CTL_GPIOA, I2C_MCS_START | I2C_MCS_RUN);
	i2cSendByte(I2C1_BASE, led_mask, I2C_MCS_RUN | I2C_MCS_STOP);
}