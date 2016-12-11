#include <stdint.h>

#define MCP23017_DEV_ID 0x27

#define MCP23017_CTL_IODIRA   0x00
#define MCP23017_CTL_IPOLA    0x02
#define MCP23017_CTL_GPINTENA 0x04
#define MCP23017_CTL_DEFVALA  0x06
#define MCP23017_CTL_INTCONA  0x08
#define MCP23017_CTL_IOCON    0x0A
#define MCP23017_CTL_GPPUA    0x0C
#define MCP23017_CTL_INTFA    0x0E
#define MCP23017_CTL_INTCAPA  0x10
#define MCP23017_CTL_GPIOA    0x12
#define MCP23017_CTL_OLATA    0x14

#define MCP23017_CTL_IODIRB   0x01
#define MCP23017_CTL_IPOLB    0x03
#define MCP23017_CTL_GPINTENB 0x05
#define MCP23017_CTL_DEFVALB  0x07
#define MCP23017_CTL_INTCONB  0x09
#define MCP23017_CTL_GPPUB    0x0D
#define MCP23017_CTL_INTFB    0x0F
#define MCP23017_CTL_INTCAPB  0x11
#define MCP23017_CTL_GPIOB    0x13
#define MCP23017_CTL_OLATB    0x15

void init_io_expander(void);
uint8_t get_buttons(void);
void set_leds(uint8_t led_mask);
