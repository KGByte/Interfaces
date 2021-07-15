
#include "SPI_Interface.h"

SPIInterface::SPIInterface(Pin *cs_pin, uint32_t clk_speed, uint8_t clk_phase, uint8_t clk_pol, uint8_t dff)
    :
    CS(cs_pin),
    clock_speed(clk_speed),
    clock_phase(clk_phase),
    clock_polarity(clk_pol),
    data_frame_format(dff)
{}

SPIInterface::~SPIInterface()
{

}