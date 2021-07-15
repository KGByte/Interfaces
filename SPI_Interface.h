

#ifndef SPI_INTERFACE_H_
#define SPI_INTERFACE_H_

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#include "Pin.h"

class SPIInterface
{
public:
    SPIInterface(Pin *cs_pin, uint32_t clk_speed, uint8_t clk_phase, uint8_t clk_pol, uint8_t dff);
    ~SPIInterface();
    //virtual methods
    virtual void Initialize()=0;
    virtual bool Write(uint32_t *data_out, uint8_t num_bytes)=0;
    virtual bool Transfer(uint32_t *data_out, uint32_t *data_in, uint8_t num_bytes)=0;
    virtual void SetCSLow()=0;
    virtual void SetCSHigh()=0;
protected:
    virtual void ClearReceiveBuffer(uint32_t *buffer)=0;
    //virtual void SetClockSpeed(uint32_t clk_speed);
    uint32_t clock_speed; //clock speed for this bus
    uint8_t clock_phase;
    uint8_t clock_polarity;
    uint8_t data_frame_format; 
    //bool num_pins;
    Pin *CS;
private:
//check bus?
//recover bus?
//timemout fun
};

#endif
