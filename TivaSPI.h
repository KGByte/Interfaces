

#ifndef TIVA_SPI_H_
#define TIVA_SPI_H_

#include "SPI_Interface.h"

#include "inc/hw_ssi.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"

#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

class TivaSPI : public SPIInterface
{
public:
    TivaSPI(uint32_t base_address, Pin *cs_pin, uint32_t clk_speed, uint8_t clk_phase, uint8_t clk_pol, uint8_t dff);
    virtual void Initialize();
    virtual bool Write(uint32_t *data_out, uint8_t num_bytes);
    virtual bool Transfer(uint32_t *data_out, uint32_t *data_in, uint8_t num_bytes);
    virtual void ClearReceiveBuffer(uint32_t *buffer);
    virtual void SetCSLow();
    virtual void SetCSHigh();

private:
    uint8_t GetClkPolarity();
    uint8_t GetClkPhase();
    uint32_t spi_base_address; //base address of SPI bus
};

#endif
