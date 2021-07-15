
#ifndef TIVA_I2C_H_
#define TIVA_I2C_H  
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"

#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

#include "Utilities/utility.h"

#include "Library/critical.h"
#include "Library/safe_counter.h"

#include "MCU/I2C_Interface.h"
#include "MCU/Pin.h"

class TivaI2C : public I2CInterface
{
public:
    TivaI2C(uint32_t i2c_base_address, Pin *sda_pin, Pin *scl_pin);
    virtual bool Write(uint8_t *data_in, uint8_t num_bytes, uint8_t device_address);
    virtual bool Read(uint8_t *data_out, uint8_t num_bytes, uint8_t device_address, uint8_t register_address);
    virtual bool ReadOnly(uint8_t *data_out, uint8_t num_bytes, uint8_t device_address);
protected:
    virtual bool CheckBus(bool round_trip_check = false);
    virtual bool CheckKnownValue();
    virtual void I2CSingleWrite(uint8_t register_address, uint8_t device_address);
    virtual void RecoverBus();

private:
    uint32_t i2c_base_address;
};

#endif
