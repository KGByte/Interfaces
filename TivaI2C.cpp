
#include "TivaI2C.h"

TivaI2C::TivaI2C(uint32_t base_address, Pin *sda_pin, Pin *scl_pin) : I2CInterface(sda_pin, scl_pin), 
    i2c_base_address(base_address)
{}

bool TivaI2C::Write(uint8_t *data_in, uint8_t num_bytes, uint8_t device_address)
{
	bool timeout_occurred = false;

	// Check Bus Integrity
    if (!CheckBus())
    {
    	return false;
    }

    //Set device address, false for writing from master to slave
    I2CMasterSlaveAddrSet(i2c_base_address, device_address, false);

    //put data into the fifo
    I2CMasterDataPut(i2c_base_address, data_in[0]);

    // Send only one byte
    if (num_bytes == 1)
    {
        //initiate single byte transfer
        I2CMasterControl(i2c_base_address, I2C_MASTER_CMD_SINGLE_SEND);

        // Loops until I2C Transaction Completed or Timeout Occurred
        timeout_occurred = !timeout_loop_function(I2CMasterBusy, i2c_base_address, false);
    }
    // Send multiple bytes
    else
    {
        // Initiate multi-byte transfer
        I2CMasterControl(i2c_base_address, I2C_MASTER_CMD_BURST_SEND_START);

        // Loops until I2C Transaction Completed or Timeout Occurred
        timeout_occurred = !timeout_loop_function(I2CMasterBusy, i2c_base_address, false);

        // If num_bytes is 2, we immediately break and send a finish command with the last byte
        for (uint8_t i = 1; i < num_bytes - 1; i++)
        {
            // Put data into the fifo
            I2CMasterDataPut(i2c_base_address, data_in[i]);

            // Initiate multi-byte transfer
            I2CMasterControl(i2c_base_address, I2C_MASTER_CMD_BURST_SEND_CONT);

            // Loops until I2C Transaction Completed or Timeout Occurred
            timeout_occurred = !timeout_loop_function(I2CMasterBusy, i2c_base_address, false);
        }

        //put last byte into the fifo
        I2CMasterDataPut(i2c_base_address, data_in[num_bytes - 1]);

        //send last byte and then stop condition
        I2CMasterControl(i2c_base_address, I2C_MASTER_CMD_BURST_SEND_FINISH);

        // Loops until I2C Transaction Completed or Timeout Occurred
        timeout_occurred = !timeout_loop_function(I2CMasterBusy, i2c_base_address, false);
    }

    //set bus status to timeout
    if (timeout_occurred)
    {
    	bus_status = kBusNotGoodTransactionTimeout;
    }

    //if no timeout or error, transaction is successful 
    if (bus_status == kBusGood)
    {
    	safeIncrement(successful_transactions);
    	time_since_last_good_transaction = 0;
    	return true;
    }
    else
    {
    	safeIncrement(failed_transactions);
    	return false;
    }
}

bool TivaI2C::Read(uint8_t *data_out, uint8_t num_bytes, uint8_t device_address, uint8_t register_address)
{
	bool timeout_occurred = false;

	// Check Bus Integrity
    if (!CheckBus())
    {
    	return false;
    }

    // Send write to register you want to read from first
    I2CSingleWrite(register_address, device_address);

    //set mode for reading
    I2CMasterSlaveAddrSet(i2c_base_address, device_address, true);

    if (num_bytes == 1)
    {
        //single receive start
        I2CMasterControl(i2c_base_address, I2C_MASTER_CMD_SINGLE_RECEIVE);

        // Loops until I2C Transaction Completed or Timeout Occurred
        timeout_occurred = !timeout_loop_function(I2CMasterBusy, i2c_base_address, false);

        //store first byte into data array
        data_out[0] = I2CMasterDataGet(i2c_base_address);
    }
    else
    {
        // Burst receive start
        I2CMasterControl(i2c_base_address, I2C_MASTER_CMD_BURST_RECEIVE_START);

        // Loops until I2C Transaction Completed or Timeout Occurred
        timeout_occurred = !timeout_loop_function(I2CMasterBusy, i2c_base_address, false);

        // Load First Byte of Data
        data_out[0] = I2CMasterDataGet(i2c_base_address);

        //store first byte into data array (high)
        for(uint8_t i = 1; i < num_bytes - 1; i++)
        {
            I2CMasterControl(i2c_base_address, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
            timeout_occurred = !timeout_loop_function(I2CMasterBusy, i2c_base_address, false);
            data_out[i] = I2CMasterDataGet(i2c_base_address);
        }

        I2CMasterControl(i2c_base_address, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

        // Loops until I2C Transaction Completed or Timeout Occurred
        timeout_occurred = !timeout_loop_function(I2CMasterBusy, i2c_base_address, false);

        // Store third byte into data array (lsb)
        data_out[num_bytes - 1] = I2CMasterDataGet(i2c_base_address);
    }

    if (timeout_occurred)
    {
    	bus_status = kBusNotGoodTransactionTimeout;
    }

    if (bus_status == kBusGood)
    {
    	time_since_last_good_transaction = 0;
    	return true;
    }
    else
    {
    	return false;
    }
}

bool TivaI2C::ReadOnly(uint8_t *data_out, uint8_t num_bytes, uint8_t device_address)
{
	bool timeout_occurred = false;

	// Check Bus Integrity
    if (!CheckBus())
    {
    	return false;
    }

    // Set mode for reading
    I2CMasterSlaveAddrSet(i2c_base_address, device_address, true);

    if (num_bytes == 1)
    {
        // Single receive start
        I2CMasterControl(i2c_base_address, I2C_MASTER_CMD_SINGLE_RECEIVE);

        // Wait for mcu to finish or timeout
        timeout_occurred = !timeout_loop_function(I2CMasterBusy, i2c_base_address, false);

        // Store first byte into data array (high)
        data_out[0] = I2CMasterDataGet(i2c_base_address);
    }
    else
    {
        //burst receive start
        I2CMasterControl(i2c_base_address, I2C_MASTER_CMD_BURST_RECEIVE_START);
        //wait for mcu to finish or timeout
        timeout_occurred = !timeout_loop_function(I2CMasterBusy, i2c_base_address, false);
        //store first byte into data array (high)
        data_out[0] = I2CMasterDataGet(i2c_base_address);

        for(uint8_t i = 1; i < num_bytes - 1; i++)
        {
            I2CMasterControl(i2c_base_address, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
            timeout_occurred = !timeout_loop_function(I2CMasterBusy, i2c_base_address, false);
            data_out[i] = I2CMasterDataGet(i2c_base_address);
        }

        I2CMasterControl(i2c_base_address, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

        timeout_occurred = !timeout_loop_function(I2CMasterBusy, i2c_base_address, false);
        //store third byte into data array (lsb)
        data_out[num_bytes - 1] = I2CMasterDataGet(i2c_base_address);
    }

    if (timeout_occurred)
    {
    	bus_status = kBusNotGoodTransactionTimeout;
    }

    if (bus_status == kBusGood)
    {
    	time_since_last_good_transaction = 0;
    	return true;
    }
    else
    {
    	return false;
    }
}

void TivaI2C::I2CSingleWrite(uint8_t register_address, uint8_t device_address)
{
	bool timeout_occurred = false;

    // Set slave address, false for writing from master to slave
    I2CMasterSlaveAddrSet(i2c_base_address, device_address, false);

    // Put data into the FIFO
    I2CMasterDataPut(i2c_base_address, register_address);

    // Initiate single byte transfer
    I2CMasterControl(i2c_base_address, I2C_MASTER_CMD_SINGLE_SEND);

    // Loops until I2C Transaction Completed or Timeout Occurred
    timeout_occurred = !timeout_loop_function(I2CMasterBusy, i2c_base_address, false);

    if (timeout_occurred)
    {
    	bus_status = kBusNotGoodTransactionTimeout;
    }
}

void TivaI2C::RecoverBus()
{
    //set clock as output to pulse manually and as open drain output
    GPIOPinTypeGPIOOutputOD(GPIO_PORTA_BASE, GPIO_PIN_6);

    for (uint8_t i = 0; i < 10; i++)
    {
        //SCLK clear
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);
        delayUs(10);

        //SCLK set
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);
        delayUs(10);

        if (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7))
        {
            break;
        }
    }

    // Perform a Hardware Reset of I2C Peripheral
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);
    delayUs(100);

    // Initialize Master
    I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), false);

    //set timeout for I2C bus
    I2CMasterTimeoutSet(I2C1_BASE, 0xFF);

    //return SCLK to I2C functionality
    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPinConfigure(GPIO_PA6_I2C1SCL);

    safeIncrement(recovery_attempts);
    time_since_last_recovery_attempt = 0;
}

bool TivaI2C::CheckBus(bool round_trip_check)
{
	bool bus_good = true;

	// Check Status of SDA Pin (should be high)
    if (SDA->GetPinStatus(3) == Pin::kLow)
    {
    	bus_status = kBusNotGoodSDALow;
    	bus_good = false;
    }
    // Check to See if I2C Peripheral is Busy
    else if (I2CMasterBusBusy(i2c_base_address))
	{
    	bus_status = kBusNotGoodPeripheralLockup;
    	bus_good = false;
	}
    // Perform Round-trip I2C Transaction Test if Requested
    else if (round_trip_check)
    {
    	bus_good = CheckKnownValue();

    	if (bus_good)
    	{
    		bus_status = kBusGood;
    	}
    	else
    	{
    		bus_status = kBusNotGoodBadReadData;
    	}
    }

    return bus_good;
}

bool TivaI2C::CheckKnownValue()
{
    uint8_t RxDataArray[2];
    uint8_t deviceId;

    // Get device id from pressure sensor, returns on correct read
    Read(RxDataArray, 1, 0x5D, 0x0F);
    deviceId = RxDataArray[0];

    // Correct Value is 0xB1
    if (deviceId == 0xB1)
    {
        safeIncrement(successful_transactions);
        return true;
    }
    else
    {
		safeIncrement(failed_transactions);
		return false;
    }
}