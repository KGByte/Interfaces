
#include "I2C_Interface.h"

I2CInterface::I2CInterface(Pin *sda_pin, Pin *scl_pin) 
    :
    SDA(sda_pin), //address of platform dependent sda pin 
    SCL(scl_pin)  //address of platform dependent scl pin
{}

I2CInterface::~I2CInterface()
{
    // TODO Auto-generated destructor stub
}

I2CInterface::BusStatus I2CInterface::GetBusStatus()
{
    return bus_status;
}

void I2CInterface::Initialize()
{

}

void I2CInterface::Tick()
{
	// Perform Automatic Bus Recovery if Needed Every 500ms
	if (bus_status != kBusGood && time_since_last_recovery_attempt >= 50)
	{
		RecoverBus();

		// Perform Round-trip Bus Check
		CheckBus(true);
	}

	// Increment Counters
	safeIncrement(time_since_last_good_transaction);
	safeIncrement(time_since_last_recovery_attempt);
}

bool I2CInterface::WriteAndConfirm(uint8_t *dataOut, uint8_t numBytes, uint8_t deviceAddress, unsigned int max_attempts)
{
    bool success = false;

    // Loop for Maximum Number of Attempts
    while (max_attempts > 0)
    {
        success = Write(dataOut, numBytes, deviceAddress);

        if (success)
        {
            success = Read(dataOut, numBytes, deviceAddress, dataOut[0]);

            if (success)
            {
                return true;
            }
        }

        max_attempts--;
    }

    // Transaction Failed
    return false;
}

unsigned int I2CInterface::GetFailedTransactions()
{
    return failed_transactions;
}

unsigned int I2CInterface::GetRecoveryAttempts()
{
    return recovery_attempts;
}

unsigned int I2CInterface::GetSuccessfulTransactions()
{
    return successful_transactions;
}

bool I2CInterface::CheckAndRecoverBus()
{
	bool bus_good = false;

	// Check Bus Status
	bus_good = CheckBus();

	// Determine if Recovery Sequence is Needed
	if (!bus_good)
	{
		// Perform Recovery Sequence
		RecoverBus();

		// Check Bus after Recovery (with Roundtrip Check)
		CheckBus(true);
	}

	return bus_good;
}
