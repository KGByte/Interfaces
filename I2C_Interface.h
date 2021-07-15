
#ifndef I2C_INTERFACE_
#define I2C_INTERFACE_

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#include "Pin.h"

class I2CInterface
{
public:
    I2CInterface(Pin *sda_pin, Pin *scl_pin);
    ~I2CInterface();

    // Bus Status
    enum BusStatus
    {
        kBusGood = 0,
        kBusNotGoodSDALow,
        kBusNotGoodPeripheralLockup,
        kBusNotGoodTransactionTimeout,
        kBusNotGoodBadReadData
    };

    // Get Current Status of the I2C Bus
    BusStatus GetBusStatus();

    // Get Number of Failed Transactions
    unsigned int GetFailedTransactions();

    // Get Number of Recovery Attempts
    unsigned int GetRecoveryAttempts();

    // Get Number of Successful I2C Transactions
    unsigned int GetSuccessfulTransactions();

    // Perform Regular Bus Checks
    void Tick();

    void Initialize();

    // Perform Write and Confirm Transaction
    bool WriteAndConfirm(uint8_t *dataOut, uint8_t numBytes, uint8_t deviceAddress, unsigned int max_attempts = 3);

    //accessed by I2C devices, implemented by specific MCU
    virtual bool Write(uint8_t *dataIn, uint8_t numBytes, uint8_t deviceAddress) = 0;
    virtual bool Read(uint8_t *dataOut, uint8_t numBytes, uint8_t deviceAddress, uint8_t registerAddress) = 0;
    virtual bool ReadOnly(uint8_t *dataOut, uint8_t numBytes, uint8_t deviceAddress) = 0;

//accessed by inherited classes
protected:
    // Checks Bus Integrity. Set Round Trip Check to true to Request an I2C transaction to verify
    virtual bool CheckBus(bool round_trip_check = false) = 0;

    // Perform Bus Check and then Recovery if Needed
    bool CheckAndRecoverBus();

    //bool I2CMasterTimeout(uint32_t ui32Base);
    virtual bool CheckKnownValue() = 0;

    //used in the read function. Writes to the register you want to read from
    virtual void I2CSingleWrite(uint8_t registerAddress, uint8_t deviceAddress) = 0;

    //Pulses clock to recover the I2C bus from faulted state
    virtual void RecoverBus() = 0;

    // Base Address of I2C Peripheral
    // uint32_t base_address;

    // The Current Health of the I2C Bus
    BusStatus bus_status;

    // Number of Successful Transactions
    unsigned int successful_transactions;

    // Number of Erroneous Transactions
    unsigned int failed_transactions;

    // Number of Bus Recovery Attempts
    unsigned int recovery_attempts;

    // Ticks Since Last Good I2C Transaction
    unsigned int time_since_last_good_transaction;

    // Ticks Since Last Recovery Attempt
    unsigned int time_since_last_recovery_attempt;
    
    Pin *SDA;
    Pin *SCL;
};

#endif
