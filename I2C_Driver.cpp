#include <I2C_Driver.h>

#include <cstring>

#include "../driverlib/gpio.h"
#include "../driverlib/i2c.h"
#include "../driverlib/interrupt.h"
#include "../driverlib/pin_map.h"
#include "../driverlib/sysctl.h"
#include "../inc/hw_i2c.h"
#include "../inc/hw_ints.h"
#include "../inc/hw_memmap.h"

TivaI2C::TivaI2C() {}
TivaI2C::~TivaI2C() {}

void TivaI2C::Initialize() {
  kBaseAddress = I2C0_BASE;
  IntEnable(INT_I2C0);

  SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0)) {
  }
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)) {
  }
  GPIOPinConfigure(GPIO_PB2_I2C0SCL);
  GPIOPinConfigure(GPIO_PB3_I2C0SDA);

  I2CMasterIntEnable(I2C0_BASE);
  I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);

  GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
  GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
}

bool TivaI2C::BlockingWrite(uint8_t *data_in, uint8_t num_bytes,
                            uint8_t device_address) {
  // Set device address, false for writing from master to slave
  I2CMasterSlaveAddrSet(kBaseAddress, device_address, false);

  // put data into the fifo
  I2CMasterDataPut(kBaseAddress, data_in[0]);

  // Send only one byte
  if (num_bytes == 1) {
    // initiate single byte transfer
    I2CMasterControl(kBaseAddress, I2C_MASTER_CMD_SINGLE_SEND);

    while (I2CMasterBusy(I2C0_BASE)) {
    }
  }
  // Send multiple bytes
  else {
    // Initiate multi-byte transfer
    I2CMasterControl(kBaseAddress, I2C_MASTER_CMD_BURST_SEND_START);

    while (I2CMasterBusy(I2C0_BASE)) {
    }

    // If num_bytes is 2, we immediately break and send a finish command with
    // the last byte
    for (uint8_t i = 1; i < num_bytes - 1; i++) {
      // Put data into the fifo
      I2CMasterDataPut(kBaseAddress, data_in[i]);

      // Initiate multi-byte transfer
      I2CMasterControl(kBaseAddress, I2C_MASTER_CMD_BURST_SEND_CONT);

      while (I2CMasterBusy(I2C0_BASE)) {
      }
    }

    // put last byte into the fifo
    I2CMasterDataPut(kBaseAddress, data_in[num_bytes - 1]);

    // send last byte and then stop condition
    I2CMasterControl(kBaseAddress, I2C_MASTER_CMD_BURST_SEND_FINISH);

    while (I2CMasterBusy(I2C0_BASE)) {
    }
  }
  return true;
}
bool TivaI2C::BlockingRead(uint8_t *data_out, uint8_t num_bytes,
                           uint8_t device_address, uint8_t register_address) {
  // Send write to register you want to read from first
  BlockingSingleWrite(register_address, device_address);

  // set mode for reading
  I2CMasterSlaveAddrSet(kBaseAddress, device_address, true);

  if (num_bytes == 1) {
    // single receive start
    I2CMasterControl(kBaseAddress, I2C_MASTER_CMD_SINGLE_RECEIVE);

    while (I2CMasterBusy(I2C0_BASE)) {
    }

    // store first byte into data array
    data_out[0] = I2CMasterDataGet(kBaseAddress);
  } else {
    // Burst receive start
    I2CMasterControl(kBaseAddress, I2C_MASTER_CMD_BURST_RECEIVE_START);

    while (I2CMasterBusy(I2C0_BASE)) {
    }

    // Load First Byte of Data
    data_out[0] = I2CMasterDataGet(kBaseAddress);

    // store first byte into data array (high)
    for (uint8_t i = 1; i < num_bytes - 1; i++) {
      I2CMasterControl(kBaseAddress, I2C_MASTER_CMD_BURST_RECEIVE_CONT);

      while (I2CMasterBusy(I2C0_BASE)) {
      }

      data_out[i] = I2CMasterDataGet(kBaseAddress);
    }

    I2CMasterControl(kBaseAddress, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

    while (I2CMasterBusy(I2C0_BASE)) {
    }

    // Store third byte into data array (lsb)
    data_out[num_bytes - 1] = I2CMasterDataGet(kBaseAddress);
  }
  return true;
}

void TivaI2C::BlockingSingleWrite(uint8_t register_address,
                                  uint8_t device_address) {
  // Set slave address, false for writing from master to slave
  I2CMasterSlaveAddrSet(kBaseAddress, device_address, false);

  // Put data into the FIFO
  I2CMasterDataPut(kBaseAddress, register_address);

  // Initiate single byte transfer
  I2CMasterControl(kBaseAddress, I2C_MASTER_CMD_SINGLE_SEND);

  while (I2CMasterBusy(I2C0_BASE)) {
  }
}

/* Start transmission and let callback to everything */
void TivaI2C::StartNonBlockingTransmission(uint8_t *data_in, uint8_t num_bytes,
                                           uint8_t register_address,
                                           uint8_t device_address) {
  /* Only start transmission if bus isn't busy */
  if (state == I2CDriverState::kIdle) {
    i2c_control.num_write_bytes = num_bytes;
    i2c_control.write_index = 0;
    i2c_control.register_address = register_address;
    state = I2CDriverState::kTransmitting;
    std::memcpy(tx_data_buffer, data_in, num_bytes);
    // Set device address, false for writing from master to slave
    I2CMasterSlaveAddrSet(kBaseAddress, device_address, false);

    // Start with register address we write to
    I2CMasterDataPut(kBaseAddress, register_address);
    // Send only one byte
    if (num_bytes == 1) {
      // initiate single byte transfer
      I2CMasterControl(kBaseAddress, I2C_MASTER_CMD_SINGLE_SEND);
    } else {
      I2CMasterControl(kBaseAddress, I2C_MASTER_CMD_BURST_SEND_START);
    }
  } else {
    return;
  }
}

/* Start transmission and let callback to everything */
void TivaI2C::StartNonBlockingRead(uint8_t num_bytes, uint8_t register_address,
                                   uint8_t device_address) {
  /* Only start transmission if bus isn't busy */
  if (state == I2CDriverState::kIdle) {
    i2c_control.num_read_bytes = num_bytes;
    i2c_control.read_index = 0;
    i2c_control.register_address = register_address;
    i2c_control.device_address = device_address;
    state = I2CDriverState::kReceiving;
    stage = I2CReceiveStage::kSendingAddress;
    // Set device address, false for writing from master to slave
    I2CMasterSlaveAddrSet(kBaseAddress, device_address, false);
    // Start with register address we write to
    I2CMasterDataPut(kBaseAddress, register_address);
    // initiate single byte transfer
    I2CMasterControl(kBaseAddress, I2C_MASTER_CMD_SINGLE_SEND);
  } else {
    return;
  }
}

void TivaI2C::I2CCallback() {
  switch (state) {
    case I2CDriverState::kIdle:

      break;
    case I2CDriverState::kTransmitting:
      /* Case to write final byte */
      if (i2c_control.write_index == i2c_control.num_write_bytes - 1) {
        // put last byte into the fifo
        I2CMasterDataPut(kBaseAddress, tx_data_buffer[i2c_control.write_index]);

        // send last byte and then stop condition
        I2CMasterControl(kBaseAddress, I2C_MASTER_CMD_BURST_SEND_FINISH);

        state = I2CDriverState::kIdle;
      } else {
        I2CMasterDataPut(kBaseAddress, tx_data_buffer[i2c_control.write_index]);

        // Initiate multi-byte transfer
        I2CMasterControl(kBaseAddress, I2C_MASTER_CMD_BURST_SEND_CONT);

        /* increment write index only if we haven't reached the end */
        i2c_control.write_index++;
      }
      break;
    case I2CDriverState::kReceiving:
      // set mode for reading
      switch (stage) {
        case I2CReceiveStage::kIdle:
          break;
        case I2CReceiveStage::kSendingAddress:
          I2CMasterSlaveAddrSet(kBaseAddress, i2c_control.device_address, true);

          if (i2c_control.num_read_bytes == 1) {
            I2CMasterControl(kBaseAddress, I2C_MASTER_CMD_SINGLE_RECEIVE);
          } else {
            I2CMasterControl(kBaseAddress, I2C_MASTER_CMD_BURST_RECEIVE_START);
          }

          stage = I2CReceiveStage::kReadingData;
          break;
        case I2CReceiveStage::kReadingData:
          /* Reading last byte */
          if (i2c_control.read_index == i2c_control.num_read_bytes - 1) {
            rx_data_buffer[i2c_control.read_index] =
                I2CMasterDataGet(kBaseAddress);
            data_ready = true;
            stage = I2CReceiveStage::kIdle;
            state = I2CDriverState::kIdle;
          } else {
            rx_data_buffer[i2c_control.read_index] =
                I2CMasterDataGet(kBaseAddress);

            I2CMasterControl(kBaseAddress, I2C_MASTER_CMD_BURST_RECEIVE_CONT);

            i2c_control.read_index++;
          }
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}

void TivaI2C::ExtractData(uint8_t *data_out) {
  /* Disable interrupts here? */
  if (data_ready) {
    std::memcpy(data_out, rx_data_buffer, i2c_control.num_read_bytes);
    data_ready = false;
  }
}