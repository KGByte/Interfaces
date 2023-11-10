#include <SPI_Driver.h>

#include <cstring>

#include "../driverlib/gpio.h"
#include "../driverlib/interrupt.h"
#include "../driverlib/pin_map.h"
#include "../driverlib/ssi.h"
#include "../driverlib/sysctl.h"
#include "../inc/hw_ints.h"
#include "../inc/hw_memmap.h"
#include "../inc/hw_ssi.h"

TivaSPI::TivaSPI() {}
TivaSPI::~TivaSPI() {}
void TivaSPI::Initialize() {
  kBaseAddress = SSI0_BASE;
  IntEnable(INT_SSI0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_SSI0)) {
  }
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)) {
  }

  GPIOPinConfigure(GPIO_PA2_SSI0CLK);
  // GPIOPinConfigure(GPIO_PA3_SSI0FSS);
  GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);
  GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
  GPIOPinConfigure(GPIO_PA4_SSI0RX);
  GPIOPinConfigure(GPIO_PA5_SSI0TX);

  GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_2);
  // SSIIntEnable(SSI0_BASE, SSI_TXEOT);
  // SSIIntEnable(SSI0_BASE, SSI_RXFF | SSI_TXFF);
  SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                     SSI_MODE_MASTER, 1000000, 8);

  SSIEnable(SSI0_BASE);
  SSISetEndOfTransmissionBit(SSI0_BASE);
  // SSILoopbackEnable(SSI0_BASE);
}

void TivaSPI::BlockingWrite(uint8_t *data_out, uint8_t num_bytes) {
  uint8_t bytes_processed = 0;
  // ClearReceiveBuffer(&data_out[0]);
  AssertCS();
  while (bytes_processed < num_bytes) {
    SSIDataPut(kBaseAddress, data_out[bytes_processed]);

    bytes_processed++;
  }
  // Wait until bus is no longer busy before returning
  while (SSIBusy(kBaseAddress))
    ;  // look into this
  DeAssertCS();
  // check to see if all bytes have been sent and return false if not
  if (bytes_processed != num_bytes) {
    return;
  }
}
void TivaSPI::BlockingTransfer(uint32_t *data_out, uint32_t *data_in,
                               uint8_t num_bytes) {
  uint8_t bytes_processed = 0;

  ClearReceiveBuffer(&data_in[0]);

  SSIIntEnable(SSI0_BASE, SSI_TXFF);

  AssertCS();
  while (bytes_processed < num_bytes) {
    // put data into the Tx FIFO
    SSIDataPut(kBaseAddress, data_out[bytes_processed]);
    // chill out for a bit
    while (SSIBusy(kBaseAddress))
      ;

    SSIDataGet(kBaseAddress, &data_in[bytes_processed]);
    while (SSIBusy(kBaseAddress))
      ;

    bytes_processed++;
  }
  DeAssertCS();
  // check to see if all bytes have been sent and return false if not
  if (bytes_processed != num_bytes) {
    return;
  }
}

void TivaSPI::ClearReceiveBuffer(uint32_t *buffer) {
  AssertCS();
  while (SSIDataGetNonBlocking(kBaseAddress, &buffer[0]))
    ;
  DeAssertCS();
}

void TivaSPI::DeAssertCS() {
  GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
}
void TivaSPI::AssertCS() { GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0); }

bool TivaSPI::StartNonBlockingWrite(uint8_t *data, uint8_t num_bytes) {
  if (state == SPI_Internal_State::kIdle) {
    state = SPI_Internal_State::kWriting;

    std::memcpy(tx_data_buffer, data, num_bytes);
    spi_control.write_index = 0;
    spi_control.num_write_bytes = num_bytes;

    AssertCS();

    for (uint8_t index = 0; index < num_bytes; index++) {
      SSIDataPutNonBlocking(SSI0_BASE, tx_data_buffer[index]);
    }

    SSIIntEnable(SSI0_BASE, SSI_TXFF);

    return true;
  } else {
    return false;
  }
}

bool TivaSPI::StartNonBlockingTransfer(uint8_t *data_out, uint8_t *data_in,
                                       uint8_t num_write_bytes,
                                       uint8_t num_read_bytes) {
  if (state == SPI_Internal_State::kIdle) {
    state = SPI_Internal_State::kTransferring;

    spi_control.write_index = 0;
    spi_control.read_index = 0;
    spi_control.num_read_bytes = num_read_bytes;
    spi_control.num_write_bytes = num_write_bytes;
    spi_control.num_total_bytes = num_read_bytes + num_write_bytes;
    std::memcpy(tx_data_buffer, data_out, spi_control.num_total_bytes);

    /* If only writing to 1 register during the transfer, go to reading step
     * next. Else, signal that more writing bytes (not dummy data) are going
     * out.*/
    if (num_write_bytes == 1) {
      stage = SPI_Transferring_Stage::kReading;
    } else {
      stage = SPI_Transferring_Stage::kWriting;
    }

    AssertCS();

    SSIDataPutNonBlocking(SSI0_BASE, tx_data_buffer[spi_control.write_index++]);

    SSIIntEnable(SSI0_BASE, SSI_TXFF);

    return true;
  } else {
    return false;
  }
}

void TivaSPI::ReceiveCallback() {}
void TivaSPI::TransmitCallback() {
  switch (state) {
    case SPI_Internal_State::kIdle: {
      break;
    }

    case SPI_Internal_State::kWriting: {
      state = SPI_Internal_State::kIdle;

      DeAssertCS();
      break;
    }

    case SPI_Internal_State::kTransferring: {
      switch (stage) {
        case SPI_Transferring_Stage::kWriting: {
          if (spi_control.write_index == spi_control.num_write_bytes - 1) {
            /* finished writing - move to reading */
            stage = SPI_Transferring_Stage::kReading;
          }
          SSIDataPutNonBlocking(kBaseAddress,
                                tx_data_buffer[spi_control.write_index++]);

          break;
        }

        case SPI_Transferring_Stage::kReading: {
          if (spi_control.num_read_bytes - 1 == spi_control.read_index) {
            SSIDataPutNonBlocking(kBaseAddress,
                                  tx_data_buffer[spi_control.write_index]);
            SSIDataGetNonBlocking(
                kBaseAddress,
                (uint32_t *)&rx_data_buffer[spi_control.read_index]);
            stage = SPI_Transferring_Stage::kFinishing;
          } else {
            /* Dummy bytes into FIFO */
            SSIDataPutNonBlocking(kBaseAddress,
                                  tx_data_buffer[spi_control.write_index++]);
            /* Get data from receive FIFO and store into local array */
            SSIDataGetNonBlocking(
                kBaseAddress,
                (uint32_t *)&rx_data_buffer[spi_control.read_index++]);
          }
          break;
        }

        case SPI_Transferring_Stage::kFinishing: {
          DeAssertCS();
          stage = SPI_Transferring_Stage::kIdle;
          state = SPI_Internal_State::kIdle;
          break;
        }

        default:
          break;
      }
    }

    default:
      break;
  }
  // SSIDataPutNonBlocking(SSI0_BASE,
  // tx_data_buffer[spi_control.write_index++]);
  // spi_control.write_index++;
  // if (!SSIBusy(SSI0_BASE)) {
  //   SSIDataPut(SSI0_BASE, tx_data_buffer[spi_control.write_index]);
  //   spi_control.write_index++;
  // } else {
  //   return;
  // }
  // state = SPI_Internal_State::kIdle;
  // // SSIIntDisable(SSI0_BASE, SSI_TXFF);
  // DeAssertCS();

  // if (spi_control.write_index == spi_control.num_write_bytes - 1) {
  //   SSIIntDisable(SSI0_BASE, SSI_TXFF);
  // }
}