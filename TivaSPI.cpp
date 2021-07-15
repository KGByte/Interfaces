
#include "TivaSPI.h"

TivaSPI::TivaSPI(uint32_t base_address, Pin *cs_pin, uint32_t clk_speed, uint8_t clk_phase, uint8_t clk_pol, uint8_t dff) : SPIInterface(cs_pin, clk_speed, clk_phase, clk_pol, dff),
    spi_base_address(base_address)
{}

void TivaSPI::Initialize()
{
    uint8_t phase = GetClkPhase();
    uint8_t polarity = GetClkPolarity();

    switch (phase)
    {
        case 0:
            if (polarity == 0)
            {
                data_frame_format = SSI_FRF_MOTO_MODE_0; //phase 0, polarity 0
            }
            else
            {
                data_frame_format = SSI_FRF_MOTO_MODE_2; //phase 0, polarity 1
            }
            break;
        
        case 1:
            if (polarity == 0)
            {
                data_frame_format = SSI_FRF_MOTO_MODE_1; //phase 1, polarity 0
            }
            else
            {
                data_frame_format = SSI_FRF_MOTO_MODE_3; //phase 1, polarity 1
            }
            break;     
    }

	SSIConfigSetExpClk(spi_base_address, SysCtlClockGet(), data_frame_format, SSI_MODE_MASTER, clock_speed, 8);

	SSIEnable(spi_base_address);
}

/**
 * @brief This function performs a SPI write to a designated register address
 * 
 * @param data_out Data to be written out
 * @param num_bytes Number of bytes transmitted
 * @param register_address Address you want to write data to
 * @return true if all bytes were sent
 * @return false if not all bytes were sent
 */
bool TivaSPI::Write(uint32_t *data_out, uint8_t num_bytes)
{
    uint8_t bytes_processed = 0;

    CS->SetOutput(Pin::kLow);
    while (bytes_processed < num_bytes)
    {
        SSIDataPut(spi_base_address, data_out[bytes_processed]);

        bytes_processed++;
    }
    //Wait until bus is no longer busy before returning
    while (SSIBusy(spi_base_address))
        ; //look into this
    CS->SetOutput(Pin::kHigh);
    //check to see if all bytes have been sent and return false if not
    if (bytes_processed != num_bytes)
    {
        return false;
    }

    return true;
}

/**
 * @brief This function initiates a SPI Tranfser from a specified register address
 *        and stores the data in data_in
 * 
 * @param data_out Data to be written
 * @param data_in  Data to be read in 
 * @param num_bytes Number of bytes in the transfer
 * @param register_address Address where you want to write/read from
 * @return true if all bytes are processed
 * @return false if not all bytes are processed
 */
bool TivaSPI::Transfer(uint32_t *data_out, uint32_t *data_in, uint8_t num_bytes)
{
    uint8_t bytes_processed = 0;

    ClearReceiveBuffer(&data_in[0]);

    CS->SetOutput(Pin::kLow);
    while (bytes_processed < num_bytes)
    {
        //put data into the Tx FIFO
        SSIDataPut(spi_base_address, data_out[bytes_processed]);
        //chill out for a bit
        while (SSIBusy(spi_base_address))
            ; 

        SSIDataGet(spi_base_address, &data_in[bytes_processed]);
        while (SSIBusy(spi_base_address))
            ;

        bytes_processed++;
    }
    CS->SetOutput(Pin::kHigh);
    //check to see if all bytes have been sent and return false if not
    if (bytes_processed != num_bytes)
    {
        return false;
    }

    return true;
}
/**
 * @brief This function reads any residual data from the SSI buffer.  This makes sure the receive
    FIFOs are empty, so we don't read any unwanted junk.
    The "non-blocking" function checks if there is any data in the receive
    FIFO and does not "hang" if there isn't.
 * 
 * @param buffer Buffer to store any residual data inside the RX Buffer
 */
void TivaSPI::ClearReceiveBuffer(uint32_t *buffer)
{
    CS->SetOutput(Pin::kLow);
    while (SSIDataGetNonBlocking(spi_base_address, &buffer[0]))
        ;
    CS->SetOutput(Pin::kHigh);
}

uint8_t TivaSPI::GetClkPolarity()
{
    return clock_polarity;
}

uint8_t TivaSPI::GetClkPhase()
{
    return clock_phase;
}

void TivaSPI::SetCSLow()
{
    CS->SetOutput(Pin::kLow);
}
void TivaSPI::SetCSHigh()
{
    CS->SetOutput(Pin::kHigh);
}
