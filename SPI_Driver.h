#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include <array>
#include <cstdint>

class TivaSPI {
 public:
  TivaSPI();
  ~TivaSPI();

  enum class SPI_Internal_State : uint8_t {
    kIdle = 0,
    kWriting,
    kTransferring,
  };

  enum class SPI_Transferring_Stage : uint8_t {
    kIdle = 0,
    kWriting,
    kReading,
    kFinishing
  };

  struct SPI_Runtime_Data {
    uint8_t read_index;
    uint8_t write_index;
    uint8_t num_read_bytes;
    uint8_t num_write_bytes;
    uint16_t num_total_bytes;
  };

  void Initialize();
  void BlockingWrite(uint8_t *data_out, uint8_t num_bytes);
  void BlockingTransfer(uint32_t *data_out, uint32_t *data_in,
                        uint8_t num_bytes);
  void ReceiveCallback();
  void TransmitCallback();

  bool StartNonBlockingWrite(uint8_t *data, uint8_t num_bytes);
  bool StartNonBlockingTransfer(uint8_t *data_out, uint8_t *data_in,
                                uint8_t num_total_bytes,
                                uint8_t num_read_bytes);

 private:
  /* Base address of SPI peripheral */
  void ClearReceiveBuffer(uint32_t *buffer);
  std::uint32_t kBaseAddress;
  static constexpr uint8_t kSize{10};
  //   std::array<std::uint8_t, kSize> rx_data_buffer;
  //   std::array<std::uint8_t, kSize> tx_data_buffer;
  uint8_t rx_data_buffer[kSize];
  uint8_t tx_data_buffer[kSize];
  void AssertCS();
  void DeAssertCS();

  SPI_Runtime_Data spi_control{};
  SPI_Internal_State state{SPI_Internal_State::kIdle};
  SPI_Transferring_Stage stage{SPI_Transferring_Stage::kIdle};
};

#endif