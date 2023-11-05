#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#include <cstdint>

class TivaI2C {
 public:
  TivaI2C();
  ~TivaI2C();

  enum class I2CDriverState : uint8_t { kIdle = 0, kTransmitting, kReceiving };
  enum class I2CReceiveStage : uint8_t {
    kIdle = 0,
    kSendingAddress,
    kReadingData
  };
  struct I2C_Runtime_Data {
    uint8_t num_write_bytes;
    uint8_t num_read_bytes;
    uint8_t read_index;
    uint8_t write_index;
    uint8_t register_address;
    uint8_t device_address;
    I2CDriverState state;
  };
  bool BlockingWrite(uint8_t *data_in, uint8_t num_bytes,
                     uint8_t device_address);
  bool BlockingRead(uint8_t *data_out, uint8_t num_bytes,
                    uint8_t device_address, uint8_t register_address);
  bool BlockingReadOnly(uint8_t *data_out, uint8_t num_bytes,
                        uint8_t device_address);
  void Initialize();
  void I2CCallback();

  void ExtractData(uint8_t *data_out);

  void StartNonBlockingTransmission(uint8_t *data_in, uint8_t num_bytes,
                                    uint8_t register_address,
                                    uint8_t device_address);
  void StartNonBlockingRead(uint8_t num_bytes, uint8_t register_address,
                            uint8_t device_address);

 private:
  void BlockingSingleWrite(uint8_t register_address, uint8_t device_address);
  std::uint32_t kBaseAddress;
  I2CDriverState state{I2CDriverState::kIdle};
  I2CReceiveStage stage{I2CReceiveStage::kIdle};
  I2C_Runtime_Data i2c_control{};
  volatile bool data_ready{false};
  uint8_t tx_data_buffer[10];
  uint8_t rx_data_buffer[10];
};

#endif
