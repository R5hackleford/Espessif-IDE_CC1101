// Minimal example: one CC1101 on SPI2, receive in a loop and send a packet every 5 s.
// Pin numbers are placeholders - change them to match your wiring. For more radios, create more
// CC1101Radio objects with their own CS pin on the same bus.
#include <stdio.h>

#include "CC1101Radio.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static constexpr int PIN_SCK = 18, PIN_MISO = 19, PIN_MOSI = 23, PIN_CS = 5;

extern "C" void app_main(void) {
  spi_bus_config_t bus = {};
  bus.sclk_io_num = PIN_SCK;
  bus.miso_io_num = PIN_MISO;
  bus.mosi_io_num = PIN_MOSI;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  bus.max_transfer_sz = 64;
  ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED));

  static CC1101Radio radio(SPI2_HOST, PIN_CS, PIN_MISO);
  int16_t rc = radio.begin(433.92f, 4.8f, 47.6f, 135.0f, CC1101Radio::MOD_GFSK);
  if (rc != CC1101Radio::ERR_NONE) {
    printf("CC1101 init failed: %d\n", rc);
    return;
  }
  printf("CC1101 part %02X version %02X\n", radio.partNumber(), radio.version());

  // Optional: hardware address filtering + Manchester (both ends must match)
  // radio.setNodeAddress(0x42);
  // radio.setAddressFilter(CC1101Radio::ADDR_NODE_BC0);
  // radio.setManchester(true);

  radio.startReceive();
  int64_t lastTx = esp_timer_get_time();
  for (;;) {
    uint8_t buf[CC1101Radio::MAX_PAYLOAD];
    CC1101Radio::PacketInfo info;
    int16_t n = radio.readPacket(buf, sizeof buf, &info);
    if (n > 0) {
      printf("RX %d bytes, RSSI %.1f dBm, LQI %u\n", n, info.rssi, info.lqi);
    } else if (n < 0 && n != CC1101Radio::ERR_CRC) {
      printf("RX error %d\n", n);
    }
    if (esp_timer_get_time() - lastTx > 5000000) {
      const char msg[] = "hello";
      int16_t r = radio.transmit((const uint8_t*)msg, sizeof msg - 1);
      printf("TX -> %d\n", r);
      radio.startReceive();  // transmit() leaves the chip idle
      lastTx = esp_timer_get_time();
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
