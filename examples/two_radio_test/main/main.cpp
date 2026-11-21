#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "CC1101Radio.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
constexpr int PIN_SCK = 18, PIN_MISO = 19, PIN_MOSI = 23;
constexpr int PIN_CS_A = 25, PIN_CS_B = 26;
constexpr float FREQUENCY_MHZ = 433.92f;
constexpr int RECEIVE_TIMEOUT_MS = 250;

CC1101Radio radioA(SPI2_HOST, PIN_CS_A, PIN_MISO);
CC1101Radio radioB(SPI2_HOST, PIN_CS_B, PIN_MISO);

void help() {
  printf("\nCommands:\n"
         "  help              Show this menu\n"
         "  status            Show both radios\n"
         "  send a <text>     Send from A to B (1-61 bytes)\n"
         "  send b <text>     Send from B to A (1-61 bytes)\n"
         "  test [count]      Test both directions (default 5, max 100)\n");
}

void status() {
  CC1101Radio* radios[] = {&radioA, &radioB};
  for (int i = 0; i < 2; ++i) {
    CC1101Radio& r = *radios[i];
    printf("%c: part=0x%02X version=0x%02X state=0x%02X frequency=%.2f MHz\n",
           'A' + i, r.partNumber(), r.version(), r.marcState(), r.frequency());
  }
}

bool exchange(CC1101Radio& sender, CC1101Radio& receiver, const uint8_t* data,
              uint8_t length) {
  if (!receiver.startReceive()) {
    printf("FAIL: receiver could not enter RX\n");
    return false;
  }
  int16_t rc = sender.transmit(data, length);
  if (rc != CC1101Radio::ERR_NONE) {
    printf("FAIL: transmit error %d\n", rc);
    return false;
  }
  int64_t deadline = esp_timer_get_time() + RECEIVE_TIMEOUT_MS * 1000LL;
  while (esp_timer_get_time() < deadline) {
    uint8_t received[CC1101Radio::MAX_PAYLOAD];
    CC1101Radio::PacketInfo info = {};
    int16_t n = receiver.readPacket(received, sizeof(received), &info);
    if (n < 0) {
      printf("FAIL: receive error %d\n", n);
      return false;
    }
    if (n > 0) {
      if (n != length || memcmp(received, data, length) != 0) {
        printf("FAIL: received unexpected payload (%d bytes)\n", n);
        return false;
      }
      printf("PASS: %d bytes, RSSI %.1f dBm, LQI %u\n", n, info.rssi, info.lqi);
      return true;
    }
    vTaskDelay(1);
  }
  printf("FAIL: receive timeout\n");
  return false;
}

void runTest(int count) {
  int passed = 0;
  for (int i = 1; i <= count; ++i) {
    char payload[32];
    int length = snprintf(payload, sizeof(payload), "CC1101 test %d", i);
    printf("A -> B %d/%d: ", i, count);
    if (exchange(radioA, radioB, reinterpret_cast<const uint8_t*>(payload), length)) ++passed;
    printf("B -> A %d/%d: ", i, count);
    if (exchange(radioB, radioA, reinterpret_cast<const uint8_t*>(payload), length)) ++passed;
    vTaskDelay(1);
  }
  printf("Result: %d/%d packets passed\n", passed, count * 2);
}

void command(char* line) {
  if (strcmp(line, "help") == 0) {
    help();
  } else if (strcmp(line, "status") == 0) {
    status();
  } else if (strcmp(line, "test") == 0) {
    runTest(5);
  } else if (strncmp(line, "test ", 5) == 0) {
    char* end;
    long count = strtol(line + 5, &end, 10);
    if (end == line + 5 || *end != '\0' || count < 1 || count > 100) {
      printf("Invalid count: use 1-100\n");
    } else {
      runTest(static_cast<int>(count));
    }
  } else if (strncmp(line, "send ", 5) == 0 && (line[5] == 'a' || line[5] == 'b') &&
             line[6] == ' ') {
    const char* text = line + 7;
    size_t length = strlen(text);
    if (length == 0 || length > CC1101Radio::MAX_PAYLOAD) {
      printf("Payload must be 1-61 bytes\n");
    } else {
      CC1101Radio& sender = line[5] == 'a' ? radioA : radioB;
      CC1101Radio& receiver = line[5] == 'a' ? radioB : radioA;
      printf("%c -> %c: ", toupper(line[5]), line[5] == 'a' ? 'B' : 'A');
      exchange(sender, receiver, reinterpret_cast<const uint8_t*>(text),
               static_cast<uint8_t>(length));
    }
  } else if (*line) {
    printf("Unknown command; type help\n");
  }
}
}  // namespace

extern "C" void app_main() {
  // Both devices must be deselected before either is initialized.
  gpio_config_t cs = {};
  cs.pin_bit_mask = (1ULL << PIN_CS_A) | (1ULL << PIN_CS_B);
  cs.mode = GPIO_MODE_OUTPUT;
  ESP_ERROR_CHECK(gpio_config(&cs));
  ESP_ERROR_CHECK(gpio_set_level(static_cast<gpio_num_t>(PIN_CS_A), 1));
  ESP_ERROR_CHECK(gpio_set_level(static_cast<gpio_num_t>(PIN_CS_B), 1));

  spi_bus_config_t bus = {};
  bus.sclk_io_num = PIN_SCK;
  bus.miso_io_num = PIN_MISO;
  bus.mosi_io_num = PIN_MOSI;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  bus.max_transfer_sz = 64;
  ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED));
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 1024, 0, 0, nullptr, 0));

  int16_t a = radioA.begin(FREQUENCY_MHZ);
  int16_t b = radioB.begin(FREQUENCY_MHZ);
  if (a != CC1101Radio::ERR_NONE || b != CC1101Radio::ERR_NONE) {
    printf("Radio initialization failed: A=%d B=%d. Check power, wiring and CS pins.\n", a, b);
    return;
  }
  printf("\nTwo-radio CC1101 test console (433.92 MHz, GFSK, 4.8 kbaud)\n");
  status();
  help();
  printf("> ");
  fflush(stdout);

  char line[128];
  size_t used = 0;
  bool overflow = false;
  for (;;) {
    uint8_t c;
    int n = uart_read_bytes(UART_NUM_0, &c, 1, pdMS_TO_TICKS(20));
    if (n < 0) {
      printf("\nUART read failed\n");
      return;
    }
    if (n == 0) continue;
    if (c == '\r' || c == '\n') {
      if (used == 0 && !overflow) continue;
      printf("\n");
      if (overflow) {
        printf("Command too long\n");
      } else {
        line[used] = '\0';
        command(line);
      }
      used = 0;
      overflow = false;
      printf("> ");
      fflush(stdout);
    } else if (c == '\b' || c == 127) {
      if (!overflow && used > 0) {
        --used;
        printf("\b \b");
        fflush(stdout);
      }
    } else if (c >= 32 && c <= 126) {
      if (used + 1 < sizeof(line) && !overflow) {
        line[used++] = static_cast<char>(c);
        putchar(c);
        fflush(stdout);
      } else {
        overflow = true;
      }
    }
  }
}
