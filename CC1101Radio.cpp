#include "CC1101Radio.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

using namespace CC1101Reg;

namespace {
// Small timing helpers so the driver logic reads the same as the Arduino version.
inline uint32_t micros() { return (uint32_t)esp_timer_get_time(); }
inline uint32_t millis() { return (uint32_t)(esp_timer_get_time() / 1000); }
inline void delayMicroseconds(uint32_t us) { esp_rom_delay_us(us); }
inline void delay(uint32_t ms) {
  if (ms < 10) esp_rom_delay_us(ms * 1000);  // shorter than a typical FreeRTOS tick
  else vTaskDelay(pdMS_TO_TICKS(ms));
}
constexpr double XOSC_HZ = 26000000.0;  // CC1101 modules use a 26 MHz crystal

// SPI header bits
constexpr uint8_t BIT_READ = 0x80;
constexpr uint8_t BIT_BURST = 0x40;

// Burst access is limited to 6.5 MHz when there is no delay between bytes (Table 22).
constexpr uint32_t SPI_MAX_HZ = 6500000UL;

// Output power steps and matching PATABLE bytes.
// 315/433/868/915 *_ML: datasheet Table 39 (multi-layer inductors).
// 868/915 *_WW: datasheet Table 37 (wire-wound inductors).
// Datasheet: PATABLE values 0x61-0x6F are not allowed.
constexpr int8_t PA_DBM[8] = {-30, -20, -15, -10, 0, 5, 7, 10};
constexpr uint8_t PA_315[8] = {0x12, 0x0D, 0x1C, 0x34, 0x51, 0x85, 0xCB, 0xC2};
constexpr uint8_t PA_433[8] = {0x12, 0x0E, 0x1D, 0x34, 0x60, 0x84, 0xC8, 0xC0};
constexpr uint8_t PA_868_ML[8] = {0x03, 0x0F, 0x1E, 0x27, 0x50, 0x81, 0xCB, 0xC2};
constexpr uint8_t PA_915_ML[8] = {0x03, 0x0E, 0x1E, 0x27, 0x8E, 0xCD, 0xC7, 0xC0};
constexpr uint8_t PA_868_WW[8] = {0x03, 0x17, 0x1D, 0x26, 0x50, 0x86, 0xCD, 0xC5};
constexpr uint8_t PA_915_WW[8] = {0x03, 0x0E, 0x1E, 0x27, 0x8E, 0x84, 0xCC, 0xC3};

// NUM_PREAMBLE codes 0..7 -> bytes (datasheet MDMCFG1)
constexpr uint8_t PREAMBLE_BYTES[8] = {2, 3, 4, 6, 8, 12, 16, 24};

// Register defaults based on TI SmartRF Studio output (433 MHz, GFSK, 4.8 kbps).
// Frequency, bitrate, deviation, bandwidth, modulation and PA are applied afterwards.
struct RegVal { uint8_t reg; uint8_t val; };
constexpr RegVal DEFAULTS[] = {
    {IOCFG0, 0x06},    // GDO0: asserts on sync word, deasserts at end of packet
    {FIFOTHR, 0x47},   // ADC_RETENTION=1 (RX BW < 325 kHz), FIFO_THR=7
    {SYNC1, 0xD3},     {SYNC0, 0x91},
    {PKTLEN, 0x3D},    // max packet length 61
    {PKTCTRL1, 0x04},  // append RSSI/LQI/CRC-OK status bytes, no address check
    {PKTCTRL0, 0x05},  // variable length, CRC on, no whitening
    {ADDR, 0x00},
    {FSCTRL1, 0x06},   {FSCTRL0, 0x00},
    {MDMCFG2, 0x02},   // placeholder, setModulation() sets MOD_FORMAT; 16/16 sync, no Manchester
    {MDMCFG1, 0x22},   // no FEC, 4 preamble bytes, channel spacing exponent 2
    {MDMCFG0, 0xF8},
    {MCSM1, 0x00},     // CCA always, go to IDLE after RX and after TX
    {MCSM0, 0x18},     // auto-calibrate on IDLE -> RX/TX
    {FOCCFG, 0x16},    {BSCFG, 0x6C},
    {AGCCTRL2, 0x43},  {AGCCTRL1, 0x40}, {AGCCTRL0, 0x91},
    {WORCTRL, 0xFB},
    {FREND1, 0x56},
    {FSCAL3, 0xE9},    {FSCAL2, 0x2A},   {FSCAL1, 0x00}, {FSCAL0, 0x1F},
    {FSTEST, 0x59},    {TEST2, 0x81},    {TEST1, 0x35},  {TEST0, 0x09},
};

uint8_t paIndexForDbm(int8_t dbm) {
  uint8_t best = 0;
  int bestDiff = 1000;
  for (uint8_t i = 0; i < 8; i++) {
    int d = abs((int)PA_DBM[i] - (int)dbm);
    if (d < bestDiff) {
      bestDiff = d;
      best = i;
    }
  }
  return best;
}

// Bytes of sync word sent/expected for a SYNC_MODE value (0 = no preamble/sync at all).
uint8_t syncBytesFor(CC1101Radio::SyncMode m) {
  switch (m) {
    case CC1101Radio::SYNC_NONE:
    case CC1101Radio::SYNC_CS_ONLY:
      return 0;
    case CC1101Radio::SYNC_30_32:
    case CC1101Radio::SYNC_30_32_CS:
      return 4;
    default:
      return 2;
  }
}
}  // namespace

CC1101Radio::CC1101Radio(spi_host_device_t host, int csPin, int misoPin, uint32_t spiHz)
    : _host(host), _cs(csPin), _miso(misoPin), _spiHz(spiHz > SPI_MAX_HZ ? SPI_MAX_HZ : spiHz) {}

CC1101Radio::~CC1101Radio() {
  if (_dev) spi_bus_remove_device(_dev);
}

// ---------------------------------------------------------------- low-level SPI

// Adds this chip to the (already initialised) SPI bus. CS is a plain GPIO so that a whole
// register access, and the wait for the chip-ready signal, happen inside one CS-low period.
bool CC1101Radio::attach() {
  if (_dev) return true;
  gpio_config_t io = {};
  io.pin_bit_mask = 1ULL << _cs;
  io.mode = GPIO_MODE_OUTPUT;
  if (gpio_config(&io) != ESP_OK) return false;
  gpio_set_level((gpio_num_t)_cs, 1);

  spi_device_interface_config_t dc = {};
  dc.mode = 0;
  dc.clock_speed_hz = (int)_spiHz;
  dc.spics_io_num = -1;
  dc.queue_size = 1;
  return spi_bus_add_device(_host, &dc, &_dev) == ESP_OK;
}

void CC1101Radio::select() {
  spi_device_acquire_bus(_dev, portMAX_DELAY);
  gpio_set_level((gpio_num_t)_cs, 0);
  if (_miso >= 0) {  // wait for the chip's crystal-ready signal (MISO low)
    uint32_t t = micros();
    while (gpio_get_level((gpio_num_t)_miso) && (micros() - t) < 2000) {
    }
  }
}

void CC1101Radio::deselect() {
  gpio_set_level((gpio_num_t)_cs, 1);
  spi_device_release_bus(_dev);
}

// Full-duplex transfer of n bytes while CS is held low. Split into <= 64-byte pieces, which the
// driver handles from the hardware FIFO without DMA-capable buffers. tx/rx may be nullptr.
void CC1101Radio::xfer(const uint8_t* tx, uint8_t* rx, size_t n) {
  size_t off = 0;
  while (off < n) {
    size_t chunk = (n - off > 64) ? 64 : (n - off);
    spi_transaction_t t = {};
    t.length = chunk * 8;
    t.tx_buffer = tx ? tx + off : nullptr;
    t.rx_buffer = rx ? rx + off : nullptr;
    spi_device_polling_transmit(_dev, &t);
    off += chunk;
  }
}

// CS must already be low. Used after reset and after SLEEP, where the crystal needs time to
// start. Without a MISO pin we fall back to a fixed delay (datasheet Table 22: >= 150 us).
void CC1101Radio::waitReady(uint32_t timeoutUs) {
  if (_miso >= 0) {
    uint32_t t = micros();
    while (gpio_get_level((gpio_num_t)_miso) && (micros() - t) < timeoutUs) {
    }
  } else {
    delayMicroseconds(timeoutUs > 1000 ? 1000 : timeoutUs);
  }
}

uint8_t CC1101Radio::readReg(uint8_t reg) {
  uint8_t tx[2] = {(uint8_t)(reg | BIT_READ), 0x00}, rx[2];
  select();
  xfer(tx, rx, 2);
  deselect();
  return rx[1];
}

void CC1101Radio::writeReg(uint8_t reg, uint8_t value) {
  uint8_t tx[2] = {reg, value};
  select();
  xfer(tx, nullptr, 2);
  deselect();
}

void CC1101Radio::rmwReg(uint8_t reg, uint8_t mask, uint8_t value) {
  uint8_t v = readReg(reg);
  writeReg(reg, (uint8_t)((v & ~mask) | (value & mask)));
}

// Status registers share addresses with strobes; the burst bit selects the register.
uint8_t CC1101Radio::readStatus(uint8_t reg) {
  uint8_t tx[2] = {(uint8_t)(reg | BIT_READ | BIT_BURST), 0x00}, rx[2];
  select();
  xfer(tx, rx, 2);
  deselect();
  return rx[1];
}

// Registers updated by the radio itself (MARCSTATE, RXBYTES, TXBYTES, PKTSTATUS) can be read
// corrupt with a small probability (datasheet 10.3 / 15.6, errata). Read until two reads agree.
uint8_t CC1101Radio::readStatusStable(uint8_t reg) {
  uint8_t a = readStatus(reg);
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t b = readStatus(reg);
    if (a == b) return b;
    a = b;
  }
  return a;
}

uint8_t CC1101Radio::readMarc() { return readStatusStable(MARCSTATE) & 0x1F; }

uint8_t CC1101Radio::strobe(uint8_t cmd) {
  uint8_t rx;
  select();
  xfer(&cmd, &rx, 1);
  deselect();
  return rx;
}

// Burst transfers are at most 64 data bytes (the FIFO size).
void CC1101Radio::readBurst(uint8_t reg, uint8_t* buf, uint8_t len) {
  if (len > 64) len = 64;
  uint8_t tx[65] = {0}, rx[65];
  tx[0] = reg | BIT_READ | BIT_BURST;
  select();
  xfer(tx, rx, (size_t)len + 1);
  deselect();
  memcpy(buf, rx + 1, len);
}

void CC1101Radio::writeBurst(uint8_t reg, const uint8_t* buf, uint8_t len) {
  if (len > 64) len = 64;
  uint8_t tx[65];
  tx[0] = reg | BIT_BURST;
  memcpy(tx + 1, buf, len);
  select();
  xfer(tx, nullptr, (size_t)len + 1);
  deselect();
}

// ---------------------------------------------------------------- init

// Manual reset, datasheet 19.1.2 / Figure 27: CS low/high, hold high >= 40 us, CS low, wait for
// SO low, SRES with CS held low, wait for SO low again, CS high.
void CC1101Radio::reset() {
  _asleep = false;
  gpio_set_level((gpio_num_t)_cs, 1);
  delayMicroseconds(5);
  gpio_set_level((gpio_num_t)_cs, 0);
  delayMicroseconds(10);
  gpio_set_level((gpio_num_t)_cs, 1);
  delayMicroseconds(45);

  uint8_t cmd = SRES;
  spi_device_acquire_bus(_dev, portMAX_DELAY);
  gpio_set_level((gpio_num_t)_cs, 0);
  waitReady(5000);
  xfer(&cmd, nullptr, 1);
  delayMicroseconds(10);
  waitReady(5000);
  gpio_set_level((gpio_num_t)_cs, 1);
  spi_device_release_bus(_dev);
  delay(1);
}

int16_t CC1101Radio::begin(float freqMHz, float bitrateKbps, float devKHz, float rxBwKHz,
                           Modulation mod) {
  if (!attach()) return ERR_SPI;
  reset();

  uint8_t ver = readStatus(VERSION);
  if (ver == 0x00 || ver == 0xFF) return ERR_CHIP_NOT_FOUND;  // genuine parts report 0x04/0x14

  // The reset above put every register back to its default; bring our shadows in line.
  _mod = MOD_GFSK;
  _manchester = false;
  _adrChk = ADDR_OFF;
  _nodeAddr = 0;
  _preambleBytes = 4;
  _syncMode = SYNC_16_16;
  _focLimitFsk = 2;
  _agcFsk[0] = 0x43; _agcFsk[1] = 0x40; _agcFsk[2] = 0x91;
  _frend1Fsk = 0x56;
  _lenMode = LEN_VARIABLE;
  _fixedLen = 0;
  _fec = false;
  _paIsRaw = false;
  _asleep = false;

  for (const RegVal& rv : DEFAULTS) writeReg(rv.reg, rv.val);

  if (!setModulation(mod)) return ERR_BAD_ARG;
  if (!setBitrate(bitrateKbps)) return ERR_BAD_ARG;
  // DEVIATN is meaningless for ASK/OOK. For MSK it is the phase-transition fraction, and 0 is
  // the SmartRF value for 500 kBd.
  if ((mod == MOD_2FSK || mod == MOD_GFSK || mod == MOD_4FSK) && !setDeviation(devKHz))
    return ERR_BAD_ARG;
  if (mod == MOD_MSK) writeReg(DEVIATN, 0x00);
  if (!setRxBandwidth(rxBwKHz)) return ERR_BAD_ARG;
  if (!setFrequency(freqMHz)) return ERR_BAD_ARG;
  setOutputPowerDbm(0);
  return ERR_NONE;
}

// ---------------------------------------------------------------- configuration

bool CC1101Radio::setFrequency(float mhz) {
  bool ok = (mhz >= 300.0f && mhz <= 348.0f) || (mhz >= 387.0f && mhz <= 464.0f) ||
            (mhz >= 779.0f && mhz <= 928.0f);
  if (!ok) return false;

  // FREQ = f_carrier * 2^16 / f_xosc  (resolution ~397 Hz)
  uint32_t word = (uint32_t)((double)mhz * 1e6 * 65536.0 / XOSC_HZ + 0.5);
  idle();
  uint8_t b[3] = {(uint8_t)(word >> 16), (uint8_t)(word >> 8), (uint8_t)word};
  writeBurst(FREQ2, b, 3);

  // The recommended PATABLE depends on the band, so re-pick it if we crossed into another one.
  const uint8_t* before = paTable();
  _freqMHz = mhz;
  if (!_paIsRaw && paTable() != before) refreshPaFromDbm();
  return true;
}

// Baud-rate limits per modulation, datasheet Table 3.
bool CC1101Radio::bitrateSupported(Modulation m, float kbps) {
  float lo = 0.6f, hi = 500.0f;
  switch (m) {
    case MOD_GFSK:
    case MOD_ASK_OOK:
      hi = 250.0f;
      break;
    case MOD_4FSK:
      hi = 300.0f;
      break;
    case MOD_MSK:
      lo = 26.0f;
      break;
    default:
      break;
  }
  return kbps >= lo && kbps <= hi;
}

bool CC1101Radio::setBitrate(float kbps) {
  if (!bitrateSupported(_mod, kbps)) return false;
  idle();
  double r = (double)kbps * 1000.0;
  // R = (256 + M) * 2^E / 2^28 * f_xosc
  int e = (int)floor(log(r * 1048576.0 / XOSC_HZ) / log(2.0));
  if (e < 0) e = 0;
  if (e > 15) e = 15;
  int m = (int)lround(r * 268435456.0 / (XOSC_HZ * (double)(1UL << e))) - 256;
  if (m > 255) {  // rounded up to 256: bump the exponent, mantissa 0 (datasheet section 12)
    m = 0;
    if (e < 15) e++;
    else m = 255;
  }
  if (m < 0) m = 0;

  writeReg(MDMCFG3, (uint8_t)m);
  rmwReg(MDMCFG4, 0x0F, (uint8_t)e);
  _bitrateBps = (256.0 + m) * (double)(1UL << e) / 268435456.0 * XOSC_HZ;
  return true;
}

bool CC1101Radio::setDeviation(float kHz) {
  if (kHz < 1.5f || kHz > 381.0f) return false;
  idle();
  // f_dev = f_xosc / 2^17 * (8 + M) * 2^E, DEVIATION_E is 3 bits (0..7), DEVIATION_M is 3 bits
  double target = (double)kHz * 1000.0;
  double bestErr = 1e18;
  uint8_t bestE = 0, bestM = 0;
  for (uint8_t e = 0; e < 8; e++) {
    for (uint8_t m = 0; m < 8; m++) {
      double dev = XOSC_HZ / 131072.0 * (8 + m) * (double)(1 << e);
      double err = fabs(dev - target);
      if (err < bestErr) {
        bestErr = err;
        bestE = e;
        bestM = m;
      }
    }
  }
  writeReg(DEVIATN, (uint8_t)((bestE << 4) | bestM));
  return true;
}

// TI DN022 section 3.2: for RX bandwidth <= 325 kHz use FIFOTHR 0x47 (ADC_RETENTION=1), TEST2 0x81,
// TEST1 0x35; above 325 kHz use FIFOTHR 0x07, TEST2 0x88, TEST1 0x31. For OOK/ASK it also gives
// FREND1 = 0xB6 above 101 kHz and 0x56 at or below (the 101.56 kHz table entry is the "100 kHz"
// SmartRF preset, hence the 102 kHz cut-off).
void CC1101Radio::updateRxAnalogForBw(float actualBwKHz) {
  _rxBwKHz = actualBwKHz;
  bool narrow = actualBwKHz <= 325.0f;
  rmwReg(FIFOTHR, 0x40, narrow ? 0x40 : 0x00);
  writeReg(TEST2, narrow ? 0x81 : 0x88);
  writeReg(TEST1, narrow ? 0x35 : 0x31);
  if (_mod == MOD_ASK_OOK) writeReg(FREND1, actualBwKHz <= 102.0f ? 0x56 : 0xB6);
}

bool CC1101Radio::setRxBandwidth(float kHz) {
  if (kHz < 58.0f || kHz > 812.0f) return false;
  idle();
  // BW = f_xosc / (8 * (4 + M) * 2^E)
  double target = (double)kHz * 1000.0;
  double bestErr = 1e18, bestBw = 0;
  uint8_t bestE = 0, bestM = 0;
  for (uint8_t e = 0; e < 4; e++) {
    for (uint8_t m = 0; m < 4; m++) {
      double bw = XOSC_HZ / (8.0 * (4 + m) * (double)(1 << e));
      double err = fabs(bw - target);
      if (err < bestErr) {
        bestErr = err;
        bestE = e;
        bestM = m;
        bestBw = bw;
      }
    }
  }
  rmwReg(MDMCFG4, 0xF0, (uint8_t)((bestE << 6) | (bestM << 4)));
  updateRxAnalogForBw((float)(bestBw / 1000.0));
  return true;
}

bool CC1101Radio::setModulation(Modulation m) {
  if (m != MOD_2FSK && m != MOD_GFSK && m != MOD_ASK_OOK && m != MOD_4FSK && m != MOD_MSK)
    return false;
  idle();

  // Manchester is not supported with MSK or 4-FSK (datasheet 16 / 16.2).
  if (m == MOD_MSK || m == MOD_4FSK) _manchester = false;

  // MOD_FORMAT is bits 6:4. Keep DEM_DCFILT_OFF (bit 7) and SYNC_MODE (bits 2:0).
  uint8_t v = readReg(MDMCFG2);
  v = (uint8_t)((v & 0x87) | (m << 4) | (_manchester ? 0x08 : 0x00));
  writeReg(MDMCFG2, v);

  // Frequency offset compensation is not supported for ASK/OOK: FOC_LIMIT must be 0 (FOCCFG).
  bool wasAsk = (_mod == MOD_ASK_OOK);
  uint8_t foc = readReg(FOCCFG);
  if (m == MOD_ASK_OOK && !wasAsk) {
    _focLimitFsk = foc & 0x03;
    writeReg(FOCCFG, (uint8_t)(foc & ~0x03));
  } else if (m != MOD_ASK_OOK && wasAsk) {
    writeReg(FOCCFG, (uint8_t)((foc & ~0x03) | _focLimitFsk));
  }

  // OOK/ASK needs different AGC settings than FSK (TI DN022). Save/restore the FSK ones.
  if (m == MOD_ASK_OOK && !wasAsk) {
    _agcFsk[0] = readReg(AGCCTRL2);
    _agcFsk[1] = readReg(AGCCTRL1);
    _agcFsk[2] = readReg(AGCCTRL0);
    _frend1Fsk = readReg(FREND1);
    writeReg(AGCCTRL2, 0x04);  // whole byte, so MAX_DVGA_GAIN / MAX_LNA_GAIN are unrestricted
    writeReg(AGCCTRL1, 0x00);
    writeReg(AGCCTRL0, 0x92);
    writeReg(FREND1, _rxBwKHz <= 102.0f ? 0x56 : 0xB6);
  } else if (m != MOD_ASK_OOK && wasAsk) {
    writeReg(AGCCTRL2, _agcFsk[0]);
    writeReg(AGCCTRL1, _agcFsk[1]);
    writeReg(AGCCTRL0, _agcFsk[2]);
    writeReg(FREND1, _frend1Fsk);
  }

  _mod = m;
  applyPa();
  return true;
}

bool CC1101Radio::setOokAgc(uint8_t magnTarget, uint8_t agcCtrl0) {
  if (_mod != MOD_ASK_OOK || magnTarget > 7) return false;
  idle();
  writeReg(AGCCTRL2, magnTarget);
  writeReg(AGCCTRL1, 0x00);
  writeReg(AGCCTRL0, agcCtrl0);
  return true;
}

bool CC1101Radio::setMskPhaseTransition(uint8_t m) {
  if (_mod != MOD_MSK || m > 7) return false;
  idle();
  writeReg(DEVIATN, m);
  return true;
}

bool CC1101Radio::setChannelSpacing(float kHz) {
  if (kHz < 25.0f || kHz > 406.0f) return false;
  idle();
  // f_chan = f_xosc / 2^18 * (256 + M) * 2^E
  double target = (double)kHz * 1000.0;
  double bestErr = 1e18;
  uint8_t bestE = 0, bestM = 0;
  for (uint8_t e = 0; e < 4; e++) {
    for (uint16_t m = 0; m < 256; m++) {
      double cs = XOSC_HZ / 262144.0 * (256 + m) * (double)(1 << e);
      double err = fabs(cs - target);
      if (err < bestErr) {
        bestErr = err;
        bestE = e;
        bestM = (uint8_t)m;
      }
    }
  }
  rmwReg(MDMCFG1, 0x03, bestE);
  writeReg(MDMCFG0, bestM);
  return true;
}

void CC1101Radio::setChannel(uint8_t channel) {
  idle();
  writeReg(CHANNR, channel);
}

// ---------------------------------------------------------------- packet handling

void CC1101Radio::setSyncWord(uint8_t hi, uint8_t lo) {
  idle();
  writeReg(SYNC1, hi);
  writeReg(SYNC0, lo);
}

bool CC1101Radio::setSyncMode(SyncMode mode) {
  if ((uint8_t)mode > 7) return false;
  idle();
  rmwReg(MDMCFG2, 0x07, (uint8_t)mode);
  _syncMode = mode;
  return true;
}

bool CC1101Radio::setPreambleBytes(uint8_t n) {
  uint8_t best = 0;
  int bestDiff = 1000;
  for (uint8_t i = 0; i < 8; i++) {
    int d = abs((int)PREAMBLE_BYTES[i] - (int)n);
    if (d < bestDiff) {
      bestDiff = d;
      best = i;
    }
  }
  idle();
  rmwReg(MDMCFG1, 0x70, (uint8_t)(best << 4));
  _preambleBytes = PREAMBLE_BYTES[best];
  return true;
}

void CC1101Radio::setPreambleQuality(uint8_t pqt) {
  if (pqt > 7) pqt = 7;
  idle();
  rmwReg(PKTCTRL1, 0xE0, (uint8_t)(pqt << 5));
}

void CC1101Radio::setCrc(bool enabled) {
  idle();
  rmwReg(PKTCTRL0, 0x04, enabled ? 0x04 : 0x00);
}

void CC1101Radio::setCrcAutoflush(bool enabled) {
  idle();
  rmwReg(PKTCTRL1, 0x08, enabled ? 0x08 : 0x00);
}

void CC1101Radio::setWhitening(bool enabled) {
  idle();
  rmwReg(PKTCTRL0, 0x40, enabled ? 0x40 : 0x00);
}

bool CC1101Radio::setManchester(bool enabled) {
  if (enabled && (_mod == MOD_MSK || _mod == MOD_4FSK || _fec)) return false;
  idle();
  rmwReg(MDMCFG2, 0x08, enabled ? 0x08 : 0x00);
  _manchester = enabled;
  return true;
}

void CC1101Radio::setNodeAddress(uint8_t addr) {
  idle();
  writeReg(ADDR, addr);
  _nodeAddr = addr;
}

bool CC1101Radio::setAddressFilter(AddressFilter mode) {
  if ((uint8_t)mode > 3) return false;
  idle();
  rmwReg(PKTCTRL1, 0x03, (uint8_t)mode);
  _adrChk = mode;
  return true;
}

// ---------------------------------------------------------------- length modes

bool CC1101Radio::setVariableLength(uint8_t maxLen) {
  if (maxLen == 0 || maxLen > MAX_PAYLOAD) return false;
  idle();
  if (_fec) {  // FEC needs fixed length
    rmwReg(MDMCFG1, 0x80, 0x00);
    _fec = false;
  }
  writeReg(PKTLEN, maxLen);
  rmwReg(PKTCTRL0, 0x03, LEN_VARIABLE);
  _lenMode = LEN_VARIABLE;
  return true;
}

bool CC1101Radio::setFixedLength(uint8_t n) {
  if (n == 0 || n > 62) return false;
  idle();
  writeReg(PKTLEN, n);
  rmwReg(PKTCTRL0, 0x03, LEN_FIXED);
  _lenMode = LEN_FIXED;
  _fixedLen = n;
  return true;
}

bool CC1101Radio::setInfiniteLength() {
  idle();
  if (_fec) {
    rmwReg(MDMCFG1, 0x80, 0x00);
    _fec = false;
  }
  rmwReg(PKTCTRL0, 0x03, LEN_INFINITE);
  _lenMode = LEN_INFINITE;
  return true;
}

bool CC1101Radio::setFec(bool enabled) {
  if (enabled && _lenMode != LEN_FIXED) return false;
  idle();
  if (enabled && _manchester) {  // FEC and Manchester can't be combined
    rmwReg(MDMCFG2, 0x08, 0x00);
    _manchester = false;
  }
  rmwReg(MDMCFG1, 0x80, enabled ? 0x80 : 0x00);
  _fec = enabled;
  return true;
}

// ---------------------------------------------------------------- output power

const uint8_t* CC1101Radio::paTable() const {
  bool ww = (_inductor == INDUCTOR_WIREWOUND);
  if (_freqMHz < 360.0f) return PA_315;
  if (_freqMHz < 779.0f) return PA_433;
  if (_freqMHz < 900.0f) return ww ? PA_868_WW : PA_868_ML;
  return ww ? PA_915_WW : PA_915_ML;
}

void CC1101Radio::refreshPaFromDbm() {
  _pa = paTable()[paIndexForDbm(_dbm)];
  applyPa();
}

int8_t CC1101Radio::setOutputPowerDbm(int8_t dbm) {
  uint8_t best = paIndexForDbm(dbm);
  _dbm = PA_DBM[best];
  _paIsRaw = false;
  _pa = paTable()[best];
  applyPa();
  return _dbm;
}

void CC1101Radio::setPaRaw(uint8_t paValue) {
  _pa = paValue;
  _paIsRaw = true;
  applyPa();
}

void CC1101Radio::setInductorType(InductorType t) {
  _inductor = t;
  if (!_paIsRaw) refreshPaFromDbm();
}

// FSK/MSK use PATABLE[0]; ASK/OOK switches between PATABLE[0] (off) and PATABLE[1] (on)
// (datasheet 24 / FREND0.PA_POWER).
void CC1101Radio::applyPa() {
  uint8_t table[8] = {0};
  if (_mod == MOD_ASK_OOK) {
    table[1] = _pa;
    writeReg(FREND0, 0x11);
  } else {
    table[0] = _pa;
    writeReg(FREND0, 0x10);
  }
  writeBurst(PATABLE, table, 8);
}

// ---------------------------------------------------------------- state control

bool CC1101Radio::waitMarc(uint8_t state, uint32_t timeoutUs) {
  uint32_t t = micros();
  while (readMarc() != state) {
    if ((micros() - t) > timeoutUs) return false;
  }
  return true;
}

void CC1101Radio::idle() {
  if (_asleep) wake();
  strobe(SIDLE);
  waitMarc(MARC_IDLE, 5000);
}

// SLEEP loses the PATABLE (except entry 0) and FSTEST/PTEST/AGCTEST/TEST2-0 (Table 43), and
// flushes both FIFOs. Keep copies so wake() can put them back.
void CC1101Radio::sleep() {
  idle();
  readBurst(PATABLE, _paSaved, 8);
  _testSaved[0] = readReg(FSTEST);
  _testSaved[1] = readReg(TEST2);
  _testSaved[2] = readReg(TEST1);
  _testSaved[3] = readReg(TEST0);
  strobe(SPWD);  // executes when CS goes high
  _asleep = true;
}

// Pulling CS low starts the crystal and takes the chip to IDLE (datasheet 19.3). Calibration
// values survive SLEEP, so no recalibration is needed unless temperature changed a lot.
void CC1101Radio::wake() {
  if (!_asleep) return;
  _asleep = false;
  spi_device_acquire_bus(_dev, portMAX_DELAY);
  gpio_set_level((gpio_num_t)_cs, 0);
  waitReady(10000);
  gpio_set_level((gpio_num_t)_cs, 1);
  spi_device_release_bus(_dev);

  writeReg(FSTEST, _testSaved[0]);
  writeReg(TEST2, _testSaved[1]);
  writeReg(TEST1, _testSaved[2]);
  writeReg(TEST0, _testSaved[3]);
  writeBurst(PATABLE, _paSaved, 8);
  waitMarc(MARC_IDLE, 5000);
}

// Manual calibration takes ~735 us (datasheet Table 34).
void CC1101Radio::calibrate() {
  idle();
  strobe(SCAL);
  delay(1);
  waitMarc(MARC_IDLE, 3000);
}

bool CC1101Radio::startReceive() {
  idle();
  strobe(SFRX);
  strobe(SRX);
  return true;
}

int16_t CC1101Radio::transmit(const uint8_t* data, uint8_t len, uint32_t timeoutMs) {
  return sendPacket(false, 0, data, len, timeoutMs);
}

int16_t CC1101Radio::transmitTo(uint8_t dest, const uint8_t* data, uint8_t len,
                                uint32_t timeoutMs) {
  return sendPacket(true, dest, data, len, timeoutMs);
}

int16_t CC1101Radio::sendPacket(bool withAddr, uint8_t dest, const uint8_t* data, uint8_t len,
                                uint32_t timeoutMs) {
  if (_lenMode == LEN_INFINITE) return ERR_BAD_ARG;
  bool fixed = (_lenMode == LEN_FIXED);
  uint16_t total = (uint16_t)len + (withAddr ? 1 : 0);
  if (len == 0 || total > (fixed ? 64 : MAX_PAYLOAD)) return ERR_LENGTH;

  idle();
  strobe(SFTX);

  // In fixed mode any size can be sent: PKTLEN is set to this packet's size and put back below.
  bool retarget = fixed && total != _fixedLen;
  if (retarget) writeReg(PKTLEN, (uint8_t)total);

  // TX FIFO burst write. Variable mode: length byte first (counts the address byte). Fixed mode:
  // no length byte. Then the optional address byte, then the payload.
  uint8_t frame[66];
  size_t n = 0;
  frame[n++] = FIFO | BIT_BURST;
  if (!fixed) frame[n++] = (uint8_t)total;
  if (withAddr) frame[n++] = dest;
  memcpy(frame + n, data, len);
  n += len;
  select();
  xfer(frame, nullptr, n);
  deselect();

  bool crc = (readReg(PKTCTRL0) & 0x04) != 0;
  strobe(STX);

  if (timeoutMs == 0) {
    // preamble + sync + (length) + payload + CRC, scaled for Manchester / FEC (2 symbols per bit)
    // and 4-FSK (2 bits per symbol), with 50% margin plus ~1 ms for the PLL calibration.
    uint8_t syncBytes = syncBytesFor(_syncMode);
    uint8_t preamble = syncBytes ? _preambleBytes : 0;
    double symbols = (preamble + syncBytes + (fixed ? 0 : 1) + total + (crc ? 2 : 0)) * 8.0;
    if (_manchester || _fec) symbols *= 2.0;
    if (_mod == MOD_4FSK) symbols *= 0.5;
    timeoutMs = (uint32_t)(symbols * 1000.0 / _bitrateBps * 1.5) + 20;
  }

  // MCSM1 sends us to IDLE after TX. MARCSTATE can still read IDLE for a moment right after
  // STX, so only accept IDLE once we have seen the chip leave it.
  int16_t result = ERR_NONE;
  bool left = false;
  uint32_t start = millis();
  for (;;) {
    uint8_t s = readMarc();
    if (s == MARC_TX_UNDERFLOW) {  // only an SFTX gets us out of this state (datasheet 15.4)
      idle();
      strobe(SFTX);
      result = ERR_TX_UNDERFLOW;
      break;
    }
    if (s != MARC_IDLE) {
      left = true;
    } else if (left) {
      break;
    }
    if (millis() - start > timeoutMs) {
      idle();
      strobe(SFTX);
      result = ERR_TIMEOUT;
      break;
    }
  }
  if (retarget) writeReg(PKTLEN, _fixedLen);
  return result;
}

// ---------------------------------------------------------------- receive

float CC1101Radio::rssiToDbm(uint8_t raw) {
  int16_t v = raw >= 128 ? (int16_t)raw - 256 : (int16_t)raw;
  return v / 2.0f - 74.0f;  // RSSI_offset is 74 dB at 433/868 MHz for all data rates (Table 31)
}

float CC1101Radio::getRSSI() { return rssiToDbm(readStatus(RSSI)); }

int16_t CC1101Radio::available() {
  uint8_t rx = readStatusStable(RXBYTES);
  if (rx & 0x80) return ERR_OVERFLOW;
  return rx & 0x7F;
}

int16_t CC1101Radio::readPacket(uint8_t* buf, uint8_t maxLen, PacketInfo* info) {
  // Read the state BEFORE the byte count. MCSM1 sends the chip to IDLE once a packet has been
  // fully received, so data in the FIFO while still in RX means a packet is only partly in.
  // Reading the count first could leave us with a stale, too-small count if the packet
  // finishes in between, and we would wrongly discard it.
  if (_lenMode == LEN_INFINITE) return ERR_BAD_ARG;  // use readStream()
  uint8_t st = readMarc();
  int16_t n = available();
  if (n == ERR_OVERFLOW) {
    startReceive();
    return ERR_OVERFLOW;
  }
  if (n == 0) return 0;
  if (st == MARC_RX || st == MARC_RX_END || st == MARC_RX_RST) return 0;

  uint8_t len;
  if (_lenMode == LEN_FIXED) {  // no length byte on air: size is whatever we configured
    len = _fixedLen;
    if ((int16_t)len + 2 > n) {
      startReceive();
      return ERR_LENGTH;
    }
  } else {
    len = readReg(FIFO);  // single read pops the length byte
    if (len == 0 || len > MAX_PAYLOAD || (int16_t)len + 2 > n - 1) {
      startReceive();
      return ERR_LENGTH;
    }
  }
  if (len > maxLen) {
    startReceive();
    return ERR_BUF_SMALL;
  }

  readBurst(FIFO, buf, len);
  uint8_t status[2];
  readBurst(FIFO, status, 2);  // RSSI, then LQI | CRC_OK<<7 (PKTCTRL1 append-status)
  startReceive();

  bool crcOk = (status[1] & 0x80) != 0;
  if (info) {
    info->rssi = rssiToDbm(status[0]);
    info->lqi = status[1] & 0x7F;
    info->crcOk = crcOk;
    info->hasAddress = (_adrChk != ADDR_OFF) && len >= 1;
    info->address = info->hasAddress ? buf[0] : 0;
    info->broadcast = false;
    if (info->hasAddress && info->address != _nodeAddr) {
      if (_adrChk == ADDR_NODE_BC0) info->broadcast = (info->address == 0x00);
      else if (_adrChk == ADDR_NODE_BC0_BCFF)
        info->broadcast = (info->address == 0x00 || info->address == 0xFF);
    }
  }
  if (!crcOk && (readReg(PKTCTRL0) & 0x04)) return ERR_CRC;
  return len;
}

int16_t CC1101Radio::readStream(uint8_t* buf, uint8_t maxLen) {
  if (_lenMode != LEN_INFINITE) return ERR_BAD_ARG;
  int16_t n = available();
  if (n == ERR_OVERFLOW) {
    startReceive();
    return ERR_OVERFLOW;
  }
  if (n <= 1) return 0;  // keep the last byte in the FIFO while the "packet" is still arriving
  uint8_t take = (uint8_t)(n - 1 > maxLen ? maxLen : n - 1);
  readBurst(FIFO, buf, take);
  return take;
}

uint8_t CC1101Radio::packetStatus() { return readStatusStable(PKTSTATUS); }
bool CC1101Radio::carrierSense() { return (packetStatus() & 0x40) != 0; }

// FREQEST is a signed offset in units of f_xosc / 2^14 (about 1.59 kHz).
float CC1101Radio::frequencyErrorKHz() {
  int8_t est = (int8_t)readStatus(FREQEST);
  return (float)(est * (XOSC_HZ / 16384.0) / 1000.0);
}

// ---------------------------------------------------------------- info

uint8_t CC1101Radio::partNumber() { return readStatus(PARTNUM); }
uint8_t CC1101Radio::version() { return readStatus(VERSION); }
uint8_t CC1101Radio::marcState() { return readMarc(); }
