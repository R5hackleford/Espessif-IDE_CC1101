// CC1101Radio - flexible driver for the TI CC1101 sub-GHz transceiver.
//
// Two layers:
//   * High level: begin(), setFrequency(), setBitrate(), startReceive(), readPacket(), ...
//   * Raw access: readReg()/writeReg()/strobe()/readBurst()/writeBurst() plus the full
//     register map in the CC1101Reg namespace, for anything the helpers don't cover.
//
// ESP-IDF component version (native spi_master driver). Several radios can share one SPI bus:
// create one CC1101Radio per chip with its own CS pin.
// Supported bands: 300-348, 387-464 and 779-928 MHz (the CC1101 cannot do 2.4 GHz).
//
// Reviewed against TI datasheet SWRS061I. Section numbers in comments refer to that document.
//
// Notes on configuration:
//   * Every configuration setter drops the chip to IDLE (datasheet 15 / 21: packet handling,
//     frequency and modem fields must only be changed in IDLE). Call startReceive() again
//     afterwards.
//   * The bitrate you set is the BAUD (symbol) rate. With Manchester on, the data rate in kbps
//     is half of it; with 4-FSK it is twice it (datasheet Table 3).
#pragma once

#include <stdint.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"

namespace CC1101Reg {
// Configuration registers
enum : uint8_t {
  IOCFG2 = 0x00, IOCFG1 = 0x01, IOCFG0 = 0x02, FIFOTHR = 0x03,
  SYNC1 = 0x04, SYNC0 = 0x05, PKTLEN = 0x06, PKTCTRL1 = 0x07, PKTCTRL0 = 0x08,
  ADDR = 0x09, CHANNR = 0x0A, FSCTRL1 = 0x0B, FSCTRL0 = 0x0C,
  FREQ2 = 0x0D, FREQ1 = 0x0E, FREQ0 = 0x0F,
  MDMCFG4 = 0x10, MDMCFG3 = 0x11, MDMCFG2 = 0x12, MDMCFG1 = 0x13, MDMCFG0 = 0x14,
  DEVIATN = 0x15, MCSM2 = 0x16, MCSM1 = 0x17, MCSM0 = 0x18,
  FOCCFG = 0x19, BSCFG = 0x1A, AGCCTRL2 = 0x1B, AGCCTRL1 = 0x1C, AGCCTRL0 = 0x1D,
  WOREVT1 = 0x1E, WOREVT0 = 0x1F, WORCTRL = 0x20,
  FREND1 = 0x21, FREND0 = 0x22, FSCAL3 = 0x23, FSCAL2 = 0x24, FSCAL1 = 0x25, FSCAL0 = 0x26,
  RCCTRL1 = 0x27, RCCTRL0 = 0x28, FSTEST = 0x29, PTEST = 0x2A, AGCTEST = 0x2B,
  TEST2 = 0x2C, TEST1 = 0x2D, TEST0 = 0x2E,
  PATABLE = 0x3E, FIFO = 0x3F
};
// Command strobes
enum : uint8_t {
  SRES = 0x30, SFSTXON = 0x31, SXOFF = 0x32, SCAL = 0x33, SRX = 0x34, STX = 0x35,
  SIDLE = 0x36, SWOR = 0x38, SPWD = 0x39, SFRX = 0x3A, SFTX = 0x3B, SWORRST = 0x3C, SNOP = 0x3D
};
// Status registers (read with the burst bit set)
enum : uint8_t {
  PARTNUM = 0x30, VERSION = 0x31, FREQEST = 0x32, LQI = 0x33, RSSI = 0x34,
  MARCSTATE = 0x35, WORTIME1 = 0x36, WORTIME0 = 0x37, PKTSTATUS = 0x38,
  VCO_VC_DAC = 0x39, TXBYTES = 0x3A, RXBYTES = 0x3B,
  RCCTRL1_STATUS = 0x3C, RCCTRL0_STATUS = 0x3D
};
// MARCSTATE values (datasheet 29.3)
enum : uint8_t {
  MARC_SLEEP = 0x00, MARC_IDLE = 0x01, MARC_RX = 0x0D, MARC_RX_END = 0x0E, MARC_RX_RST = 0x0F,
  MARC_RX_OVERFLOW = 0x11, MARC_FSTXON = 0x12, MARC_TX = 0x13, MARC_TX_END = 0x14,
  MARC_TX_UNDERFLOW = 0x16
};
}  // namespace CC1101Reg

class CC1101Radio {
 public:
  enum Modulation : uint8_t {
    MOD_2FSK = 0, MOD_GFSK = 1, MOD_ASK_OOK = 3, MOD_4FSK = 4, MOD_MSK = 7
  };

  // MDMCFG2.SYNC_MODE (datasheet Table 30). The "_CS" variants also require carrier sense.
  enum SyncMode : uint8_t {
    SYNC_NONE = 0,       // no preamble/sync sent in TX, none detected in RX
    SYNC_15_16 = 1,      // 15 of 16 sync bits must match
    SYNC_16_16 = 2,      // default
    SYNC_30_32 = 3,      // 32-bit sync word (SYNC1:SYNC0 sent twice), 30 of 32 bits match
    SYNC_CS_ONLY = 4,    // no preamble/sync, carrier sense above threshold
    SYNC_15_16_CS = 5,
    SYNC_16_16_CS = 6,
    SYNC_30_32_CS = 7
  };

  // PKTCTRL1.ADR_CHK - hardware address filtering on received packets (datasheet 15.3.1).
  enum AddressFilter : uint8_t {
    ADDR_OFF = 0,           // no address check
    ADDR_NODE = 1,          // accept only our node address
    ADDR_NODE_BC0 = 2,      // node address + 0x00 broadcast
    ADDR_NODE_BC0_BCFF = 3  // node address + 0x00 and 0xFF broadcast
  };

  // PKTCTRL0.LENGTH_CONFIG. Variable (default): a length byte is sent first. Fixed: no length byte
  // on air, both ends agree on the size. Infinite: no length at all, bytes stream through the FIFO.
  enum LengthMode : uint8_t { LEN_VARIABLE = 1, LEN_FIXED = 0, LEN_INFINITE = 2 };

  // Which PATABLE set to use (datasheet Tables 37 and 39). Only 868/915 MHz differ.
  enum InductorType : uint8_t { INDUCTOR_MULTILAYER = 0, INDUCTOR_WIREWOUND = 1 };

  // Return codes (negative = error). readPacket() returns the payload length when >= 0.
  static constexpr int16_t ERR_NONE = 0;
  static constexpr int16_t ERR_CHIP_NOT_FOUND = -1;
  static constexpr int16_t ERR_BAD_ARG = -2;
  static constexpr int16_t ERR_TIMEOUT = -3;
  static constexpr int16_t ERR_OVERFLOW = -4;
  static constexpr int16_t ERR_CRC = -5;
  static constexpr int16_t ERR_BUF_SMALL = -6;
  static constexpr int16_t ERR_LENGTH = -7;
  static constexpr int16_t ERR_TX_UNDERFLOW = -8;
  static constexpr int16_t ERR_SPI = -9;  // could not add the device to the SPI bus

  // 64-byte FIFO minus length byte and 2 appended status bytes. When address filtering is used
  // the address byte counts as part of this payload.
  static constexpr uint8_t MAX_PAYLOAD = 61;

  struct PacketInfo {
    float rssi;        // dBm
    uint8_t lqi;
    bool crcOk;
    bool hasAddress;   // true when address filtering is on (buf[0] is the destination address)
    uint8_t address;   // destination address the packet was sent to (valid if hasAddress)
    bool broadcast;    // true when the packet matched a broadcast address instead of ours
  };

  // host: the SPI host you passed to spi_bus_initialize() (e.g. SPI2_HOST). csPin is driven by
  // this class (not by the SPI peripheral). misoPin lets the driver wait for the chip-ready
  // signal after CS goes low: pass the same GPIO you gave the bus as miso_io_num (-1 skips the
  // wait). spiHz is clamped to 6.5 MHz, the datasheet limit for burst access (Table 22).
  CC1101Radio(spi_host_device_t host, int csPin, int misoPin = -1, uint32_t spiHz = 4000000);
  ~CC1101Radio();
  CC1101Radio(const CC1101Radio&) = delete;
  CC1101Radio& operator=(const CC1101Radio&) = delete;

  // Call spi_bus_initialize() first. Adds the device to the bus, resets the chip, loads sane defaults and applies the
  // settings below. Returns ERR_NONE on success, ERR_BAD_ARG if the modulation / bitrate /
  // deviation / bandwidth combination is outside what the datasheet allows.
  int16_t begin(float freqMHz = 433.92f, float bitrateKbps = 4.8f, float devKHz = 47.6f,
                float rxBwKHz = 135.0f, Modulation mod = MOD_GFSK);
  void reset();

  // ---- Configuration (each returns false if the argument is out of range / unsupported) ----
  bool setFrequency(float mhz);
  bool setBitrate(float kbps);          // baud rate; limits depend on modulation (see below)
  bool setDeviation(float kHz);         // ~1.59 .. 381 kHz (2-FSK / GFSK / 4-FSK only)
  bool setRxBandwidth(float kHz);       // 58 .. 812 kHz
  // Does not check the bitrate. Valid baud rates (Table 3): 2-FSK 0.6-500, GFSK/ASK/OOK 0.6-250,
  // 4-FSK 0.6-300, MSK 26-500. Switching to MSK or 4-FSK automatically turns Manchester off.
  // For ASK/OOK the frequency offset compensation is disabled (datasheet 14.1 / FOCCFG).
  // Returns false only for an invalid modulation value.
  bool setModulation(Modulation m);
  static bool bitrateSupported(Modulation m, float kbps);
  // ASK/OOK receive settings from TI DN022 (SWRA215E):
  //   AGCCTRL2 = 0x03..0x07, AGCCTRL1 = 0x00, AGCCTRL0 = 0x91 or 0x92, and FREND1 = 0xB6 when the
  //   RX bandwidth is above ~101 kHz (0x56 otherwise).
  // setModulation(MOD_ASK_OOK) applies 0x04/0x00/0x92 (the best case in DN022's example, at
  // 3.8 kBd / 100 kHz) and restores the FSK values when you leave OOK. DN022 stresses that the
  // optimum changes with data rate and bandwidth and must be found by measuring PER over the whole
  // input range, so use this to tune: magnTarget is the whole AGCCTRL2 byte (0..7), agcCtrl0 the
  // raw AGCCTRL0 (0x91 = 8 dB decision boundary, 0x92 = 12 dB; with 8 dB the "low" bit must be
  // >= 16 dB below the "high" bit). Only valid while in ASK/OOK (returns false otherwise).
  // DN022 also says FSCTRL1 (IF frequency) should follow the RX bandwidth, using the SmartRF
  // value for the preset at or above it (58/100/232/325/540/812 kHz); that table isn't in the
  // note, so set it with writeReg(FSCTRL1, ...) from SmartRF Studio.
  bool setOokAgc(uint8_t magnTarget = 4, uint8_t agcCtrl0 = 0x92);
  // MSK only: DEVIATN.DEVIATION_M sets the fraction of the symbol period used for the phase
  // change (0 is the SmartRF value for 500 kBd). For rates below 500 kBd TI recommends 2-FSK /
  // GFSK with deviation = baud/4 instead of MSK. Returns false if not in MSK or m > 7.
  bool setMskPhaseTransition(uint8_t m);
  bool setChannelSpacing(float kHz);    // ~25.4 .. 405 kHz
  void setChannel(uint8_t channel);

  // ---- Packet handling ----
  void setSyncWord(uint8_t hi, uint8_t lo);
  bool setSyncMode(SyncMode mode);
  bool setPreambleBytes(uint8_t n);     // snaps to 2,3,4,6,8,12,16,24 (4 recommended, 8 at 500 kBd)
  void setPreambleQuality(uint8_t pqt); // PQT 0..7; 0 = sync word always accepted
  void setCrc(bool enabled);
  void setCrcAutoflush(bool enabled);   // flush RX FIFO on bad CRC (read each packet before the next)
  void setWhitening(bool enabled);

  // Manchester encoding/decoding (MDMCFG2.MANCHESTER_EN). Halves the data rate. Not supported
  // together with MSK or 4-FSK (returns false if the current modulation is one of those).
  // Both ends must use the same setting.
  bool setManchester(bool enabled);
  bool manchester() const { return _manchester; }

  // Hardware address filtering (receive side). With filtering on, the first payload byte after
  // the length byte is the destination address; the chip drops packets that don't match, and
  // restarts RX by itself. Use transmitTo() or put the address in data[0] when transmitting.
  void setNodeAddress(uint8_t addr);
  bool setAddressFilter(AddressFilter mode);
  AddressFilter addressFilter() const { return _adrChk; }
  uint8_t nodeAddress() const { return _nodeAddr; }

  // ---- Packet length modes (for talking to devices with their own framing) ----
  // Variable (default): [len][payload]. transmit()/readPacket() as before; maxLen is the largest
  // length byte accepted on RX (1..61).
  bool setVariableLength(uint8_t maxLen = MAX_PAYLOAD);
  // Fixed: no length byte on air. RX accepts exactly n bytes (1..62: the FIFO also holds the 2
  // appended status bytes). transmit()/transmitTo() may send ANY size 1..64 per call; PKTLEN is
  // switched for that packet and put back afterwards, so RX settings are unchanged. With address
  // filtering the address byte counts toward the size.
  bool setFixedLength(uint8_t n);
  // Infinite: the radio keeps receiving after the sync word until you call idle(). Use
  // startReceive() then readStream() in a loop. Transmit is not available in this mode (switch to
  // fixed mode to send raw bytes). CRC and address filtering do not apply to a stream.
  bool setInfiniteLength();
  LengthMode lengthMode() const { return _lenMode; }
  uint8_t fixedLength() const { return _fixedLen; }
  // Convolutional FEC + interleaving (datasheet 17). Needs fixed-length mode (returns false
  // otherwise) and is not allowed with Manchester, so enabling it turns Manchester off. Halves the
  // data rate for a given baud rate. Leaving fixed mode turns it off again.
  bool setFec(bool enabled);
  bool fec() const { return _fec; }

  // ---- Output power ----
  int8_t setOutputPowerDbm(int8_t dbm); // snaps to nearest PA step, returns the dBm chosen
  void setPaRaw(uint8_t paValue);       // raw PATABLE byte (datasheet: 0x61-0x6F is not allowed)
  void setInductorType(InductorType t); // PATABLE set for 868/915 MHz boards (default multilayer)

  float frequency() const { return _freqMHz; }
  float bitrate() const { return _bitrateBps / 1000.0f; }

  // ---- State control ----
  void idle();
  void sleep();      // lowest power; PATABLE and test registers are saved and restored by wake()
  void wake();       // CS low, wait for the crystal, restore what SLEEP loses. Leaves chip in IDLE
  void calibrate();
  bool startReceive();  // IDLE -> flush RX FIFO -> RX
  // Both leave the chip in IDLE. Return ERR_NONE, ERR_LENGTH, ERR_TIMEOUT or ERR_TX_UNDERFLOW.
  int16_t transmit(const uint8_t* data, uint8_t len, uint32_t timeoutMs = 0);
  // Sends [dest][data...] so that receivers with address filtering pick it up.
  // len + 1 must be <= MAX_PAYLOAD.
  int16_t transmitTo(uint8_t dest, const uint8_t* data, uint8_t len, uint32_t timeoutMs = 0);

  // ---- Receive ----
  float getRSSI();                  // live RSSI in dBm (chip must be in RX)
  int16_t available();              // bytes in RX FIFO, or ERR_OVERFLOW
  // Returns payload length (>0), 0 if no complete packet yet, or a negative error. Restarts RX.
  // With address filtering on, buf[0] is the destination address and is included in the length.
  int16_t readPacket(uint8_t* buf, uint8_t maxLen, PacketInfo* info = nullptr);
  // Infinite mode only. Copies up to maxLen bytes that have arrived so far and returns the count
  // (0 if none), or ERR_OVERFLOW (RX restarted, bytes were lost: poll faster). Always leaves one
  // byte in the FIFO, as the datasheet requires while a packet is still being received.
  int16_t readStream(uint8_t* buf, uint8_t maxLen);
  uint8_t packetStatus();           // raw PKTSTATUS register
  bool carrierSense();              // PKTSTATUS.CS
  float frequencyErrorKHz();        // FREQEST converted to kHz (FSK/MSK only; 0 for ASK/OOK)

  // ---- Info ----
  uint8_t partNumber();
  uint8_t version();
  uint8_t marcState();

  // ---- Raw access ----
  uint8_t readReg(uint8_t reg);
  void writeReg(uint8_t reg, uint8_t value);
  uint8_t readStatus(uint8_t reg);
  uint8_t strobe(uint8_t cmd);
  void readBurst(uint8_t reg, uint8_t* buf, uint8_t len);
  void writeBurst(uint8_t reg, const uint8_t* buf, uint8_t len);

 private:
  bool attach();
  void select();
  void deselect();
  void xfer(const uint8_t* tx, uint8_t* rx, size_t n);
  void waitReady(uint32_t timeoutUs);
  uint8_t readStatusStable(uint8_t reg);
  uint8_t readMarc();
  bool waitMarc(uint8_t state, uint32_t timeoutUs);
  void applyPa();
  void refreshPaFromDbm();
  const uint8_t* paTable() const;
  void updateRxAnalogForBw(float actualBwKHz);
  void rmwReg(uint8_t reg, uint8_t mask, uint8_t value);
  int16_t sendPacket(bool withAddr, uint8_t dest, const uint8_t* data, uint8_t len,
                     uint32_t timeoutMs);
  static float rssiToDbm(uint8_t raw);

  spi_host_device_t _host;
  int _cs;
  int _miso;
  uint32_t _spiHz;
  spi_device_handle_t _dev = nullptr;

  float _freqMHz = 433.92f;
  double _bitrateBps = 4800.0;
  Modulation _mod = MOD_GFSK;
  uint8_t _pa = 0x60;
  int8_t _dbm = 0;
  bool _paIsRaw = false;
  InductorType _inductor = INDUCTOR_MULTILAYER;

  bool _manchester = false;
  AddressFilter _adrChk = ADDR_OFF;
  uint8_t _nodeAddr = 0;
  uint8_t _preambleBytes = 4;
  SyncMode _syncMode = SYNC_16_16;
  uint8_t _focLimitFsk = 2;   // FOCCFG.FOC_LIMIT to restore when leaving ASK/OOK
  uint8_t _agcFsk[3] = {0x43, 0x40, 0x91};  // AGCCTRL2/1/0 to restore when leaving ASK/OOK
  uint8_t _frend1Fsk = 0x56;                // FREND1 to restore when leaving ASK/OOK
  float _rxBwKHz = 135.0f;
  LengthMode _lenMode = LEN_VARIABLE;
  uint8_t _fixedLen = 0;
  bool _fec = false;

  bool _asleep = false;
  uint8_t _paSaved[8] = {0};
  uint8_t _testSaved[4] = {0};  // FSTEST, TEST2, TEST1, TEST0
};
