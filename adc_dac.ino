/*
 * Ball Balancing Table - Rev 1 Analog Interface Board
 * Bring-up and test firmware, no calibration applied
 *
 * Board: Seeed XIAO ESP32-C6
 * DAC:   TI DAC8568IAPW  (8ch, 16-bit, internal 2.5V ref, gain 1)
 * ADC:   TI ADS8688IDBT  (8ch, 16-bit, internal 4.096V ref)
 *
 * Output chain: DAC 0-2.5V -> op-amp gain 9.66 -> 0-24.15V at J5
 * Input chain:  0-24V at J6 -> divider -> 0-9.6V -> ADC
 *
 * Open the serial monitor at 115200 baud, line ending Newline.
 * Type h for the command list.
 */

#include <SPI.h>
#include "adc_dac.h"

// ---------------------------------------------------------------- pin map
const int PIN_DAC_CS    = D0;   // DAC SYNC,  active low
const int PIN_DAC_LDAC  = D1;   // DAC LDAC,  active low, through R54
const int PIN_ADC_CS    = D2;   // ADC CS,    active low
const int PIN_ADC_RST   = D3;   // ADC RST,   active low
const int PIN_DAC_CLR   = D4;   // DAC CLR,   active low
const int PIN_SCK       = D8;
const int PIN_MISO      = D9;
const int PIN_MOSI      = D10;

// ------------------------------------------------------------- board math
const float DAC_VREF    = 2.5f;      // internal reference, A grade gain 1
const float OPAMP_GAIN  = 9.66f;     // 1 + 86.6k/10k
const float OUT_FS      = DAC_VREF * OPAMP_GAIN;   // 24.15 V

const float DIV_RATIO   = 2.5f;      // 24V in -> 9.6V at the ADC pin

// Full-scale range at the ADC pin. Changed at runtime by the 'G' command so
// the scaling always matches whatever range register is actually programmed.
float ADC_FS = 10.24f;

// SPI: both parts clock data on the falling edge -> mode 1
SPISettings spiDAC(10000000, MSBFIRST, SPI_MODE1);
SPISettings spiADC(10000000, MSBFIRST, SPI_MODE1);

// per-channel correction, all unity until you measure
float outGain[8]   = {1,1,1,1,1,1,1,1};
float outOffset[8] = {0,0,0,0,0,0,0,0};
float inGain[8]    = {1,1,1,1,1,1,1,1};
float inOffset[8]  = {0,0,0,0,0,0,0,0};

// =========================================================== DAC8568
//
// 32-bit frame, MSB first:
//   [31]      don't care
//   [30:28]   prefix
//   [27:24]   control
//   [23:20]   address
//   [19:4]    16-bit data
//   [3:0]     feature
//
void dacWrite32(uint32_t word) {
  SPI.beginTransaction(spiDAC);
  digitalWrite(PIN_DAC_CS, LOW);
  SPI.transfer((word >> 24) & 0xFF);
  SPI.transfer((word >> 16) & 0xFF);
  SPI.transfer((word >>  8) & 0xFF);
  SPI.transfer( word        & 0xFF);
  digitalWrite(PIN_DAC_CS, HIGH);
  SPI.endTransaction();
}

// load one channel's input register, no output change yet
void dacLoadChannel(uint8_t ch, uint16_t code) {
  uint32_t w = (0x0UL << 24)                        // control 0000 = write input reg n
             | ((uint32_t)(ch & 0x0F) << 20)
             | ((uint32_t)code << 4);
  dacWrite32(w);
}

// transfer every loaded input register to the outputs at once
void dacLatch() {
  digitalWrite(PIN_DAC_LDAC, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_DAC_LDAC, HIGH);
}

void dacSetCode(uint8_t ch, uint16_t code) {
  dacLoadChannel(ch, code);
  dacLatch();
}

uint16_t voltsToCode(uint8_t ch, float volts) {
  float corrected = (volts - outOffset[ch]) / outGain[ch];
  float frac = corrected / OUT_FS;
  if (frac < 0) frac = 0;
  if (frac > 1) frac = 1;
  return (uint16_t)(frac * 65535.0f + 0.5f);
}

void dacSetVolts(uint8_t ch, float volts) {
  dacSetCode(ch, voltsToCode(ch, volts));
}

void dacInit() {
  dacWrite32(0x07000000UL);                 // software reset
  delay(10);

  for (uint8_t ch = 0; ch < 8; ch++) dacLoadChannel(ch, 0);
  dacLatch();                               // zero BEFORE the reference comes up

  dacWrite32(0x08000001UL);                 // enable internal 2.5V ref, static mode
  delay(10);

  for (uint8_t ch = 0; ch < 8; ch++) dacLoadChannel(ch, 0);
  dacLatch();                               // zero again now the reference is live
}

// =========================================================== ADS8688
//
// Command frame is 16 bits out, then 16 bits of result back.
// The result belongs to the channel selected in the PREVIOUS frame,
// so every read is issued twice and the second answer is the real one.
//
// SDO timing: the device puts its MSB on the line at the 16th SCLK falling
// edge and a new bit on every falling edge after. In SPI mode 1 the master
// samples on falling edges, so the MSB lands in the LAST bit of the first
// 16-bit transfer and the rest arrive shifted left by one in the second.
//
uint16_t adcFrame(uint16_t cmd) {
  SPI.beginTransaction(spiADC);
  digitalWrite(PIN_ADC_CS, LOW);
  uint16_t hi = SPI.transfer16(cmd);   // SCLK 1-16, its LSB = data bit 15
  uint16_t lo = SPI.transfer16(0);     // SCLK 17-32, bits 14..0 shifted left
  digitalWrite(PIN_ADC_CS, HIGH);
  SPI.endTransaction();
  return ((hi & 0x0001) << 15) | (lo >> 1);
}

// program-register write: [15:9] address, [8] = 1 for write, [7:0] data
void adcWriteReg(uint8_t addr, uint8_t data) {
  uint16_t cmd = ((uint16_t)(addr & 0x7F) << 9) | (1 << 8) | data;
  adcFrame(cmd);
}

// program-register read: [15:9] address, [8] = 0 for read
// same one-bit shift applies to the returned byte
uint8_t adcReadReg(uint8_t addr) {
  SPI.beginTransaction(spiADC);
  digitalWrite(PIN_ADC_CS, LOW);
  uint16_t hi = SPI.transfer16((uint16_t)(addr & 0x7F) << 9);
  uint16_t lo = SPI.transfer16(0);
  digitalWrite(PIN_ADC_CS, HIGH);
  SPI.endTransaction();
  return ((hi & 0x0001) << 7) | ((lo >> 9) & 0x7F);
}

uint16_t adcReadRaw(uint8_t ch) {
  uint16_t cmd = 0xC000 | ((uint16_t)(ch & 0x07) << 10);  // MAN_Ch_n
  adcFrame(cmd);              // first frame selects the channel
  return adcFrame(cmd);       // second frame returns its result
}

float adcReadVolts(uint8_t ch) {
  uint16_t raw = adcReadRaw(ch);
  float vAdc = ((float)raw / 65536.0f) * ADC_FS;   // at the ADC pin
  float vIn  = vAdc * DIV_RATIO;                   // back at the connector
  return (vIn - inOffset[ch]) / inGain[ch];
}

void adcDumpRanges() {
  Serial.println(F("\n--- range registers ---"));
  for (uint8_t ch = 0; ch < 8; ch++) {
    uint8_t v = adcReadReg(0x05 + ch);
    Serial.printf("  IN %d  (reg 0x%02X) = 0x%02X\n", ch + 1, 0x05 + ch, v);
  }
  Serial.printf("firmware is scaling for %.2f V full scale at the ADC pin\n", ADC_FS);
}

void adcInit() {
  digitalWrite(PIN_ADC_RST, LOW);
  delay(1);
  digitalWrite(PIN_ADC_RST, HIGH);
  delay(20);                       // internal reference settling

  // range registers live at 0x05 (ch0) through 0x0C (ch7)
  // 0x05 = 0 to 10.24 V unipolar -- verify with 'g'
  for (uint8_t ch = 0; ch < 8; ch++) adcWriteReg(0x05 + ch, 0x05);
  delay(5);
  ADC_FS = 10.24f;
}

// =========================================================== tests

void testOutputSweep(uint8_t ch) {
  const float steps[] = {0, 6, 12, 18, 24};
  Serial.printf("\n--- OUT %d sweep ---\n", ch + 1);
  Serial.println(F("commanded   code     <- measure the pin now"));
  for (float v : steps) {
    uint16_t code = voltsToCode(ch, v);
    dacSetCode(ch, code);
    Serial.printf("  %6.2f V   %5u\n", v, code);
    delay(2500);          // time to read the meter
  }
  dacSetVolts(ch, 0);
  Serial.println(F("back to 0 V"));
}

void testInputRead(uint8_t ch) {
  Serial.printf("\n--- IN %d, reading every second, send any key to stop ---\n", ch + 1);
  while (!Serial.available()) {
    uint16_t raw = adcReadRaw(ch);
    float vAdc = ((float)raw / 65536.0f) * ADC_FS;
    Serial.printf("  raw %5u   at ADC pin %6.3f V   at connector %6.3f V\n",
                  raw, vAdc, vAdc * DIV_RATIO);
    delay(1000);
  }
  while (Serial.available()) Serial.read();
}

void testAllInputsOnce() {
  Serial.println(F("\n--- all inputs ---"));
  for (uint8_t ch = 0; ch < 8; ch++) {
    uint16_t raw = adcReadRaw(ch);
    float vAdc = ((float)raw / 65536.0f) * ADC_FS;
    Serial.printf("  IN %d: raw %5u   %6.3f V at ADC pin   %6.3f V at connector\n",
                  ch + 1, raw, vAdc, vAdc * DIV_RATIO);
  }
}

void testLoopback(uint8_t outCh, uint8_t inCh) {
  const float steps[] = {0, 6, 12, 18, 24};
  Serial.printf("\n--- loopback: jumper OUT %d to IN %d ---\n", outCh + 1, inCh + 1);
  Serial.println(F("commanded   readback    error"));
  for (float v : steps) {
    dacSetVolts(outCh, v);
    delay(100);
    float r = adcReadVolts(inCh);
    Serial.printf("  %6.2f V   %6.3f V   %+6.3f V\n", v, r, r - v);
  }
  dacSetVolts(outCh, 0);
}

void testCrosstalk(uint8_t drivenCh) {
  Serial.printf("\n--- drive OUT %d to 24 V, read every input ---\n", drivenCh + 1);
  dacSetVolts(drivenCh, 24.0f);
  delay(200);
  testAllInputsOnce();
  dacSetVolts(drivenCh, 0);
}

// =========================================================== shell

void help() {
  Serial.println(F("\n================ commands ================"));
  Serial.println(F("  h            this list"));
  Serial.println(F("  i            re-init DAC and ADC"));
  Serial.println(F("  z            all outputs to 0 V"));
  Serial.println(F("  o <ch> <V>   set one output, ch 1-8"));
  Serial.println(F("  s <ch>       sweep one output 0-24 V"));
  Serial.println(F("  S            sweep all eight outputs"));
  Serial.println(F("  r <ch>       read one input continuously"));
  Serial.println(F("  R            read all eight inputs once"));
  Serial.println(F("  l <out> <in> loopback test"));
  Serial.println(F("  x <ch>       crosstalk check"));
  Serial.println(F("  --- ADC range diagnostics ---"));
  Serial.println(F("  g            dump all eight range registers"));
  Serial.println(F("  G <ch> <val> write one range register, val 0-15"));
  Serial.println(F("  F <volts>    tell firmware the ADC pin full scale"));
  Serial.println(F("=========================================="));
}

void adcdac_setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(PIN_DAC_CS,   OUTPUT);  digitalWrite(PIN_DAC_CS,   HIGH);
  pinMode(PIN_DAC_LDAC, OUTPUT);  digitalWrite(PIN_DAC_LDAC, HIGH);
  pinMode(PIN_DAC_CLR,  OUTPUT);  digitalWrite(PIN_DAC_CLR,  HIGH);
  pinMode(PIN_ADC_CS,   OUTPUT);  digitalWrite(PIN_ADC_CS,   HIGH);
  pinMode(PIN_ADC_RST,  OUTPUT);  digitalWrite(PIN_ADC_RST,  HIGH);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);

  // Serial.println(F("\nRev 1 board test firmware, uncalibrated"));
  dacInit();
  // Serial.println(F("DAC init sent, all channels commanded to 0 V"));
  adcInit();
  // Serial.println(F("ADC init sent, range registers written"));
  adcDumpRanges();
//  help();
}

void adcdac_loop() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  char c = line.charAt(0);
  String rest = line.substring(1);
  rest.trim();

  int a = -1, b = -1;
  float fv = 0;
  int sp = rest.indexOf(' ');
  if (sp > 0) {
    a  = rest.substring(0, sp).toInt();
    b  = rest.substring(sp + 1).toInt();
    fv = rest.substring(sp + 1).toFloat();
  } else if (rest.length()) {
    a  = rest.toInt();
    fv = rest.toFloat();
  }

  switch (c) {
    case 'h': help(); break;

    case 'i': dacInit(); adcInit(); adcDumpRanges(); break;

    case 'z':
      for (uint8_t ch = 0; ch < 8; ch++) dacLoadChannel(ch, 0);
      dacLatch();
      Serial.println(F("all outputs 0 V"));
      break;

    case 'o':
      if (a >= 1 && a <= 8) {
        dacSetVolts(a - 1, fv);
        Serial.printf("OUT %d set to %.2f V\n", a, fv);
      } else Serial.println(F("usage: o <1-8> <volts>"));
      break;

    case 's':
      if (a >= 1 && a <= 8) testOutputSweep(a - 1);
      else Serial.println(F("usage: s <1-8>"));
      break;

    case 'S':
      for (uint8_t ch = 0; ch < 8; ch++) testOutputSweep(ch);
      break;

    case 'r':
      if (a >= 1 && a <= 8) testInputRead(a - 1);
      else Serial.println(F("usage: r <1-8>"));
      break;

    case 'R': testAllInputsOnce(); break;

    case 'l':
      if (a >= 1 && a <= 8 && b >= 1 && b <= 8) testLoopback(a - 1, b - 1);
      else Serial.println(F("usage: l <out 1-8> <in 1-8>"));
      break;

    case 'x':
      if (a >= 1 && a <= 8) testCrosstalk(a - 1);
      else Serial.println(F("usage: x <1-8>"));
      break;

    case 'g': adcDumpRanges(); break;

    case 'G':
      if (a >= 1 && a <= 8 && b >= 0 && b <= 15) {
        adcWriteReg(0x05 + (a - 1), (uint8_t)b);
        delay(2);
        Serial.printf("IN %d range reg written 0x%02X, reads back 0x%02X\n",
                      a, b, adcReadReg(0x05 + (a - 1)));
      } else Serial.println(F("usage: G <ch 1-8> <value 0-15>"));
      break;

    case 'F':
      if (fv > 0.1f && fv < 25.0f) {
        ADC_FS = fv;
        Serial.printf("ADC pin full scale now %.3f V\n", ADC_FS);
      } else Serial.println(F("usage: F <volts>   e.g. F 5.12"));
      break;

    default: Serial.println(F("unknown, type h"));
  }
}
