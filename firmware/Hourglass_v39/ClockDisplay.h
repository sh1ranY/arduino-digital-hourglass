#ifndef ELEC281_AUX_CLOCK_DISPLAY_H
#define ELEC281_AUX_CLOCK_DISPLAY_H

#include <Arduino.h>

// A separate MAX7219 bus keeps all previously verified hourglass pins intact.
// The four-display board is read with its input header on the right.  These raw
// transforms are provisional until the physical board's calibration is checked.
namespace AuxClock {
const uint8_t DIN_PIN = 7;
const uint8_t CLK_PIN = 8;
const uint8_t CS_PIN = 9;
const uint8_t MODULES = 4;
const uint8_t CHAIN_INDEX_FOR_TILE[MODULES] = {3, 2, 1, 0};
const uint8_t RAW_ROTATION[MODULES] = {0, 0, 0, 0};  // Clockwise quarter turns.
const bool RAW_MIRROR_X[MODULES] = {false, false, false, false};
const bool RAW_MIRROR_Y[MODULES] = {false, false, false, false};

static uint32_t logicalRows[8];  // Bit 31 is the leftmost logical LED.
static uint8_t rawRows[MODULES][8];  // Chain index 0 is nearest the Arduino.
static uint16_t lastSeconds = 65535U;
static char lastMode = '\0';

inline void sendByte(uint8_t value) {
  for (uint8_t mask = 0x80; mask; mask >>= 1) {
    digitalWrite(CLK_PIN, LOW);
    digitalWrite(DIN_PIN, (value & mask) ? HIGH : LOW);
    delayMicroseconds(1);  // Explicit data setup / clock-low time.
    digitalWrite(CLK_PIN, HIGH);
    delayMicroseconds(1);  // Explicit clock-high time, independent of CPU speed.
  }
  digitalWrite(CLK_PIN, LOW);
}

inline void registerAll(uint8_t address, uint8_t value) {
  digitalWrite(CS_PIN, LOW);
  for (uint8_t module = 0; module < MODULES; ++module) {
    sendByte(address);
    sendByte(value);
  }
  digitalWrite(CS_PIN, HIGH);
}

inline void clearCanvas() {
  for (uint8_t row = 0; row < 8; ++row) logicalRows[row] = 0;
}

inline void pixel(uint8_t x, uint8_t y) {
  if (x < 32 && y < 8) logicalRows[y] |= uint32_t(1UL) << (31 - x);
}

inline void flushCanvas() {
  for (uint8_t module = 0; module < MODULES; ++module)
    for (uint8_t row = 0; row < 8; ++row) rawRows[module][row] = 0;
  for (uint8_t y = 0; y < 8; ++y) {
    for (uint8_t x = 0; x < 32; ++x) {
      if (!(logicalRows[y] & (uint32_t(1UL) << (31 - x)))) continue;
      const uint8_t tile = uint8_t(x / 8);
      const uint8_t module = CHAIN_INDEX_FOR_TILE[tile];
      uint8_t rawX = uint8_t(x % 8), rawY = y;
      if (RAW_MIRROR_X[module]) rawX = uint8_t(7 - rawX);
      if (RAW_MIRROR_Y[module]) rawY = uint8_t(7 - rawY);
      for (uint8_t turn = 0; turn < (RAW_ROTATION[module] & 3); ++turn) {
        const uint8_t oldX = rawX;
        rawX = uint8_t(7 - rawY);
        rawY = oldX;
      }
      rawRows[module][rawY] |= uint8_t(0x80U >> rawX);
    }
  }
  for (uint8_t row = 0; row < 8; ++row) {
    digitalWrite(CS_PIN, LOW);
    // Farthest chip's word must leave the Arduino first; nearest is last.
    for (uint8_t count = MODULES; count > 0; --count) {
      const uint8_t module = uint8_t(count - 1);
      sendByte(uint8_t(row + 1));
      sendByte(rawRows[module][row]);
    }
    digitalWrite(CS_PIN, HIGH);
  }
}

inline void begin() {
  digitalWrite(CS_PIN, HIGH);
  digitalWrite(CLK_PIN, LOW);
  digitalWrite(DIN_PIN, LOW);
  pinMode(CS_PIN, OUTPUT);
  pinMode(CLK_PIN, OUTPUT);
  pinMode(DIN_PIN, OUTPUT);
  registerAll(0x0C, 0);  // Keep outputs off while blanking and configuring.
  registerAll(0x0F, 0);  // Display test off.
  registerAll(0x09, 0);  // No BCD decoding: every register is a row bitmap.
  registerAll(0x0A, 0);  // Minimum intensity for the initial physical test.
  registerAll(0x0B, 7);  // Scan all eight rows.
  clearCanvas();
  flushCanvas();
  registerAll(0x0C, 1);
  lastSeconds = 65535U;
  lastMode = '\0';
}

inline void digit(uint8_t value, uint8_t x) {
  static const uint8_t font[10][7] = {
    {14, 17, 19, 21, 25, 17, 14},  // 0
    {4, 12, 4, 4, 4, 4, 14},      // 1
    {14, 17, 1, 2, 4, 8, 31},     // 2
    {30, 1, 1, 14, 1, 1, 30},     // 3
    {2, 6, 10, 18, 31, 2, 2},     // 4
    {31, 16, 16, 30, 1, 1, 30},   // 5
    {14, 16, 16, 30, 17, 17, 14}, // 6
    {31, 1, 2, 4, 8, 8, 8},       // 7
    {14, 17, 17, 14, 17, 17, 14}, // 8
    {14, 17, 17, 15, 1, 1, 14}    // 9
  };
  if (value > 9) return;
  for (uint8_t y = 0; y < 7; ++y)
    for (uint8_t col = 0; col < 5; ++col)
      if (font[value][y] & (0x10U >> col)) pixel(uint8_t(x + col), y);
}

inline void modeGlyph(char mode) {
  uint8_t rows[5] = {0, 0, 0, 0, 0};
  switch (mode) {
    case 'S': rows[0]=7; rows[1]=4; rows[2]=7; rows[3]=1; rows[4]=7; break;
    case 'T': rows[0]=7; rows[1]=2; rows[2]=2; rows[3]=2; rows[4]=2; break;
    case 'R': rows[0]=6; rows[1]=5; rows[2]=6; rows[3]=5; rows[4]=5; break;
    case 'P': rows[0]=6; rows[1]=5; rows[2]=6; rows[3]=4; rows[4]=4; break;
    case 'H': rows[0]=5; rows[1]=5; rows[2]=7; rows[3]=5; rows[4]=5; break;
    case 'E': rows[0]=7; rows[1]=4; rows[2]=6; rows[3]=4; rows[4]=7; break;
    case 'D': rows[0]=6; rows[1]=5; rows[2]=5; rows[3]=5; rows[4]=6; break;
    default: rows[0]=7; rows[1]=1; rows[2]=2; rows[4]=2; break;
  }
  for (uint8_t y = 0; y < 5; ++y)
    for (uint8_t x = 0; x < 3; ++x)
      if (rows[y] & (4U >> x)) pixel(x, uint8_t(y + 1));
}

inline bool render(uint16_t seconds, char mode) {
  if (seconds > 5999U) seconds = 5999U;  // This layout supports 00:00–99:59.
  if (seconds == lastSeconds && mode == lastMode) return false;
  const uint8_t minutes = uint8_t(seconds / 60U);
  const uint8_t remainder = uint8_t(seconds % 60U);
  clearCanvas();
  modeGlyph(mode);
  digit(uint8_t(minutes / 10), 4);
  digit(uint8_t(minutes % 10), 10);
  pixel(16, 2);
  pixel(16, 4);
  digit(uint8_t(remainder / 10), 18);
  digit(uint8_t(remainder % 10), 24);
  flushCanvas();
  lastSeconds = seconds;
  lastMode = mode;
  return true;
}

inline bool service(uint16_t seconds, char mode) { return render(seconds, mode); }

inline void calibration() {
  // Read left→right as 1 2 3 4, with one top-left corner dot and two adjacent
  // bottom-right dots on each tile.  A normal render restores the live clock.
  static const uint8_t numbers[4][5] = {
    {2, 6, 2, 2, 7}, {7, 1, 7, 4, 7},
    {7, 1, 7, 1, 7}, {5, 5, 7, 1, 1}
  };
  clearCanvas();
  for (uint8_t tile = 0; tile < MODULES; ++tile) {
    const uint8_t left = uint8_t(tile * 8);
    pixel(left, 0);
    pixel(uint8_t(left + 6), 7);
    pixel(uint8_t(left + 7), 7);
    for (uint8_t y = 0; y < 5; ++y)
      for (uint8_t x = 0; x < 3; ++x)
        if (numbers[tile][y] & (4U >> x))
          pixel(uint8_t(left + x + 2), uint8_t(y + 1));
  }
  flushCanvas();
  lastSeconds = 65535U;
  lastMode = '\0';
}
}  // namespace AuxClock
#endif
