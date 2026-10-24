#ifndef ELEC281_PRESET_DISPLAY_H
#define ELEC281_PRESET_DISPLAY_H

// Screen coordinates are upright on the physically diamond-mounted displays:
// x = col - row, y = row + col - 7.  Only opposite-parity x/y positions exist.
// putPixel retains the individually verified module rotations and flip mapping.
namespace PresetDisplay {
inline void screenPixel(uint8_t module, int8_t x, int8_t y) {
  const int8_t rowTwice = int8_t(y + 7 - x);
  const int8_t colTwice = int8_t(y + 7 + x);
  if ((rowTwice & 1) || (colTwice & 1)) return;
  const int8_t row = int8_t(rowTwice / 2);
  const int8_t col = int8_t(colTwice / 2);
  if (row >= 0 && row < 8 && col >= 0 && col < 8)
    putPixel(module, uint8_t(row), uint8_t(col), true);
}

// A square 3 x 5 font sampled on alternate screen positions, rather than
// drawing in the native matrix rows (which would lean through 45 degrees).
inline void glyph(uint8_t module, const uint8_t (&rows)[5]) {
  for (uint8_t y = 0; y < 5; ++y)
    for (uint8_t x = 0; x < 3; ++x)
      if (rows[y] & uint8_t(4U >> x))
        screenPixel(module, int8_t(2 * x - 2), int8_t(2 * y - 5));
}

inline void twoDigits(uint8_t module, bool thirty) {
  // Two seven-row outline digits fit the diamond's widening middle.  Each
  // stroke is snapped to a real LED; the outer corners are never clipped.
  for (int8_t y = -3; y <= 3; ++y) {
    const bool even = (y % 2 == 0);
    const int8_t left = even ? 1 : 2;
    const int8_t right = even ? 5 : 4;
    screenPixel(module, left, y);
    screenPixel(module, right, y);
  }
  if (thirty) {
    for (int8_t y = -3; y <= 3; ++y) {
      if (y == -3 || y == 3) {
        screenPixel(module, -4, y);
        screenPixel(module, -2, y);
      } else if (y == 0) {
        screenPixel(module, -3, y);
        screenPixel(module, -1, y);
      } else {
        screenPixel(module, (y % 2 == 0) ? -1 : -2, y);
      }
    }
  } else {
    for (int8_t y = -3; y <= 3; y += 2) screenPixel(module, -2, y);
    screenPixel(module, -4, -1);  // Short ascending serif identifies the one.
    screenPixel(module, -4, 3);
    screenPixel(module, 0, 3);
  }
}
inline void minuteUnit(uint8_t module) {
  // Widely separated vertical stems and a descending centre valley distinguish
  // M from the H-like shape of a three-column font at this resolution.
  for (int8_t y = -5; y <= 3; ++y) {
    const int8_t edge = (y % 2 == 0) ? 3 : 2;
    screenPixel(module, -edge, y);
    screenPixel(module, edge, y);
  }
  screenPixel(module, -1, -4);
  screenPixel(module, 1, -4);
  screenPixel(module, 0, -3);
}
}  // namespace PresetDisplay

void renderPresetTime(uint8_t index, bool editing) {
  if (index > 5) index = 1;  // A corrupt selection visibly falls back to 1 min.
  static const uint8_t one[5] = {2, 6, 2, 2, 7};
  static const uint8_t two[5] = {7, 1, 7, 4, 7};
  static const uint8_t three[5] = {7, 1, 7, 1, 7};
  static const uint8_t five[5] = {7, 4, 7, 1, 7};
  static const uint8_t seconds[5] = {7, 4, 7, 1, 7};
  clearFrame();
  const uint8_t top = upperModule();
  const uint8_t bottom = uint8_t(1 - top);
  if (index == 0 || index == 5) PresetDisplay::twoDigits(top, index == 0);
  else if (index == 1) PresetDisplay::glyph(top, one);
  else if (index == 2) PresetDisplay::glyph(top, two);
  else if (index == 3) PresetDisplay::glyph(top, three);
  else PresetDisplay::glyph(top, five);
  if (index == 0) PresetDisplay::glyph(bottom, seconds);
  else PresetDisplay::minuteUnit(bottom);
  if (editing) PresetDisplay::screenPixel(top, 0, -7);
  flushFrame();
}

#endif
