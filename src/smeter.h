#ifndef SMETER_H
#define SMETER_H

namespace SMETER
{
  static const uint8_t meter[64] =
  {
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c,
    0x3c
  };

  static const uint32_t peak(const int16_t sig)
  {
    static uint32_t p = 0;
    const uint32_t level = ((uint32_t)abs(sig))<<12; // 128 * 2000 = 256,000, 4096 * 2000 = 8,192,000
    if (level>p)
    {
      p = level;
    }
    else
    {
      uint32_t decay = p>>14;
      if (decay==0) decay = 1;
      if (p>decay) p -= decay;
    }
    return p>>12;
  }
}
#endif
