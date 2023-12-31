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
    static const uint32_t DECAY_POWER = 14u;
    static uint32_t p = 0;
    const uint32_t abs_raw = abs(sig);
    const uint32_t abs_sig = abs_raw>5?abs_raw-5:0;
    const uint32_t level = abs_sig<<DECAY_POWER;
    if (level>p)
    {
      p = level;
    }
    else
    {
      uint32_t decay = p>>15;
      if (decay==0) decay = 1;
      if (p>decay) p -= decay;
    }
    return p>>DECAY_POWER;
  }
}
#endif
