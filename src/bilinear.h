#include <stdint.h>
void bilinear_scale(const uint16_t *src, uint16_t w, uint16_t h,
  uint16_t *dst, const uint16_t newWidth, const uint16_t newHeight);