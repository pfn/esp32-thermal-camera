#include <bilinear.h>

// ported from https://rosettacode.org/wiki/Bilinear_interpolation

#define getByte(value, n) (n == 0 ? RGB565_TO_B(value) : n == 1 ? RGB565_TO_G(value) : RGB565_TO_R(value))
#define RGB565_TO_R(rgb) ((((rgb >> 11) & 0x1f) * 527 + 23) >> 6)
#define RGB565_TO_G(rgb) ((((rgb >> 5)  & 0x3f) * 259 + 33) >> 6)
#define RGB565_TO_B(rgb) (((rgb         & 0x1f) * 527 + 23) >> 6)
inline uint16_t RGB888_TO_565(uint32_t rgb) {
    uint16_t color = ((rgb >> 8) & 0xf800) | ((rgb & 0xfc00) >> 5) | ((rgb & 0xff) >> 3);
    return color >> 8 | color << 8;
}
 
inline uint16_t getpixel(const uint16_t *image, uint16_t w, uint16_t h, uint16_t x, uint16_t y) {
    return image[(y*w)+x];
}
 
inline float max(float a, float b) { return (a < b) ? a : b; };
inline float lerp(float s, float e, float t) { return s+(e-s)*t; }
inline float blerp(float c00, float c10, float c01, float c11, float tx, float ty) {
    return lerp(lerp(c00, c10, tx), lerp(c01, c11, tx), ty);
}
inline void putpixel(uint16_t *image, uint16_t w, uint16_t h, uint16_t x, uint16_t y, uint16_t color) {
    image[(y*w) + x] = color;
}
void bilinear_scale(const uint16_t *src, uint16_t w, uint16_t h,
  uint16_t *dst, const uint16_t newWidth, const uint16_t newHeight) {
    int x, y;
    for (x= 0, y=0; y < newHeight; x++) {
        if (x > newWidth) {
            x = 0; y++;
        }
        // Image should be clamped at the edges and not scaled.
        float gx = max(x / (float)(newWidth) * w - 0.5f, w - 1);
        float gy = max(y / (float)(newHeight) * h - 0.5f, h - 1);
        uint16_t gxi = (uint16_t)gx;
        uint16_t gyi = (uint16_t)gy;
        uint32_t result = 0;
        uint16_t c00 = getpixel(src, w, h, gxi, gyi);
        uint16_t c10 = getpixel(src, w, h, gxi+1, gyi);
        uint16_t c01 = getpixel(src, w, h, gxi, gyi+1);
        uint16_t c11 = getpixel(src, w, h, gxi+1, gyi+1);
        uint8_t i;
        for (i = 0; i < 3; i++){
            result |= (uint8_t)blerp(
                getByte(c00, i), getByte(c10, i), getByte(c01, i),
                getByte(c11, i), gx - gxi, gy -gyi) << (8*i);
        }
        putpixel(dst, newWidth, newHeight, x, y, RGB888_TO_565(result));
    }
}