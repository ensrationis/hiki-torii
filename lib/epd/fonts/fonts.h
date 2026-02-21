#ifndef __FONTS_H
#define __FONTS_H

#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif

// ASCII bitmap font
typedef struct _tFont
{
  const uint8_t *table;
  uint16_t Width;
  uint16_t Height;
} sFONT;

extern sFONT Font24;
extern sFONT Font20;
extern sFONT Font16;

#ifdef __cplusplus
}
#endif

#endif
