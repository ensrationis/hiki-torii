#ifndef _EPD_4IN2_V2_H_
#define _EPD_4IN2_V2_H_

#include "Debug.h"
#include "DEV_Config.h"

// Display resolution
#define EPD_4IN2_V2_WIDTH       400
#define EPD_4IN2_V2_HEIGHT      300

#define Seconds_1_5S      0
#define Seconds_1S        1

void EPD_4IN2_V2_Init(void);
void EPD_4IN2_V2_Init_Fast(UBYTE Mode);
void EPD_4IN2_V2_Init_4Gray(void);
bool EPD_4IN2_V2_Clear(void);
bool EPD_4IN2_V2_Display(UBYTE *Image);
bool EPD_4IN2_V2_Display_Fast(UBYTE *Image);
bool EPD_4IN2_V2_Display_4Gray(const UBYTE *Image);
bool EPD_4IN2_V2_PartialDisplay(UBYTE *Image, UWORD Xstart, UWORD Ystart, UWORD Xend, UWORD Yend);
bool EPD_4IN2_V2_PartialDisplay(UBYTE *Image);
void EPD_4IN2_V2_Sleep(void);
bool EPD_4IN2_V2_ReadBusy(void);
void EPD_4IN2_V2_Reset(void);

#endif
