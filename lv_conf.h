#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

#define LV_COLOR_DEPTH          16
#define LV_COLOR_16_SWAP        0

#define LV_DISP_DEF_REFR_PERIOD  30

#define LV_MEM_SIZE  (64 * 1024U)

/* Speicher aus PSRAM */
#define LV_MEM_CUSTOM           1
#define LV_MEM_CUSTOM_INCLUDE   <stdlib.h>
#define LV_MEM_CUSTOM_ALLOC     malloc
#define LV_MEM_CUSTOM_FREE      free
#define LV_MEM_CUSTOM_REALLOC   realloc

/* Ticker */
#define LV_TICK_CUSTOM          1
#define LV_TICK_CUSTOM_INCLUDE  <Arduino.h>
#define LV_TICK_CUSTOM_SYS_TIME_EXPR (millis())

/* Logging */
#define LV_USE_LOG              1
#define LV_LOG_LEVEL            LV_LOG_LEVEL_WARN
#define LV_LOG_PRINTF           1

/* Widgets – nur was wir brauchen */
#define LV_USE_LABEL            1
#define LV_USE_BTN              1
#define LV_USE_ARC              1
#define LV_USE_LINE             1
#define LV_USE_IMG              1   /* Für Windzeiger-Bild */
#define LV_USE_CANVAS           1   /* Für gefüllte Polygone */
#define LV_USE_METER            0
#define LV_USE_CHART            0

/* Diese Widgets aus */
#define LV_USE_TEXTAREA         0
#define LV_USE_KEYBOARD         0
#define LV_USE_SPINBOX          0
#define LV_USE_MSGBOX           0
#define LV_USE_SPINNER          0
#define LV_USE_CALENDAR         0
#define LV_USE_COLORWHEEL       0
#define LV_USE_IMGBTN           0
#define LV_USE_LIST             0
#define LV_USE_MENU             0
#define LV_USE_TABVIEW          0
#define LV_USE_TILEVIEW         0
#define LV_USE_WIN              0

/* Standard Fonts */
#define LV_FONT_MONTSERRAT_14   1
#define LV_FONT_MONTSERRAT_24   1
#define LV_FONT_MONTSERRAT_18   1   // neu hinzugefügt

#define LV_FONT_MONTSERRAT_48   1   /* Eigene Version verwendet */
#define LV_FONT_MONTSERRAT_120   1   /* Eigene Version verwendet */
#define LV_FONT_MONTSERRAT_200   1   /* Eigene Version verwendet */
//#define LV_FONT_MONTSERRAT_240   1   /* Eigene Version verwendet */
#define montserrat_240_bold_numeric 1

#define LV_FONT_DEFAULT         &lv_font_montserrat_14

#define LV_USE_FONT_COMPRESSED 1


/* Erlaubt große Fonts (Arrays > 64KB) */
#define LV_FONT_FMT_TXT_LARGE   1

/* Stelle sicher, dass der Font-Engine genug Cache zur Verfügung steht */
#define LV_FONT_CUSTOM_DECLARE  /* nichts eintragen */

/* Theme */
#define LV_USE_THEME_DEFAULT    1
#define LV_THEME_DEFAULT_DARK   1

/* GPU Beschleunigung falls verfügbar */
#define LV_USE_GPU              0
#define LV_USE_GPU_STM32_DMA2D  0
#define LV_USE_GPU_NXP_PXP      0
#define LV_USE_GPU_NXP_VG_LITE  0
#define LV_USE_GPU_SDL          0

/* Draw komplexe Layer */
#define LV_DRAW_COMPLEX         1
#define LV_USE_DRAW_MASK        1   /* Wichtig für Canvas-Polygone */

/* Schatten */
#define LV_USE_SHADOW           0

/* Gradient */
#define LV_USE_GRADIENT         0

#endif /* LV_CONF_H */