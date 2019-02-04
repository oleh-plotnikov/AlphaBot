#ifndef NIMBLE_DISPLAY_H_
#define NIMBLE_DISPLAY_H_

#define OLED_RESET 9
#define OLED_SA0   8

typedef enum DEST
{
    eNIMBLE_LCD,
    eNIMBLE_SERIAL,
} nimble_destination_t;

void nimble_display_init();
void nimble_debug_print(const char c[], int type);

#endif