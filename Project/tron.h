#include "lcd_images.h"
#include "lcd.h"

void tron_draw_up(unsigned int curr_x, unsigned int curr_y, bool remote);
void tron_draw_down(unsigned int curr_x, unsigned int curr_y, bool remote);
void tron_draw_left(unsigned int curr_x, unsigned int curr_y, bool remote);
void tron_draw_right(unsigned int curr_x, unsigned int curr_y, bool remote);
void turn_hor_from_ver(unsigned int curr_x, unsigned int curr_y, bool up, bool remote);
void turn_ver_from_hor(unsigned int curr_x, unsigned int curr_y, bool left, bool remote);

void tron_update_bitmap_lr(unsigned int curr_x, unsigned int curr_y, uint32_t bitmap[240][10]);
void tron_update_bitmap_ud(unsigned int curr_x, unsigned int curr_y, uint32_t bitmap[240][10]);