#include "tron.h"

void tron_draw_up(unsigned int curr_x, unsigned int curr_y, bool remote) {
	uint16_t fg_color;
	
	if (remote) {
		fg_color = LCD_COLOR_RED;
	} else {
		fg_color = LCD_COLOR_BLUE2;
	}
	
	lcd_draw_image(
		curr_x-5,
		10,
		curr_y,
		20,
		tron_up,
		fg_color,
		LCD_COLOR_BLACK
	);
}

void tron_draw_down(unsigned int curr_x, unsigned int curr_y, bool remote) {
	uint16_t fg_color;
	
	if (remote) {
		fg_color = LCD_COLOR_RED;
	} else {
		fg_color = LCD_COLOR_BLUE2;
	}
	
	lcd_draw_image(
		curr_x-5,
		10,
		curr_y-20,
		20,
		tron_down,
		fg_color,
		LCD_COLOR_BLACK
	);
}

void tron_draw_left(unsigned int curr_x, unsigned int curr_y, bool remote) {
	uint16_t fg_color;
	
	if (remote) {
		fg_color = LCD_COLOR_RED;
	} else {
		fg_color = LCD_COLOR_BLUE2;
	}
	
	lcd_draw_image(
		curr_x,
		20,
		curr_y-5,
		10,
		tron_up,
		fg_color,
		LCD_COLOR_BLACK
	);
}

void tron_draw_right(unsigned int curr_x, unsigned int curr_y, bool remote) {
	uint16_t fg_color;
	
	if (remote) {
		fg_color = LCD_COLOR_RED;
	} else {
		fg_color = LCD_COLOR_BLUE2;
	}
	
	lcd_draw_image(
		curr_x-20,
		20,
		curr_y-5,
		10,
		tron_up,
		fg_color,
		LCD_COLOR_BLACK
	);
}

void tron_update_bitmap_lr(unsigned int curr_x, unsigned int curr_y, uint32_t bitmap[240][10]) {
	bitmap[curr_x][(int)((curr_y-5)/32)] |= 0x03FF << ((curr_y-5) % 32);
	bitmap[curr_x][(int)((curr_y-4)/32)] |= 0x03FF << ((curr_y-4) % 32);
}

void tron_update_bitmap_ud(unsigned int curr_x, unsigned int curr_y, uint32_t bitmap[240][10]) {
	int i;
	
	for (i=curr_x-5; i<curr_x+5; i++) {
		bitmap[curr_x][(int)(curr_y/32)] |= 0x1 << (curr_y % 32);
	}
}
