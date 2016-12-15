#include "tron.h"

void tron_draw_up(unsigned int curr_x, unsigned int curr_y, bool remote, bool box) {
	uint16_t fg_color;
	uint16_t bg_color;
	
	if (remote) {
		fg_color = REMOTE_FG;
		bg_color = REMOTE_BG;
	} else {
		fg_color = SELF_FG;
		bg_color = SELF_BG;
	}
	if (box) {
		fg_color = bg_color;
	}
	
	lcd_draw_image(
		curr_x-5,
		10,
		curr_y,
		20,
		tron_up,
		fg_color,
		bg_color
	);
}

void tron_draw_down(unsigned int curr_x, unsigned int curr_y, bool remote, bool box) {
	uint16_t fg_color;
	uint16_t bg_color;
	
	if (remote) {
		fg_color = REMOTE_FG;
		bg_color = REMOTE_BG;
	} else {
		fg_color = SELF_FG;
		bg_color = SELF_BG;
	}
	if (box) {
		fg_color = bg_color;
	}
	
	lcd_draw_image(
		curr_x-5,
		10,
		curr_y-20,
		20,
		tron_down,
		fg_color,
		bg_color
	);
}

void tron_draw_left(unsigned int curr_x, unsigned int curr_y, bool remote, bool box) {
	uint16_t fg_color;
	uint16_t bg_color;
	
	if (remote) {
		fg_color = REMOTE_FG;
		bg_color = REMOTE_BG;
	} else {
		fg_color = SELF_FG;
		bg_color = SELF_BG;
	}
	if (box) {
		fg_color = bg_color;
	}
	
	lcd_draw_image(
		curr_x,
		20,
		curr_y-5,
		10,
		tron_left,
		fg_color,
		bg_color
	);
}

void tron_draw_right(unsigned int curr_x, unsigned int curr_y, bool remote, bool box) {
	uint16_t fg_color;
	uint16_t bg_color;
	
	if (remote) {
		fg_color = REMOTE_FG;
		bg_color = REMOTE_BG;
	} else {
		fg_color = SELF_FG;
		bg_color = SELF_BG;
	}
	if (box) {
		fg_color = bg_color;
	}
	
	lcd_draw_image(
		curr_x-20,
		20,
		curr_y-5,
		10,
		tron_right,
		fg_color,
		bg_color
	);
}

void turn_hor_from_ver(unsigned int curr_x, unsigned int curr_y, bool up, bool remote){
	uint16_t fg_color;
	uint8_t *trail;

	if (remote) {
		fg_color = LCD_COLOR_RED;
	} else {
		fg_color = LCD_COLOR_BLUE2;
	}

	if (up){
		trail = (uint8_t *)ver_trail_for_up;
	} else {
		trail = (uint8_t *)ver_trail_for_down;
	}

	lcd_draw_image(
			curr_x,                 // X Pos
			imageWidthPixels,   // Image Horizontal Width
			curr_y,                 // Y Pos
			imageHeightPixels,  // Image Vertical Height
			trail,       // Image
			fg_color,      // Foreground Color
			LCD_COLOR_BLACK     // Background Color
		);
}
void turn_ver_from_hor(unsigned int curr_x, unsigned int curr_y, bool left, bool remote){
	uint16_t fg_color;
	uint8_t *trail;

	if (remote) {
		fg_color = LCD_COLOR_RED;
	} else {
		fg_color = LCD_COLOR_BLUE2;
	}

	if (left){
		trail = (uint8_t *)hor_trail_for_left;
	} else {
		trail = (uint8_t *)hor_trail_for_right;
	}

	lcd_draw_image(
			curr_x,                 // X Pos
			imageHeightPixels,   // Image Horizontal Width
			curr_x,                 // Y Pos
			imageWidthPixels,  // Image Vertical Height
			trail,       // Image
			fg_color,      // Foreground Color
			LCD_COLOR_BLACK     // Background Color
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
