/* Name: Nibras Alam & Al Hisham Anik
# ID: 1617818 & 1585385
# CMPUT 275, Winter 2020
#
# Assignment 2: Driving Route Finder Part 2
#
# Consulted C++ Documentation; Used Lecture Slides
*/
/*
 * Routine for drawing an image patch from the SD card to the LCD display.
 */

#ifndef _LCD_IMAGE_H
#define _LCD_IMAGE_H

#include <MCUFRIEND_kbv.h>

typedef struct {
  char file_name[50];
  uint16_t ncols;
  uint16_t nrows;
} lcd_image_t;

/* Draws the referenced image to the LCD screen.
 *
 * img           : the image to draw
 * tft           : the initialized tft struct
 * icol, irow    : the upper-left corner of the image patch to draw
 * scol, srow    : the upper-left corner of the screen to draw to
 * width, height : controls the size of the patch drawn.
 */
void lcd_image_draw(const lcd_image_t *img, MCUFRIEND_kbv *tft,
		    uint16_t icol, uint16_t irow,
		    uint16_t scol, uint16_t srow,
		    uint16_t width, uint16_t height);

#endif
