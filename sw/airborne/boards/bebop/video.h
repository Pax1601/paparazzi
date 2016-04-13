/*
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file boards/bebop/video.h
 * Initialization of the video specific parts of the Bebop
 */

#include <stddef.h>
#include <inttypes.h>

#ifndef BOARDS_BEBOP_VIDEO_H
#define BOARDS_BEBOP_VIDEO_H

void mt9v117_init(void);
void mt9f002_init(void);

extern void mt9f002_open(void);
extern void mt9f002_close(void);
extern void mt9f002_set_address(uint8_t address);
extern void mt9f002_write_reg8(uint16_t reg, uint8_t value);
extern void mt9f002_write_reg16(uint16_t reg, uint16_t value);
extern uint8_t mt9f002_read_reg8(uint16_t reg);
extern uint16_t mt9f002_read_reg16(uint16_t reg);


#endif /* BOARDS_BEBOP_VIDEO_H */
