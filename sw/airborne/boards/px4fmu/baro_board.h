
/*
 * board specific fonctions for the PX4FMU board
 *
 */

#ifndef BOARDS_PX4FMU_BARO_H
#define BOARDS_PX4FMU_BARO_H

// only for printing the baro type during compilation
#define BARO_BOARD BARO_BOARD_MS5611_I2C

extern void baro_event(void);

#define BaroEvent baro_event

#endif /* BOARDS_PX4FMU_BARO_H */
