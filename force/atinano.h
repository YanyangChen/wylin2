/**
 * \file atinano.h
 * \author Paul Griffiths
 */

// Feb 2011, AddEndSensorReadings function to clear some structures

#ifndef _ATINANO_H_
#define _ATINANO_H_

/** \brief Sample the FT sensor. Result is stored in last_ft_reading */
void readFTSensor( void );

/** \brief Get a copy of the last FT reading.
 * \param ftvec Vector of length 6 to receive [Fx, Fy, Fz, Tx, Ty, Tz] in (N) and (Nm). */
void getLastFTReading( double ftvec[] );

/** \brief Initialize reading the ATI Nano sensor */
int init_atinano(const char *brd, const char *calname);

// Frees memory allocated for Calibration struct
void EndSensorReading ( void );

#endif
