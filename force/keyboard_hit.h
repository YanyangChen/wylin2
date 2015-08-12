/**
 * \file kbhit.h
 * \author David NAVARRO ALARCON
*/

#ifndef _KBHIT_H_
#define _KBHIT_H_

void init_keyboard( void );
void close_keyboard( void );
int kbhit( void );
int readch( void );

#endif

