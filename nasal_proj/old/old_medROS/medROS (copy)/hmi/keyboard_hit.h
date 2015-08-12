/**
 * \file kbhit.h
 * \author David NAVARRO ALARCON
*/

#ifndef KBHIT_H_
#define KBHIT_H_

void init_keyboard( void );
void close_keyboard( void );
int kbhit( void );
int readch( void );

#endif

