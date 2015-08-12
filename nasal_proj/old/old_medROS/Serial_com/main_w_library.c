// code to test serial communication library

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include "rs232.h"

int main(int argc,char** argv)
{
	int rv = 0, i = 0;
	char buff[255];
	static int acc = 0;
	rv = OpenComport(21, 9600);
	printf("Returned value when opening %d\n", rv);
	while(i++<1000000) {
//		memset(&buff,0,sizeof(buff));
		rv = PollComport(21, buff, 2);
		if (rv == 2) {
			buff[rv] = 0;
			if (!strncmp(buff,"-",1)) acc += -1;
			else if (!strncmp(buff,"+",1)) acc += 1;
		}
		else acc += 0;
		printf("Accumulator: %d\n",acc);
	}
	CloseComport(21);
	return 0;
}
