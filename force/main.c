// Simple code to read the force/torque date using comedi
// Based on the code of Paul Griffiths 
// compile with: gcc main.c atinano.c sleep_time.c keyboard_hit.c -o run -lcomedi -latift-0.1 -lm

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include "atinano.h"
#include "math.h"
#include "sleep_time.h"
#include "keyboard_hit.h"

// nano43
//#define Comedi_Dev "/dev/comedi0"
//#define Calibration "/home/mecha/Sensors/ATI/Calibration/FT9820.cal"

// mini40
#define Comedi_Dev "/dev/comedi0"
#define Calibration "/home/mecha/Sensors/ATI/Calibration/FT11269.cal"

int main( void )
{
	double ft[6];	/**< Array of force torque readings. */
	int ch = 0; // get key
	double mag = 0.0; // force magnitude
	int SAVE_DATA = 0; // flags
	char *data = (char *)malloc(1000); // array to be used for storing data
  FILE *pFile = NULL; // pointer to the file where the data is to be saved
	int DATA_FILE = 0; // flag to control openning of the data file
	double time_var = 0.0; // time
	const double sample_time = 0.01; // sample loop

	init_atinano(Comedi_Dev, Calibration);	/**< Initialize force/torque sensor and nidaq. */
	init_keyboard(); // start reading keyboard	

	while(ch != 'q') {
		readFTSensor();
		getLastFTReading( ft );
		mag = sqrt( ft[0]*ft[0] + ft[1]*ft[1] + ft[2]*ft[2] );
	
    if(kbhit())
      ch = readch();
	
		if(ch == 'm')
			SAVE_DATA = 1;

		if (SAVE_DATA) printf("\nOK baby, time %g\n",time_var);
		printf("\nFx = %lf, \nFy = %lf, \nFz = %lf, \nmag = %lf\n", ft[0], ft[1], ft[2], mag );

		// if SAVED_DATA, put data into a file
    if (SAVE_DATA)
    {
      if (DATA_FILE == 0) {  // create and open file
        pFile = fopen("saved_data.txt","wa"); // open data file
        DATA_FILE = 1; // block call
      }
      // cast data as a string
      sprintf(data,"%g %g %g %g %g\n", time_var, ft[0], ft[1], ft[2], mag);
			// put it into "saved_data.txt"
      fputs(data, pFile);
			time_var += sample_time;
    }

		sleep_time(sample_time); // wait
	
}

	EndSensorReading();
	close_keyboard();
	
	return 1;
}
