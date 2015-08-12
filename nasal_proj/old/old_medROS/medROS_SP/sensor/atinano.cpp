/**
 * \file atinano.c 
 * \author Paul Griffiths 
 */

// Some "little" modifications were made to this source code
// by David NAVARRO ALARCON, Feb 2011

#include <stdio.h>
#include <stdlib.h>
#include <comedilib.h>
#include <atift-0.1/ATI/ftconfig.h>
#include "atinano.h"

// DAQ card stuff
#define ADDA_InChan 6
comedi_t *ADDA_Board;
const int ADDA_InDevice = 0;
const int ADDA_InRange = 0; // range = 0 for [-10, 10]
const int aref= AREF_DIFF;
int MaxDataIn;

// Force sensor stuff
Calibration *FTCal;  //struct containing calibration information
short FTUnit; //return value from functions
float volts[6] = {0,0,0,0,0,0}, last_ft_reading[6] = {0,0,0,0,0,0};
int idx, i;
unsigned int data;


/* Get a copy of last_ft_read */
void getLastFTReading( double ftvec[] )
{
    for (idx = 0; idx<6; idx++ )
        ftvec[idx] = last_ft_reading[idx];
}

/* Sample the FT sensor. Result is stored in last_ft_reading */
void readFTSensor( void )
{
    for( i = 0; i < ADDA_InChan; i++ )
    {
        comedi_data_read(ADDA_Board,ADDA_InDevice,i,ADDA_InRange,aref,&data);
        volts[i]=comedi_to_phys(data,comedi_get_range(ADDA_Board,ADDA_InDevice,i,ADDA_InRange),MaxDataIn );
    }
    ConvertToFT( FTCal, volts, last_ft_reading );
}

int init_atinano(const char *brd, const char *calname)
{
   // open the comedi device
    ADDA_Board = comedi_open(brd);
    if(!ADDA_Board)
    {
        comedi_perror(brd);
                    return -1;
    }

    // force sensor configuration and initialization
    FTCal = createCalibration(calname, 1);
    if (FTCal == NULL)
        {
            fprintf(stderr,"FT calibration file could not be loaded!!\n");
            exit(1);
        }

    // set force/torque unit -- force[N] and torque[Nm]
    FTUnit = SetForceUnits(FTCal,"N");
    if (FTUnit > 0)
        {
            fprintf(stderr,"Error with setting force unit\n");
            exit(1);
        }

    FTUnit = SetTorqueUnits(FTCal,"Nm");
    if (FTUnit > 0)
    {
        fprintf(stderr,"Error with setting torque unit\n");
        exit(1);
    }

    // maxdata range for the input and output channels. (same at 65535
    // for all input channels)
    MaxDataIn = comedi_get_maxdata(ADDA_Board,ADDA_InDevice,0);

	// Get volts from DAQ card
    for( i = 0; i < ADDA_InChan; i++ )
    {
        comedi_data_read(ADDA_Board,ADDA_InDevice,i,ADDA_InRange,aref,&data);
        volts[i]=comedi_to_phys(data,comedi_get_range(ADDA_Board,ADDA_InDevice,i,ADDA_InRange),MaxDataIn );
    }

	// Get bias on the measurement
	Bias( FTCal, volts );

    return 1;
}

void EndSensorReading ( void )
{
	// Frees memory allocated for Calibration struct by a successful call to createCalibration
	destroyCalibration( FTCal );
	printf("\nClear\n");
}

