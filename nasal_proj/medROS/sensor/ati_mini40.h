#ifndef _ATI_MINI40_H_
#define _ATI_MINI40_H_

#include <math.h>
#include "atinano.h"

#define Comedi_Dev "/dev/comedi0"
#define Calibration "./FT11269.cal"
#define ARRAY_LENGTH 6

class ati_mini40
{
public:
  ati_mini40();

public:
  void getvalue();
  void init(double t);
  void close();

public:
  double magnitude;
  double fs[ARRAY_LENGTH];

private:
  double theta;
  double ft[ARRAY_LENGTH];
  double R[2][2];
};

#endif //_ATI_MINI40_H_
