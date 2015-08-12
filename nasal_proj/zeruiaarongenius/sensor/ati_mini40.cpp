#include "ati_mini40.h"

ati_mini40::ati_mini40()
{
}

void ati_mini40::init(double t)
{
  theta = t;
  magnitude = 0;
  R[0][0] = cos(theta); 
  R[0][1] = -sin(theta);
  R[1][0] = sin(theta);
  R[1][1] = cos(theta);
  for(int i = 0; i < ARRAY_LENGTH; i++)
    {
      fs[i] = 0;
      ft[i] = 0;
    }

  init_atinano(Comedi_Dev, Calibration);
}

void ati_mini40::getvalue()
{
  readFTSensor();
  getLastFTReading(ft);

  fs[0] = R[0][0] * ft[0] + R[0][1] * ft[1];
  fs[1] = R[1][0] * ft[0] + R[1][1] * ft[1];
  fs[2] = ft[2];
  magnitude = sqrt(fs[0] * fs[0] + fs[2] * fs[2]);
}

void ati_mini40::close()
{
  EndSensorReading();
}
