#ifdef USE_PID

#include <PID_v1.h>

double pidTemperature = -1000.0;


//Define the aggressive and conservative Tuning Parameters
//double aggKp=1, aggKi=0, aggKd=0;
double kP = 40, kI = 0, kD = 0;

double setPoint, output;

bool relayState, lastRelayState;

//PID temperaturePID(&pidPemperature, &output, &setPoint, consKp, consKi, consKd, P_ON_M, DIRECT);
PID temperaturePID(&pidPemperature, &output, &setPoint, kP, kI, kD, DIRECT);

int windowSize = 5000;
int minWindow = 1000;
int relayOnTime;
unsigned long windowStartTime;

void initPID() {
  setPoint = 28.0;

  windowStartTime = millis();

  temperaturePID.SetMode(AUTOMATIC);
  temperaturePID.SetSampleTime(1000);

  Serial.println("PID Temperature;output;relayOnTime;relayState");
}


void pollPID() {

  if (pidPemperature < (-999.00)) {
    return;
  }

  unsigned long now = millis();

  if (now - windowStartTime > windowSize) {
    windowStartTime += windowSize;

    //   double gap = abs(setPoint - pidPemperature); //distance away from setpoint
    //  if (gap < 2)
    //  {  //we're close to setpoint, use conservative tuning parameters
    //    temperaturePID.SetTunings(aggKp, aggKi, aggKd, P_ON_M); // TEMP
    //    //temperaturePID.SetTunings(consKp, consKi, consKd, P_ON_M);
    //  }
    //  else
    //  {
    //    //we're far from setpoint, use aggressive tuning parameters
    //    temperaturePID.SetTunings(aggKp, aggKi, aggKd, P_ON_M);
    //  }

    temperaturePID.Compute();

    Serial.print(pidPemperature);
    Serial.print(";");
    Serial.print(output);
    Serial.print(";");
    Serial.print(relayOnTime);
    Serial.print(";");
    Serial.print(relayState);
    Serial.println("");
    lastRelayState = relayState;

  } else {
    temperaturePID.Compute();
  }

  relayOnTime = (output / 255) * windowSize;
  if (relayOnTime > minWindow && relayOnTime < now - windowStartTime) {
    relayState = 1;
  }
  else if (relayOnTime > minWindow && windowSize - relayOnTime < minWindow) {
    relayState = 1;
  } else {
    relayState = 0;
  }


  if (lastRelayState != relayState) {
  
  }

}

#define XDRV_08

boolean Xdrv08(byte function)
{
  boolean result = false;

  switch (function) {
    case FUNC_INIT:
      initPID();
      break;
    case FUNC_EVERY_SECOND:
      pollPID();
      break;
  }

  return result;
}



#endif // USE_PID
