#ifdef USE_PID

#include <PID_v1.h>


enum PidCommands {
  CMND_PID, CMND_PID_TEMPERATURE, CMND_PID_P, CMND_PID_I, CMND_PID_D
};

const char kPidCommands[] PROGMEM =
  D_CMND_PID "|" D_CMND_PID_TEMPERATURE "|" D_CMND_PID_P "|" D_CMND_PID_I "|" D_CMND_PID_D;


double pidTemperature = -1000.0;

//Define the aggressive and conservative Tuning Parameters
//double aggKp=1, aggKi=0, aggKd=0;
double kP = 60, kI = 0.01, kD = 0;

double setPoint, output;

boolean pidActive, pidActiveLastState, relayState, lastRelayState;

//PID temperaturePID(&pidTemperature, &output, &setPoint, consKp, consKi, consKd, P_ON_M, DIRECT);
//PID temperaturePID(&pidTemperature, &output, &setPoint, kP, kI, kD, DIRECT);
PID temperaturePID(&pidTemperature, &output, &setPoint, kP, kI, kD, P_ON_M, DIRECT);

unsigned long windowSize = 1000 * 60;
unsigned long minWindow = 1000 * 10;
unsigned long logInterval = 1000 * 30;

unsigned long relayOnTime;
unsigned long windowStartTime;

unsigned long logTimer = 0;

float safetyLimit = 1.05; // Percent of taregt temperature

void startPID() {
  pidActive = true;
  temperaturePID.SetMode(AUTOMATIC);
  windowStartTime = millis();
  logTimer = millis();
}


void stopPID() {
  pidActive = false;
  temperaturePID.SetMode(MANUAL);
  ExecuteCommandPower(1, 0);
}


void initPID() {
  setPoint = 24.0;
  
  temperaturePID.SetSampleTime(1000);  
  startPID();  
}


void publishPidState() {
  snprintf_P(mqtt_data, sizeof(mqtt_data), "%s", GetStateText(pidActive));
  MqttPublishPrefixTopic_P(STAT, D_CMND_PID, 1);
}


void pollPID() {

  // Fix bug of wrong temperature in beginning (85.0) or when temperature was not yet intialized
  if (pidTemperature == 85.0 || pidTemperature < (-999.00)) {
    return;
  }
  
  if (pidTemperature >= setPoint * safetyLimit) {
    stopPID();
    return;
  }

  if (pidActive != pidActiveLastState) {
    pidActiveLastState = pidActive;
    publishPidState();  
  }

  if (pidActive == false) {
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
  

  } else {
    temperaturePID.Compute();
  }

  if (now - logTimer > logInterval) {
    logTimer += logInterval;

    char pidTemperature_chr[10];
    char output_chr[10];
    char relayOnTime_chr[10];
    char setPoint_chr[1];
    char pidActive_chr[5];

    dtostrfd(pidTemperature, Settings.flag2.temperature_resolution, pidTemperature_chr);
    dtostrfd((output/255) * 100, 0, output_chr);
    dtostrfd(relayOnTime, 0, relayOnTime_chr);
    dtostrfd(setPoint, 0, setPoint_chr);
    if (pidActive) {
      snprintf_P(pidActive_chr, sizeof(pidActive_chr), PSTR("true"));
    } else {
      snprintf_P(pidActive_chr, sizeof(pidActive_chr), PSTR("false"));
    }

    snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("{\"" "time" "\":\"%s\","), GetDateAndTime().c_str());
    snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("%s \"pid\": {\"active\": %s, \"temperature\": %s, \"pidValue\": %s, \"relayOnTime\": %s, \"targetTemperature\": %s}"), mqtt_data, pidActive_chr, pidTemperature_chr, output_chr, relayOnTime_chr, setPoint_chr );
    snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("%s}"), mqtt_data);
    MqttPublishPrefixTopic_P(TELE, PSTR("PID"), Settings.flag.mqtt_sensor_retain);
   
  }
    
  relayOnTime = (output / 255) * windowSize;
  
  
  if (relayOnTime > minWindow && relayOnTime > now - windowStartTime) {
    relayState = 1;
  }
  else if (relayOnTime > minWindow && relayOnTime <= now - windowStartTime && windowSize - relayOnTime < minWindow) {
    relayState = 1;
  } else {
    relayState = 0;
  }

  if (lastRelayState != relayState) {
    lastRelayState = relayState;
    ExecuteCommandPower(1, relayState);
  }

}


boolean PidCommand() {

  char command [CMDSZ];
  boolean serviced = true;
  uint8_t status_flag = 0;

  int command_code = GetCommandCode(command, sizeof(command), XdrvMailbox.topic, kPidCommands);
  if (CMND_PID == command_code) {
    if ((XdrvMailbox.payload >= 0) && (XdrvMailbox.payload <= 1)) {
      switch (XdrvMailbox.payload) {
        case 0: // Off
          stopPID();
          break;
        case 1: // On
          startPID();
          break;
      }
    } 
    snprintf_P(mqtt_data, sizeof(mqtt_data), S_JSON_COMMAND_SVALUE, command, GetStateText(pidActive));
  }
  else if (CMND_PID_TEMPERATURE == command_code) {
    if (XdrvMailbox.payload >= 0) {
      setPoint = XdrvMailbox.payload;
    }
    snprintf_P(mqtt_data, sizeof(mqtt_data), S_JSON_COMMAND_NVALUE, command, setPoint);
  }
  else if (CMND_PID_P == command_code) {
    if (XdrvMailbox.payload >= 0) {
      kP = XdrvMailbox.payload;
    }
    snprintf_P(mqtt_data, sizeof(mqtt_data), S_JSON_COMMAND_NVALUE, command, kP);
  }
  else if (CMND_PID_I == command_code) {
    if (XdrvMailbox.payload >= 0) {
      kI = XdrvMailbox.payload;
    }
    snprintf_P(mqtt_data, sizeof(mqtt_data), S_JSON_COMMAND_NVALUE, command, kI);
  }
  else if (CMND_PID_D == command_code) {
    if (XdrvMailbox.payload >= 0) {
      kD = XdrvMailbox.payload;
    }
    snprintf_P(mqtt_data, sizeof(mqtt_data), S_JSON_COMMAND_NVALUE, command, kD);
  }
  else {
    serviced = false;
  }
  
  return serviced;
}



/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

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
    case FUNC_COMMAND:
        result = PidCommand();
        break;
  }

  return result;
}



#endif // USE_PID

