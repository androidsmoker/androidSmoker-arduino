/**
   The Android Smoker sketch is used to control the custom Android BBQ Smoker.

   The custom board it is controlling has:
     - 2 food temperature probes
     - 1  pit temperature probe
     - 1 brushless fan to stoke fire
     - 1 wind speed sensor (not currently used)

   This program reads the temperature values and uses a PID library to control determine when to run the fan in order to maintain temperature at a given setpoint.

   It also communicates all sensor data and can receive commands from an android phone plugged in via USB (using android accessory kit - BBQ Accessory!)

   The android phone has a basic UI and in turn exposes a websocket so clients from anywhere can connect

   Finally there is a website with a full UI that connects to the websocket.  Typically this is run on iPads to monitor the smoker from within the house.

   The probes used in this project are Maverick XXX - There were some great projects that others had done to interface with these probes, in particular the
   "HeaterMeter" https://github.com/CapnBry/HeaterMeter project by Bryan Mayland is a great resource.

   BBQ Build Documented at: http://www.bbq-brethren.com/forum/showthread.php?t=127852

*/
#include <Max3421e.h>
#include <Usb.h>
#include <AndroidAccessory.h>
#include <stdlib.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>


//Android Accessory Library
AndroidAccessory acc("Kapapa",
                     "AndroidSmoker",
                     "AndroidSmoker",
                     "1.0",
                     "http://androidsmoker.com",
                     "0000000012345678");

//sensor vars
unsigned long lastTime = 0;
unsigned long lastSensorTime = 0;
double setpoint, fanSpeed;
double pitTemp, food1, food2;
String pidStatus = "AUTO";
double kp = 5, ki = 0.02, kd = 0;

double aTuneStep = 25, aTuneNoise = 2, aTuneStartValue = 250;
unsigned int aTuneLookBack = 20;
boolean tuning = false;

//PID Library
PID pid(&pitTemp, &fanSpeed, &setpoint, kp, ki, kd, DIRECT);
PID_ATune pidTuner(&pitTemp, &fanSpeed);

//declare local functions
void fmtDouble(double val, byte precision, char *buf, unsigned bufLen = 0xffff);
unsigned fmtUnsigned(unsigned long val, char *buf, unsigned bufLen = 0xffff, byte width = 0);
void setup();
void loop();

void setup()
{
  Serial.begin(115200);
  analogWrite(3, 0);

  getTemps();
  setpoint = 250;

  pid.SetSampleTime(2000);
  pid.SetMode(AUTOMATIC);

  delay(50);
  Serial.print("call poweron...\n");
  acc.powerOn();
}

void loop() {
  //pid computation
   pidUpdate();
   
  //anything below 5 will not turn the fan
  if (fanSpeed < 5)
    fanSpeed = 0;

  //process data on each loop
  readFromPhone();
  sendSensorData();
  getTemps();

  delay(10);
}

void pidUpdate() {
  //auto-tune

  if (tuning)
  {
    byte val = (pidTuner.Runtime());
    if (val != 0)
    {
      tuning = false;
    }
    if (!tuning)
    { //we're done, set the tuning parameters
      kp = pidTuner.GetKp();
      ki = pidTuner.GetKi();
      kd = pidTuner.GetKd();

      pid.SetTunings(kp, ki, kd);

      char skp[10];
      char ski[10];
      char skd[10];

      fmtDouble(kp, 2, skp, 0xffff);
      fmtDouble(ki, 2, ski, 0xffff);
      fmtDouble(kd, 2, skd, 0xffff);

      char str[128];
      sprintf(str, "AUTOTUNE,%s,%s,%s\n", skp, ski, skd);
      Serial.print(str);

      if (acc.isConnected()) {
        String s = String(str);
        int len = s.length();
        Serial.print("write output to usb: ");
        Serial.println(len);
        acc.write(str, len);
      }

    }
  }
  else {
    if (pid.Compute()) {
      char str[128];
      char error[20];
      char iterm[20];
      char dinput[20];

      fmtDouble(pid.GetError(), 2, error, 0xffff);
      fmtDouble(pid.GetITerm(), 2, iterm, 0xffff);
      fmtDouble(pid.GetDInput(), 2, dinput, 0xffff);

      sprintf(str, "PID,%d,%d,%d,%s,%s,%s\n", (int)pitTemp, (int)fanSpeed, (int)setpoint, error, iterm, dinput);
      Serial.print(str);
      writeUSB(str);
    }
  }
}

void autoTune(boolean enable) {
  if (enable) {
    //Set the output to the desired starting frequency.
    pitTemp = aTuneStartValue;
    pidTuner.SetNoiseBand(aTuneNoise);
    pidTuner.SetOutputStep(aTuneStep);
    pidTuner.SetLookbackSec((int)aTuneLookBack);
    tuning = true;
  }
  else {
    pidTuner.Cancel();
    tuning = false;
  }
}

void sendSensorData() {
  unsigned long now = millis();
  unsigned long timeChange = (now - lastSensorTime);

  if (timeChange >= 2000) {
    lastSensorTime = now;

    char str[128];

    char control[128];
    pidStatus.toCharArray(control, pidStatus.length() + 1);

    sprintf(str, "SENSOR,%d,%d,%d,%d,%d,%s\n", (int)pitTemp, (int)food1, (int)food2, (int)fanSpeed, (int)setpoint, control);
    Serial.print(str);
    writeUSB(str);
  }
}



void readFromPhone() {
  if (!acc.isConnected())
    return;

  byte msg[128];

  int len = acc.read(msg, sizeof(msg), 1);

  if (len > 0) {
    // assumes only one command per packet
    Serial.print(len);

    String cmd = String((char*)msg);

    Serial.print(" received: '");
    Serial.print(cmd);
    Serial.println("'");

    cmd.toLowerCase();

    if (cmd.startsWith("setpoint")) {
      setpoint = getInt(cmd.substring(cmd.indexOf("=") + 1, cmd.length()));

      Serial.print("setpoint is: ");
      Serial.println(setpoint);
    }
    else if (cmd.startsWith("pidconstants")) {
      String vals  = cmd.substring(cmd.indexOf("=") + 1, cmd.length());
      char *k;
      char *p;

      char buf[vals.length()];
      vals.toCharArray(buf, vals.length());
      char *thebuf = buf;

      kp = atof(strtok_r(thebuf, ",", &p));
      ki = atof(strtok_r(NULL, ",", &p));
      kd = atof(strtok_r(NULL, ",", &p));

      Serial.println("setTunings (kp)...");
      Serial.print("kp: "); Serial.println(kp);
      Serial.print("ki: "); Serial.println(ki);
      Serial.print("kd: "); Serial.println(kd);

      pid.SetTunings(kp, ki, kd);
    }
    else if (cmd.startsWith("pid")) {
      String pidval = cmd.substring(cmd.indexOf("=") + 1, cmd.length());

      if (pidval.equals("on")) {
        pid.SetMode(AUTOMATIC);
        pidStatus = "AUTO";
        Serial.println("pid ON");
      }
      else {
        pid.SetMode(MANUAL);
        pidStatus = "MANUAL";
        Serial.println("pid OFF");
      }
    }
    else if (cmd.startsWith("autotune")) {
      String val = cmd.substring(cmd.indexOf("=") + 1, cmd.length());
      Serial.println("val is: "); Serial.println(val);
      if (val.equals("on")) {
        autoTune(true);
        Serial.println("autoTune ON");
      }
      else {
        autoTune(false);
        Serial.println("autoTune OFF");
      }

    }
    else if (cmd.startsWith("fan")) {
      int speed = getInt(cmd.substring(cmd.indexOf("=") + 1, cmd.length()));
      pid.SetMode(MANUAL);
      pidStatus = "MANUAL";
      fanSpeed = speed;
      Serial.print("fan speed set to: ");
      Serial.println(fanSpeed);
    }

  }
}


void writeUSB(char *str) {
  if (acc.isConnected()) {
    String s = String(str);
    int len = s.length();
    Serial.print("write output to usb: ");
    Serial.println(len);
    acc.write(str, len);
  }
}
void getTemps() {
  unsigned long now = millis();
  unsigned long timeChange = (now - lastTime);

  if (timeChange >= 1000) {
    lastTime = now;
    analogWrite(3, fanSpeed);

    pitTemp = readAndOverSample(3, 5.36924e-4, 1.91396e-4, 6.60399e-8, 10000);
    food1 = readAndOverSample(4, 5.36924e-4, 1.91396e-4, 6.60399e-8, 10000);
    food2 = readAndOverSample(5, 5.36924e-4, 1.91396e-4, 6.60399e-8, 10000);
  }
}



int readAndOverSample(int apin, double A, double B, double C, int resistor) {

  int sensorvalue = 0;
  unsigned int accumulated = 0;
  int numsamples = 64; // to get 3 more bits of precision from 10 to 13, 2^(2*3) = 64 samples
  int n = 0;

  sensorvalue = analogRead(apin);
  delay(2);

  // take 64 samples
  for (int i = 0; i < numsamples; i++) {
    sensorvalue = analogRead(apin);
    // Serial.print(sensorvalue); Serial.print(" ");
    if (sensorvalue == 0 || sensorvalue >= 1023) {
      return -1;
    }
    accumulated += sensorvalue;
    n++;

    // wait 2 milliseconds before the next loop
    // for the analog-to-digital converter to settle
    // after the last reading:
    delay(2);
  }

  // https://github.com/CapnBry/HeaterMeter/blob/c05dc0c39672f12aaf25314c2dfe46a51fb3535d/arduino/heatermeter/grillpid.cpp
  unsigned int ADCmax = (1 << 13) - 1;
  unsigned int ADCval = accumulated >> 3;
  float R, T;

  R = log( resistor / ((ADCmax / (float)ADCval) - 1.0f));
  T = 1.0f / ((C * R * R + B) * R + A);

  float celcius = T - 273.15f;
  float temp = ((celcius) * (9.0f / 5.0f)) + 32.0f;

  /*
    Serial.print("pin: ");
    Serial.println(apin);
    Serial.print("initial read: ");
    Serial.println(sensorvalue);
    Serial.print("accumulated=");
    Serial.print(accumulated);
    Serial.print(", ");
    Serial.print(n);
    Serial.print("x");
    Serial.println();
    Serial.print("R=");
    Serial.println(R);
    Serial.print("T=");
    Serial.println(T);
    Serial.print("ADCmax=");
    Serial.print(ADCmax);
    Serial.print(", ");
    Serial.print("ADCval=");
    Serial.print(ADCval);
    Serial.println();
    Serial.print("temp=");
    Serial.print(celcius);
    Serial.println(" C");
    Serial.print("temp=");
    Serial.print(temp);
    Serial.println(" F");
    Serial.println();
  */

  return temp;
}


double getDouble(String sval) {
  char cval[sval.length() + 1];
  sval.toCharArray(cval, sizeof(cval));
  return atof(cval);
}

int getInt(String sval) {
  char cval[sval.length() + 1];
  sval.toCharArray(cval, sizeof(cval));
  return atoi(cval);
}

char* formatDouble(double d) {
  char s[50];
  fmtDouble(d, 2, s, 0xffff);

  Serial.print("s: "); Serial.println(s);
  return s;
}

//
// Produce a formatted string in a buffer corresponding to the value provided.
// If the 'width' parameter is non-zero, the value will be padded with leading
// zeroes to achieve the specified width.  The number of characters added to
// the buffer (not including the null termination) is returned.
//
unsigned
fmtUnsigned(unsigned long val, char *buf, unsigned bufLen, byte width)
{
  if (!buf || !bufLen)
    return (0);

  // produce the digit string (backwards in the digit buffer)
  char dbuf[10];
  unsigned idx = 0;
  while (idx < sizeof(dbuf))
  {
    dbuf[idx++] = (val % 10) + '0';
    if ((val /= 10) == 0)
      break;
  }

  // copy the optional leading zeroes and digits to the target buffer
  unsigned len = 0;
  byte padding = (width > idx) ? width - idx : 0;
  char c = '0';
  while ((--bufLen > 0) && (idx || padding))
  {
    if (padding)
      padding--;
    else
      c = dbuf[--idx];
    *buf++ = c;
    len++;
  }

  // add the null termination
  *buf = '\0';
  return (len);
}

//
// Format a floating point value with number of decimal places.
// The 'precision' parameter is a number from 0 to 6 indicating the desired decimal places.
// The 'buf' parameter points to a buffer to receive the formatted string.  This must be
// sufficiently large to contain the resulting string.  The buffer's length may be
// optionally specified.  If it is given, the maximum length of the generated string
// will be one less than the specified value.
//
// example: fmtDouble(3.1415, 2, buf); // produces 3.14 (two decimal places)
//
void
fmtDouble(double val, byte precision, char *buf, unsigned bufLen)
{
  if (!buf || !bufLen)
    return;

  // limit the precision to the maximum allowed value
  const byte maxPrecision = 6;
  if (precision > maxPrecision)
    precision = maxPrecision;

  if (--bufLen > 0)
  {
    // check for a negative value
    if (val < 0.0)
    {
      val = -val;
      *buf = '-';
      bufLen--;
    }

    // compute the rounding factor and fractional multiplier
    double roundingFactor = 0.5;
    unsigned long mult = 1;
    for (byte i = 0; i < precision; i++)
    {
      roundingFactor /= 10.0;
      mult *= 10;
    }

    if (bufLen > 0)
    {
      // apply the rounding factor
      val += roundingFactor;

      // add the integral portion to the buffer
      unsigned len = fmtUnsigned((unsigned long)val, buf, bufLen);
      buf += len;
      bufLen -= len;
    }

    // handle the fractional portion
    if ((precision > 0) && (bufLen > 0))
    {
      *buf++ = '.';
      if (--bufLen > 0)
        buf += fmtUnsigned((unsigned long)((val - (unsigned long)val) * mult), buf, bufLen, precision);
    }
  }

  // null-terminate the string
  *buf = '\0';
}



