#include "Arduino.h"
#include "GPS_Air530Z1.h"
#include <TimeLib.h>

static String calchecksum(String cmd)
{
  uint8_t checksum=cmd[1];
  char temp[5];
  for(int i=2;i<cmd.length();i++)
  {
    checksum^=cmd[i];
  }
  memset(temp,0,5);
  sprintf(temp, "*%02X\r\n", checksum);
  cmd += temp;
  return cmd;
}

Air530Z1Class::Air530Z1Class(uint8_t powerCtl,uint8_t modePin) 
  :_powerCtl(powerCtl),_modePin(modePin)
  {}

void Air530Z1Class::begin()
{
  uint16_t baud = 9600;
  pinMode(_powerCtl,OUTPUT);
  digitalWrite(_powerCtl, LOW);

  GPSSerial.begin(baud);

  _baud = baud;
  Serial.print("GPS baudrate updated to ");
  Serial.println(baud);

  
  delay(10);
  setmode(MODE_GPS_BEIDOU_GLONASS);
  delay(10);
  setNMEA(NMEA_GSV|NMEA_GGA|NMEA_GSA|NMEA_RMC|NMEA_VTG|NMEA_GLL);
  delay(10);
}

void Air530Z1Class::setmode(GPSMODE mode)
{
  String cmd="$PCAS04,7";
  switch(mode)
  {
    case MODE_GPS_BEIDOU:
      cmd[8] = '3';
      break;
    case MODE_GPS_GLONASS:
      cmd[8] = '5';
      break;
    case MODE_GPS_BEIDOU_GLONASS:
      cmd[8] = '7';
      break;
    case MODE_GPS:
      cmd[8] = '1';
      break;
    case MODE_BEIDOU:
      cmd[8] = '2';
      break;
    case MODE_GLONASS:
      cmd[8] = '4';
      break;
    default:
      cmd[8] = '7';
      break;
  }
  cmd = calchecksum(cmd);
  sendcmd(cmd);
}

void Air530Z1Class::setNMEA(uint8_t nmeamode)
{
  String cmd = "$PCAS03,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0";
  if(nmeamode & NMEA_GGA)
  {
    cmd[8] = '1';
  }
  if(nmeamode & NMEA_GLL)
  {
    cmd[10] = '1';
  }
  if(nmeamode & NMEA_GSA)
  {
    cmd[12] = '1';
  }
  if(nmeamode & NMEA_GSV)
  {
    cmd[14] = '1';
  }
  if(nmeamode & NMEA_RMC)
  {
    cmd[16] = '1';
  }
  if(nmeamode & NMEA_VTG)
  {
    cmd[18] = '1';
  }
  if(nmeamode & NMEA_GST)
  {
    cmd[36] = '1';
  }

  cmd = calchecksum(cmd);
  
  sendcmd(cmd);
}

void Air530Z1Class::setBaud(uint32_t baud)
{
  String cmd = "$PCAS01,1";
  switch(baud)
  {
    case 9600:
      cmd[8] = '1';
      break;
    case 19200:
      cmd[8] = '2';
      break;
    case 38400:
      cmd[8] = '3';
      break;
    case 57600:
      cmd[8] = '4';
      break;
    case 115200:
      cmd[8] = '5';
      break;
    default:
      cmd[8] = '1';
      break;
  }
  cmd = calchecksum(cmd);
  sendcmd(cmd);
}

int Air530Z1Class::available(void)
{
  return GPSSerial.available();
}

int Air530Z1Class::read(void)
{
  return GPSSerial.read();
}


void Air530Z1Class::end()
{
  GPSSerial.flush();
  digitalWrite(_powerCtl, HIGH);
  GPSSerial.end();
}

void Air530Z1Class::sendcmd(String cmd)
{

  while(GPSSerial.available())//wait for gps serial idel
  {
    GPSSerial.readStringUntil('\r');
  }
  GPSSerial.print(cmd);
}
