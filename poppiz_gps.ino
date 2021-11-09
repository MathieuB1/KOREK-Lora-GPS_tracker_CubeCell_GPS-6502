/* Heltec Automation
 *
 * Sketch on Heltec HTCC-AB02S
 * --> Get GPS and send data over Lora Local Node with AES encryption
 *
 * Scenario:
 * 1. Receive conf from Local Server see https://github.com/MathieuB1/KOREK-WifiLora-GPS_tracker
 * AES key + Name + Sleep Frequency
 * 2. Get GPS and Send it to Lora Node Local Server
 * 3. Sleep each 30 seconds for waiting a "po", then send a "pi"
 * 4. Trigger GPS until whistle timeout
 *
 * this project also realess in GitHub:
 * https://github.com/HelTecAutomation/ASR650x-Arduino
 * */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "GPS_Air530Z.h"
#include <EEPROM.h>
#include "aes.h"


// Lora
#define RF_FREQUENCY                                868000000 // Hz

#define TX_OUTPUT_POWER                             14        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       9         // [SF7..SF12]
#define LORA_CODINGRATE                             4         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 1024 // Define the payload size here

#define ONE_MINUTE 60000

// Sleep Config
#define timetillsleep ONE_MINUTE *3 // wake up each 3 minutes
#define timetillwakeup ONE_MINUTE *3

// Internal AES
#define ECB 1
#define AES128 1
#define AES_SIZE 16
#define AES_KEY_LENGHT 4
#define DEFAULT_AES_KEY "1234123412341234" // when EEPROM conf is empty

//if GPS module is Air530Z, use this
#define GPS_TIMEOUT 32000*2 // Air530Z spec says 32 seconds for cold acquisition
#define GPS_SLEEP_BEFORE_NEXT 6000 // wait 6 seconds for next capture
Air530ZClass GPS;

// Whistle
#define WHISTLE_PERIOD 180000 // send gps during 3 mins

// Lora
char decrypted[BUFFER_SIZE];
char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

struct AES_ctx ctx, ctxInit;

// The real key is set by the Server at init
uint8_t AppKey1[] = DEFAULT_AES_KEY ;

typedef struct {
   char title[256];
   int frequency;
   char aes[AES_KEY_LENGHT];
} clientConf;

clientConf myLocalConf;

static RadioEvents_t RadioEvents;
double txNumber;
int16_t rssi,rxSize;

unsigned long elaspsed_time = 0;

//Some utilities for going into low power mode
static TimerEvent_t sleep;
static TimerEvent_t wakeUp;


bool confSetup = false;
bool resetConf=false;
bool whistle_mode = false;
bool lowpower=true;
bool triggerGps = true;


void setup() {
    boardInitMcu();
    Serial.begin(115200);
    EEPROM.begin(512);
    txNumber=0;
    rssi=0;

    RadioEvents.RxDone = OnRxDone;

    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetSyncWord( 0x39 );
    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                       LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                       LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                       0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

    memset(decrypted, 0, sizeof(decrypted));
    memset(txpacket, 0, sizeof(txpacket));

    // Try loading and create Ctx for the AES key
    // Otherwise use the default one
    setAesKeyCtx();

      // Give a time slot when pressing the User Button
    pinMode(USER_KEY,INPUT);
    Serial.println("waiting for User key press");
    delay(200);
    Serial.println("end User key press");
    int rstPinValue = digitalRead(USER_KEY);
    if (!rstPinValue) {
      resetConf=true;
      Serial.println("Reset the conf...");
    }
    
    TimerInit( &sleep, onSleep );
    TimerInit( &wakeUp, onWakeUp );

}


void loop()
{

  unsigned long elaspsed_loop_time = millis();
  
  // Wait for Lora Signal
  Radio.Rx( 0 );
  Radio.IrqProcess( );
  // Wait for message
  delay(2500);
  Radio.Sleep( );

  // Get battery level:
  uint8_t batt_percent = getBatteryLevel();
  if(batt_percent < 25)
  {
    char message[8] = "low-";
    strcat(message,(char*)batt_percent);
    encryptPayload(message, txpacket, ctx);
    sendingMessage(txpacket);
  }
    
  // User button pressed after reboot
  if(resetConf) {
    triggerGps = false;
    lowpower = false;
    resetConfig();
  }

 // If we receive a whistle request
  if(whistle_mode){ 
    lowpower = false;
    triggerGps = true;
    if(elaspsed_time > WHISTLE_PERIOD) {
      elaspsed_time = 0;
      whistle_mode=false;
      lowpower = true;
    }
  }

  if(triggerGps) {
    lowpower = false;
    bool gps_signal = false;
    
    Serial.println("Activating GPS...");
    
      // Init GPS
      GPS.begin(115200);


        uint32_t starttime = millis();
        while( (millis()-starttime) < GPS_TIMEOUT )
        {
          while (GPS.available() > 0)
          {
            GPS.encode(GPS.read());
          }

          // gps fixed in a second
          if( GPS.location.age() < 1000 && GPS.date.year() != 2000 && (int)GPS.hdop.hdop() > 0 && (int)GPS.hdop.hdop() < 3 )
          {
            gps_signal = true;
            break;
          }
        }
        
      GPS.end();
      Serial.println("Stopping GPS...");
        
      String battStr;
      battStr = String(batt_percent);
      char batt[3];
      battStr.toCharArray(batt,3);
        
      if (gps_signal)
      {

        Serial.println("Got gps signal");
        Serial.printf("%d/%02d/%02d",GPS.date.year(),GPS.date.day(),GPS.date.month());

         // Preparing the message to send
        String dateStr, tmpdate, tmpmonth, latStr, lonStr, precisionStr;

        tmpdate = (String(GPS.date.day()).length() == 1) ? "0" + String(GPS.date.day()) : String(GPS.date.day());
        tmpmonth = (String(GPS.date.month()).length() == 1) ? "0" + String(GPS.date.month()) :  String(GPS.date.month());
        
        dateStr += tmpdate + tmpmonth + String(GPS.date.year())[2] + String(GPS.date.year())[3];
        latStr += String(GPS.location.lat(),8);
        lonStr += String(GPS.location.lng(),8);
        precisionStr += String(GPS.hdop.hdop(),4);
        
        char date[7];
        dateStr.toCharArray(date,7);
        char lat[10];
        latStr.toCharArray(lat,8);
        char lon[10];
        lonStr.toCharArray(lon,8);
        char precision[4];
        precisionStr.toCharArray(precision,4);


        // gps = {"title": "poppiz", "lat":7.101813, "lon":43.58843, "date": "300919", "batt": 86, "precision":3.0}
        
        char message[1024] = "";
        dataToSendToServer(lat,lon,date,batt,precision,message);

        // Sending the JSON message
        encryptPayload(message, txpacket, ctx);
        sendingMessage(txpacket);
        delay(GPS_SLEEP_BEFORE_NEXT);

      } else {
        Serial.println("GPS failed!");
        // Do not send message, saving battery...
      }

    // Low power mode
    if(!whistle_mode) { 
      triggerGps = false;
      lowpower = true;
    }

  }

  elaspsed_time += millis() - elaspsed_loop_time;
  Serial.printf("\nElaspsed time: %d\n", elaspsed_time);   
  
  if(lowpower){
    onSleep();
    lowPowerHandler();
  }

}


void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    rssi=rssi;
    rxSize=size;
    Radio.Sleep( );

    Serial.println("Lora packet received!");

    // Because the payload is not cleared each time :-(
    memcpy(rxpacket, payload, size );
    // Add end string at the received message!
    rxpacket[size]='\0';

    // Receiving a whistle from Server
    decryptPayload((uint8_t*)rxpacket, decrypted, ctx);
    if (strcmp(decrypted, "po") == 0)
    {
        Serial.println("whistle received!");
        encryptPayload("pi", txpacket, ctx);
        whistle_mode = true;
        elaspsed_time = 0;
        sendingMessage("pi");
    }

    // If EEPROM Conf already setup, we ignore this step
    if (!confSetup) {
        // Receiving the configuration from Server
        AES_init_ctx(&ctxInit, AppKey1);
        decryptPayload((uint8_t*)rxpacket, decrypted, ctxInit);
        if (String(decrypted).startsWith("init"))
        {
          Serial.println("\r\nServer conf received!");
          char toStoreInMem[512] = {};
          memset(toStoreInMem,0,sizeof(toStoreInMem));
          memcpy(toStoreInMem, (char*)decrypted+5, strlen(decrypted)-5);
    
          writeStringToEEPROM(0, String(toStoreInMem));
          retrieveConfFromEEPROM();

          // Send confirmation to Server
          encryptPayload("init:ok", txpacket, ctxInit);
          sendingMessage(txpacket);
          // Let's set the AES key received
          setAesKeyCtx();
        }
    }
}

///////////////////////////////
// Lora Send
///////////////////////////////
void sendingMessage(char *toSend) {
    Serial.printf("\r\nsending packet \"%s\" , length %d\r\n", toSend, strlen(txpacket));
    // Repeat the message 2 times
    for (uint8_t i=0; i<2; ++i){
      Radio.Send( (uint8_t *)txpacket, strlen(txpacket) ); //send the package out
      delay(0.2);
    }

}

uint8_t getBatteryLevel() {
  // 5.5 End-Device Status (DevStatusReq, DevStatusAns)
  // 0      The end-device is connected to an external power source.
  // 1..254 The battery level, 1 being at minimum and 254 being at maximum
  // 255    The end-device was not able to measure the battery level.
  const double maxBattery = 4.20; //4.212;
  const double minBattery = 3.6;
  const double batVoltage = fmax(minBattery, fmin(maxBattery, getBatteryVoltage() / 1000.0));
  const uint8_t batlevel = (BAT_LEVEL_EMPTY + ((batVoltage - minBattery) / (maxBattery - minBattery)) * (BAT_LEVEL_FULL - BAT_LEVEL_EMPTY))*100/254;
  Serial.printf("Battery level (1-100): %u\n", batlevel);
  return batlevel;
}

void onSleep()
{
  Serial.printf("Going into lowpower mode, %d ms later wake up.\r\n",timetillwakeup);
  Radio.Sleep();
  lowpower=true;
  //timetillwakeup ms later wake up;
  TimerSetValue( &wakeUp, timetillwakeup );
  TimerStart( &wakeUp );
}

void onWakeUp()
{
  Serial.printf("Woke up, %d ms later into lowpower mode.\r\n",timetillsleep);
  lowpower=false;
  elaspsed_time += timetillwakeup;
  if (!whistle_mode && elaspsed_time < myLocalConf.frequency * 1000) 
  {
    Serial.printf("Going into lowpower mode at next loop.\r\n");
    triggerGps = false;
    lowpower=true;
  } else {
    elaspsed_time = 0;
    triggerGps = true;
  }

}

///////////////////////////////
// Reset
///////////////////////////////
void resetConfig() {
    clearEEPROM();
    memcpy(AppKey1, DEFAULT_AES_KEY, sizeof(AppKey1));
    Serial.println((char*)AppKey1);
    resetConf = false;
    confSetup = false;
    Serial.println("Conf successfully reset!");
}

//////////////////////////////////
// Sending data GPS Data to Server
//////////////////////////////////
void dataToSendToServer(char *lat, char *lon, char *date, char *batt, char *precision, char *message) {
    // Init JSON
    strcat(message, "{");
    strcat(message, "\"title\": \"");
    strcat(message, myLocalConf.title);
    strcat(message, "\", ");

    strcat(message, "\"lat\":");
    strcat(message, lat);
    strcat(message, ", ");

    strcat(message, "\"lon\":");
    strcat(message, lon);
    strcat(message, ", ");
    
    strcat(message, "\"batt\":");
    strcat(message, batt);
    strcat(message, ", ");
    
    strcat(message, "\"date\": \"");
    strcat(message, date);
    strcat(message, "\", ");

    strcat(message, "\"precision\":");
    strcat(message, precision);
    // End JSON
    strcat(message, "}");

    Serial.println("JSON to send:");
    Serial.println(message);
}
///////////////////////////////
// Client Conf Extractor
///////////////////////////////
void retrieveConf(char* localConf, clientConf* myLocalConf)
{
      // "title=" + title + "&frequency=" + str(frequency) + "&aes=" + aes_pass

      // Extract keys
      char *title = strtok(localConf, "&");
      char *frequency = strtok(NULL, "&");
      char *aes = strtok(NULL, "&");

      // Extract value from keys
      char *key = strtok(title, "=");
      title = strtok(NULL, "=");
      
      key = strtok(frequency, "=");
      frequency = strtok(NULL, "=");

      key = strtok(aes, "=");
      aes = strtok(NULL, "=");

      /* Set Conf */
      // Set the Title
      memcpy(myLocalConf->title, title, sizeof(*title)*strlen(title));
      // Set the Conf
      myLocalConf->frequency = String(frequency).toInt();
      // We only take 4 characters for AES key
      memset(myLocalConf->aes, 0, sizeof(*aes)*AES_KEY_LENGHT+1);
      memcpy(myLocalConf->aes, aes, sizeof(*aes)*AES_KEY_LENGHT);

      Serial.printf("\r\nlocal conf setup: \n title= \"%s\"\n frequency=\"%d\"\n aes=\"%s\"\n", \
      myLocalConf->title, myLocalConf->frequency, myLocalConf->aes);

}
///////////////////////////////
// EEPROM: 512 bytes
///////////////////////////////
void clearEEPROM() {
   // write a 0 to all 512 bytes of the EEPROM
    for (int i = 0; i < 512; i++) {
    EEPROM.write(i, 0);
   }
  EEPROM.commit();
}

void retrieveConfFromEEPROM() {
    Serial.println("Retreiving Conf...");
    String retrievedString = readStringFromEEPROM(0);
    char Buf[retrievedString.length()] = {};
    retrievedString.toCharArray(Buf, retrievedString.length());
    retrieveConf(Buf, &myLocalConf);
}

void writeStringToEEPROM(int addrOffset, const String &strToWrite)
{
  byte len = strToWrite.length();
  EEPROM.write(addrOffset, len);
  for (int i = 0; i < len; i++)
  {
    EEPROM.write(addrOffset + 1 + i, strToWrite[i]);
  }

  if (EEPROM.commit()) {
    Serial.println("EEPROM successfully committed");
  } else {
    Serial.println("ERROR! EEPROM commit failed");
  }
}

String readStringFromEEPROM(int addrOffset)
{
  int newStrLen = EEPROM.read(addrOffset);
  char data[newStrLen + 1];
  for (int i = 0; i < newStrLen; i++)
  {
    data[i] = EEPROM.read(addrOffset + 1 + i);
  }
  data[newStrLen] = '\0'; // the character may appear in a weird way, you should read: 'only one backslash and 0'
  return String(data);
}


///////////////////////////////
// AES Tools
///////////////////////////////
void setAesKeyCtx() {
  int first_mem_val = EEPROM.read(0);
  if (first_mem_val != 0)
  {
    retrieveConfFromEEPROM();
    setAesKey16b(&myLocalConf, AppKey1);
    Serial.println("Pending Association...");
    confSetup = true;
  } else {
    Serial.println("Pending Association...");
    confSetup = false;
  }

  Serial.println("\nUsing the following AES key:");
  Serial.println((char*)AppKey1);
  //https://github.com/kokke/tiny-AES-c
  AES_init_ctx(&ctx, AppKey1);
}

void setAesKey16b (clientConf* myLocalConf, uint8_t* AppKey){
      //We take only 4 chars for the AES key
    uint8_t buff[AES_SIZE+1] = {};
    memset(buff,0,sizeof(buff));
    char key[AES_KEY_LENGHT+1]= {};
    memset(key,0,sizeof(key));
    
    memcpy(key, myLocalConf->aes, sizeof(key));
    for (int i=0;i<AES_KEY_LENGHT;i++){
      strcat((char*)buff,key);
    }
    memcpy(AppKey,buff,sizeof(buff));
}

void decryptPayload(uint8_t *payload, char *decrypted, AES_ctx &ctx) {
      // Decode message
    uint8_t header_length = 4;
    uint8_t size_length = 16;

    memset(decrypted, 0, sizeof(decrypted));
    for (int i=0; i<(strlen((char*)payload)-header_length); i+=size_length)
    {
       uint8_t buff_decode[size_length+1] = {};
       memset(buff_decode, 0, sizeof(buff_decode));
       for(uint8_t j=0; j<16; j++){
          buff_decode[j]=payload[i+header_length+j];
       }
       AES_ECB_decrypt(&ctx, (uint8_t*)buff_decode);
       strcat(decrypted, (char*)buff_decode);
    }
    trim(decrypted);
    Serial.printf("\r\ndecrypted packet \"%s\" , length %d\r\n",decrypted, strlen(decrypted));
}


void encryptPayload(char *input, char *txpacket, AES_ctx &ctx) {
  
  uint8_t size_length = 16;
  int linecounter = 1;
  char message[strlen(input)+size_length] = {};

  memset(message, 0, sizeof(input)+size_length);
  strcat(message, input);
  
  // Set Lora Header
  uint8_t loraHeader[4] = { 0xff, 0x41, linecounter, strlen(input) };

  // Determine the lenght of padding to add
  uint8_t space_to_add = size_length-strlen(input)%size_length;

  // The final message must be a multiple of 16, so we set padding spaces
  uint8_t buff_to_add[size_length+1] = {};
  memset(buff_to_add, 0, sizeof(buff_to_add));
  memset(buff_to_add, ' ', space_to_add);
  strcat(message, (char*)buff_to_add);

  // Prepare chunks of 16 bytes for AES ECB encryption
  for (int i=0; i<(strlen(message)); i+=size_length)
  {   
    uint8_t buff_encode[size_length+1] = {};
    memcpy(buff_encode, (char*)message+i, size_length);
    AES_ECB_encrypt(&ctx,(uint8_t*)buff_encode);
    memcpy(txpacket+4+i,buff_encode,size_length);
  }
  memcpy(txpacket,loraHeader,4);
}

///////////////////////////////
// Utils
///////////////////////////////
char *trim (char *s)
{
    int i;
    while (isspace (*s)) s++;   // skip left side white spaces
    for (i = strlen (s) - 1; (isspace (s[i])); i--) ;   // skip right side white spaces
    s[i + 1] = '\0';
}