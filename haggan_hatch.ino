//A4 hum ionizer relay pin
//A2 STH15 data pin
//A3 STH15 clock pin
//5     ONE wire bus
//A5 DHT22 data pin
//7 over heat fan 
//8 Heat relay 
//6 Servo pin for egg turning

//A1 HIH4030
//ethenet use 10,11,12,13 or 50,51,52 and pin 4 and 53


#include "SPI.h"
#include "Ethernet.h"
#define WEBDUINO_FAVICON_DATA ""
#include "WebServer.h"
#include <Time.h>
#include <Timezone.h>
#include <Servo.h> 
#include <FlexiTimer2.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include <HIH4030.h>
#include <SHT1x.h>
#include "DHT.h"

Servo myservo;
int Servopin=6;
int pos = 93;    // variable to store the servo position 
int sign =1;
unsigned long lastMillis,lastMillis2;
double Setpoint, Input,Output;
double humSetpoint, humInput,humOutput;
float humidity;
double underKp=2546, underKi=0, underKd=0;
double consKp=350, consKi=0.35, consKd=0;
//double consKp=350, consKi=0.35, consKd=0;
//double consKp=2546, consKi=97.09, consKd=0;
//double consKp=3703, consKi=182, consKd=5383;

//double consKp=2469.31, consKi=3133.83, consKd=1056.93;
//2546.48 ki: 88.83
//2546.48 ki: 97.09 kd: 0.00
P(leftbracket) ="[";
  P(rightbracket) ="]";
  P(semi) =",";

PID myPID(&Input, &Output, &Setpoint,underKp,underKi,underKd, DIRECT);
PID humPID(&humInput, &humOutput, &humSetpoint,underKp,underKi,underKd, DIRECT);

#define PIN_HIH4030 A1
#define ONE_WIRE_BUS 5
#define TEMPERATURE_PRECISION 12
//#define filterSamples   20
#define DHTPIN A5  
#define DHTTYPE DHT22   
DHT dht(DHTPIN, DHTTYPE);

// Specify data and clock connections and instantiate SHT1x object
#define dataPin  A2
#define clockPin A3
SHT1x sht1x(dataPin, clockPin);

#define RelayPin 8
#define humRelayPin A4
#define FanPin 7
int WindowSize = 4500;
unsigned long windowStartTime;

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

/*
// Kalman Temp filter setup
float P = 1.0;
float varP = pow(0.01, 2);
float varM = pow(0.5, 2);
float K = 1.0;


// Kalman Hum filter setup
float Phum = 1.0;
float varPhum = pow(0.01, 2);
float varMhum = pow(0.5, 2);
float Khum = 1.0;
float Kalmanhum = 20.0;
*/

/*
/* CHANGE THIS TO YOUR OWN UNIQUE VALUE.  The MAC number should be
 * different from any other devices on your network or you'll have
 * problems receiving packets. */
static uint8_t mac[] = { 
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

unsigned int localPort = 8888;      // local port to listen for UDP packets

//IPAddress timeServer(193, 149, 10, 65); // time.nist.gov NTP server
IPAddress timeServer(192, 168, 1, 1); // local NTP server

const int NTP_PACKET_SIZE= 48; // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets 

// A UDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

//time_t prevDisplay = 0; // when the digital clock was displayed
//Central European Time (Frankfurt, Borlänge)
TimeChangeRule CEST = {
  "CEST", Last, Sun, Mar, 2, 120};     //Central European Summer Time
TimeChangeRule CET = {
  "CET ", Last, Sun, Oct, 3, 60};       //Central European Standard Time
Timezone CE(CEST, CET);



/* CHANGE THIS TO MATCH YOUR HOST NETWORK.  Most home networks are in
 * the 192.168.0.XXX or 192.168.1.XXX subrange.  Pick an address
 * that's not in use and isn't going to be automatically allocated by
 * DHCP from your router. */
//static uint8_t ip[] = { 192, 168, 1, 210 };

/* This creates an instance of the webserver.  By specifying a prefix
 * of "", all pages will be at the root of the server. */
#define PREFIX ""
WebServer webserver(PREFIX, 80);

#define NAMELEN 32
#define VALUELEN 32

void setCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  URLPARAM_RESULT rc;
  char name[NAMELEN];
  char value[VALUELEN];
  double webP,webI,webD;
  webP=myPID.GetKp();
  webI=myPID.GetKi();
  webD=myPID.GetKd();
  P(Page_start) = "<html><head><title>HAGGAN HATCHER INPUT</title></head><body>\n";
  P(Page_end) = "</body></html>";
  P(Params_end) = "End of parameters<br>\n";
  P(Parsed_item_separator) = " = '";
  server.httpSuccess();
  if (type == WebServer::HEAD)
    return;
   server.printP(Page_start);
   
   if (strlen(url_tail))
  {
     while (strlen(url_tail))
    {
      rc = server.nextURLparam(&url_tail, name, NAMELEN, value, VALUELEN);
    }
  }
}
/*
void parsedCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  URLPARAM_RESULT rc;
  char name[NAMELEN];
  char value[VALUELEN];
  double webP,webI,webD;
  webP=myPID.GetKp();
  webI=myPID.GetKi();
  webD=myPID.GetKd();

  P(Page_start) = "<html><head><title>HAGGAN HATCHER INPUT</title></head><body>\n";
  P(Page_end) = "</body></html>";
  P(Params_end) = "End of parameters<br>\n";
  P(Parsed_item_separator) = " = '";
  server.httpSuccess();

  if (type == WebServer::HEAD)
    return;

  server.printP(Page_start);

  if (strlen(url_tail))
  {
    while (strlen(url_tail))
    {
      rc = server.nextURLparam(&url_tail, name, NAMELEN, value, VALUELEN);
      if (rc == URLPARAM_EOS)
        server.printP(Params_end);
      else
      {
      //  server.print(name);
      //  server.printP(Parsed_item_separator);
      //  server.print(value);
        if(name == "MODE")myPID.SetMode(int(value));
        if(name == "OUTPUT")Output=strtod(value,NULL);
        if(name == "P")webP=strtod(value,NULL);
        if(name == "I")webI=strtod(value,NULL);
        if(name == "D")webD=strtod(value,NULL);
        
      }
    }
  }
  myPID.SetTunings(webP, webI, webD);

  server.printP(leftbracket);
  server.print(Output);
  server.printP(semi);
  server.print(myPID.GetMode());
  server.printP(semi);
  server.print(myPID.GetKp());
  server.printP(semi);
  server.print(myPID.GetKi());
  server.printP(semi);
  server.print(myPID.GetKd());
  server.printP(rightbracket);

  server.printP(Page_end);
  
  


}

*/
/* commands are functions that get called by the webserver framework
 * they can read any posted data from client, and they output to the
 * server to send data back to the web browser. */
void sendTempHum(WebServer &server, WebServer::ConnectionType type, char *, bool)
{

  server.httpSuccess("Content-type: text/json");

  

  if (type != WebServer::HEAD)
  {
    server.printP(leftbracket);
    server.print(now());
    server.print("000");
    server.printP(semi);
    server.print(Input); 
    server.printP(semi);
    server.print(humidity);
    server.printP(semi);
    server.print(Output);
    server.printP(rightbracket);
  }

}

void changeHum(WebServer &server, WebServer::ConnectionType type, char *, bool)
{

  server.httpSuccess("Content-type: text/json");

  

  if (type != WebServer::HEAD)
  {
    if(humSetpoint==68)humSetpoint=50;
    else if(humSetpoint==50)humSetpoint=68;
    server.printP(leftbracket);
    server.print(now());
    server.print("000");
    server.printP(semi);
    server.print(humInput); 
    server.printP(semi);
    server.print(humOutput);
    server.printP(semi);
    server.print(humSetpoint);
    server.printP(rightbracket);
  }



}


void helloCmd(WebServer &server, WebServer::ConnectionType type, char *, bool)
{
  server.httpSuccess();
  if (type != WebServer::HEAD)
  {
    
    P(helloMsg)=
      "<!DOCTYPE html><html><head><meta http-equiv=\"content-type\" content=\"text/html; charset=UTF-8\">"
      "<title>Haggans Egg Hatcher</title>"
      "<script type='text/javascript' src='http://192.168.1.219/jquery-2.1.0.js'></script>"
      "<script src=\"http://192.168.1.219/highcharts.js\"></script>"
      "<script src=\"http://192.168.1.219/egg.js\"></script>"
      "</head><body><h1>Haggans Egg hatcher </h1><div id=\"container\" style=\"height: 400px\"></div></body></html>";
    server.printP(helloMsg);
  }
}

void setup()
{

  Serial.begin(9600);
  myservo.attach(Servopin);  // attaches the servo on pin 6 to the servo object 
  myservo.write(93);
  pinMode(FanPin, OUTPUT);
  //relay card works HIGH is off and ground is on
  digitalWrite(FanPin,HIGH); 
  pinMode(RelayPin, OUTPUT); 
  pinMode(humRelayPin, OUTPUT); 
  digitalWrite(humRelayPin,HIGH); 
  
  Setpoint = 37.5;
  humSetpoint = 68;
  lastMillis = millis(); 
  //HUM
  HIH4030::setup(PIN_HIH4030);
  //Temp sensor
  sensors.begin();
  sensors.requestTemperatures();
  sensors.setResolution(TEMPERATURE_PRECISION);
  dht.begin();
  Input = sensors.getTempCByIndex(0);
  humInput = dht.readHumidity();

  /* initialize the Ethernet adapter */
  // start Ethernet and UDP
  if (Ethernet.begin(mac) == 0) {
    //  Serial.println("Failed to configure Ethernet using DHCP");
    // no point in carrying on, so do nothing forevermore:
    for(;;)
      ;
  }
  
  Udp.begin(localPort);

  //Serial.println("waiting for sync");
  setSyncProvider(getNtpTime);
  while(timeStatus()== timeNotSet)   
    ; // wait until the time is set by the sync provider



  windowStartTime = millis();


  /* setup our default command that will be run when the user accesses
   * the root page on the server */
  webserver.setDefaultCommand(&helloCmd);
  //webserver.addCommand("set", &setCmd);

  /* run the same command if you try to load /index.html, a common
   * default page name */
  webserver.addCommand("live.php", &sendTempHum);
  webserver.addCommand("changehum", &changeHum);

  /* start the webserver */
  webserver.begin();
  //  myPID.SetSampleTime(2000);
  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, 1500);
  humPID.SetOutputLimits(200, 4500);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  humPID.SetMode(AUTOMATIC);
//  myPID.SetMode(MANUAL);
  FlexiTimer2::set(10,1.0/1000, relayInterupt); //Interrupt every 1/1000 milisec
  FlexiTimer2::start();
}

void loop()
{
  float shttemp;
  float shthum;
  float dhttemp;
  float dhthum;
  float hihhum;
  float dalltemp;
  float avrt=0;
  float avrh=0;
  int avnum=0;
  int avnumhum=0;
  
  if(Input>32)myPID.SetOutputLimits(0, 1000);
  if(Input<32)myPID.SetOutputLimits(0, 1500);

//  if(Input>37)myPID.SetOutputLimits(0, 500);

  if(Output < millis() - windowStartTime) 
  {
    sensors.requestTemperatures(); // Send the command to get temperatures
    //digitalSmooth(sensors.getTempCByIndex(0), tempSmoothArray);     
    //Input = digitalSmooth(sensors.getTempCByIndex(1), tempSmoothArray); 
    FlexiTimer2::stop();
    dalltemp = sensors.getTempCByIndex(0); 
//    shttemp=sht1x.readTemperatureC();
  //  shthum=sht1x.readHumidity();
    dhttemp=dht.readTemperature();
    dhthum=dht.readHumidity();

    FlexiTimer2::start();

    
    if (!isnan(dhttemp) && dhttemp!=0.00)
      {
      avrt=avrt+dhttemp;
      avrh=avrh+dhthum;
      avnum++;
      avnumhum++;
      }
/*
    if (shttemp>-39)
        {
        avrt=avrt+shttemp;
        avrh=avrh+shthum;
        avnum++; 
        avnumhum++;
        }
*/
    if(dalltemp!=85 && dalltemp!=-127)
       {
       avrt=avrt+dalltemp; 
       avnum++;
       }
       //temp avrage
       Input=avrt/avnum;
       //Kalman filter process for temp
       //P = P + varP;
       //K = P / (P + varM);    
       //Input = K * avrt + (1 - K) * Input;
       //P = (1 - K) * P;
       
    //   hihhum=HIH4030::read(PIN_HIH4030, Input);
  /*     if(hihhum>0 && hihhum<101)
        {
         avrh=avrh+hihhum;
         avnumhum++; 
        }
*/
        //hum Avrage
        humidity=avrh/avnumhum;
        humInput=humidity;
        Serial.print("TEMP: ");
//	Serial.print(shttemp, DEC);        
  //      Serial.print(",");
	Serial.print(dhttemp, DEC);        
        Serial.print(",");
	Serial.print(dalltemp, DEC);        
        Serial.print(",");
	Serial.print(Input, DEC);
        Serial.print(",");
	Serial.println(Output, DEC);

        Serial.print("HUMM: ");
//	Serial.print(shthum, DEC);        
  //      Serial.print(",");
	Serial.print(dhthum, DEC);        
  //      Serial.print(",");
//	Serial.print(hihhum, DEC);        
        Serial.print(",");
	Serial.println(humidity, DEC);
        

  }
/*
if (millis() - lastMillis2 > 120000)
  {
    lastMillis2 = millis();
    if(Output==1000)Output=0;
    else Output=1000;
    
  }
  */

  if (millis() - lastMillis > 2000 && sign!= 0)
  {
    lastMillis = millis();
    myservo.write(pos+=sign);
    //Serial.println(pos);
    if(pos == 138 || pos == 23)
    {
      sign=0;
      myservo.detach();
    }
  }
  //TURN EGG   
  if(minute() == 15 || minute() == 30 || minute() == 45 || minute() == 00 )
  { 
    //Serial.println(minute());
    if(pos == 138) 
    {
      sign=-1;
      myservo.attach(Servopin);
    }
    if(pos == 23)
    {
      sign=1;
      myservo.attach(Servopin);
    }
  }    

  //HEAT SAFETY
  if(Input>41) digitalWrite(RelayPin,LOW);
  if(Input>34 && Input<40)myPID.SetTunings(consKp, consKi, consKd);
  if(Input>40){
    myPID.SetTunings(underKp, underKi, underKd);
    //LOW is = ON
    digitalWrite(FanPin,LOW);
  }
  else digitalWrite(FanPin,HIGH);  
  //TurnEggs();
  //char buff[64];
  //int len = 64;

  /* process incoming connections one at a time forever */
  webserver.processConnection();
}

/*
void TurnEggs(){
 if(minute() == 15 || 30 || 45 || 00)
 { 
 if(pos == 140) sign=-1;
 if(pos == 25)  sign=1;
 }
 }
 */

void relayInterupt() {
  /************************************************
   * turn the output pin on/off based on pid output
   ************************************************/

  unsigned long myNow = millis(); 

  if(myNow - windowStartTime> WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }

  if(Output < myNow - windowStartTime) 
  {
    digitalWrite(RelayPin,LOW);
  }
  else if (Output >9 )
  {
    digitalWrite(RelayPin,HIGH);
  }

 if(humOutput < myNow - windowStartTime) 
  {
    digitalWrite(humRelayPin,HIGH);
  }
  else if (humOutput >200 )
  {
    digitalWrite(humRelayPin,LOW);
  }
 
  myPID.Compute();
  humPID.Compute();
  

}





/*-------- NTP code ----------*/

unsigned long getNtpTime()
{
  sendNTPpacket(timeServer); // send an NTP packet to a time server
  // wait to see if a reply is available
  delay(1000);
  if ( Udp.parsePacket() ) {
    // We've received a packet, read the data from it
    Udp.read(packetBuffer,NTP_PACKET_SIZE);  // read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;

    // now convert NTP time into Unix time
    // starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    unsigned long epoch = secsSince1900 - seventyYears;  
    return CE.toLocal(epoch);
  }
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address 
unsigned long sendNTPpacket(IPAddress& address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE); 
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49; 
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp: 		   
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer,NTP_PACKET_SIZE);
  Udp.endPacket(); 
}


