/*
 *  This sketch is running a web server for configuring WiFI if can't connect or for controlling of one GPIO to switch a light/LED
 *  Also it supports to change the state of the light via MQTT message and gives back the state after change.
 *  The push button has to switch to ground. It has following functions: Normal press less than 1 sec but more than 50ms-> Switch light. Restart press: 3 sec -> Restart the module. Reset press: 20 sec -> Clear the settings in EEPROM
 *  While a WiFi config is not set or can't connect:
 *    http://server_ip will give a config page with
 *  While a WiFi config is set:
 *    http://server_ip/gpio -> Will display the GIPIO state and a switch form for it
 *    http://server_ip/gpio?state_sw=0 -> Will change the GPIO13 to Low (Relay OFF)
 *    http://server_ip/gpio?state_sw=1 -> Will change the GPIO13 to High (Relay ON)
 *    http://server_ip/cleareeprom -> Will reset the WiFi setting and rest to configure mode as AP
 *  server_ip is the IP address of the ESP8266 module, will be
 *  printed to Serial when the module is connected. (most likly it will be 192.168.4.1)
 * To force AP config mode, press button 20 Secs!
 *  For several snippets used, the credit goes to:
 *  - https://github.com/esp8266
 *  - https://github.com/chriscook8/esp-arduino-apboot
 *  - https://github.com/knolleary/pubsubclient
 *  - https://github.com/vicatcu/pubsubclient <- Currently this needs to be used instead of the origin
 *  - https://gist.github.com/igrr/7f7e7973366fc01d6393
 *  - http://www.esp8266.com/viewforum.php?f=25
 *  - http://www.esp8266.com/viewtopic.php?f=29&t=2745
 *    http://www.esp8266.com/viewtopic.php?p=30065 Ajax auto update for html page By Manudax
 *  - And the whole Arduino and ESP8266 comunity
 */

#define DEBUG
//#define WEBOTA
//debug added for information, change this according your needs

#ifdef DEBUG
#define Debug(x)    Serial.print(x)
#define Debugln(x)  Serial.println(x)
#define Debugf(...) Serial.printf(__VA_ARGS__)
#define Debugflush  Serial.flush
#else
#define Debug(x)    {}
#define Debugln(x)  {}
#define Debugf(...) {}
#define Debugflush  {}
#endif


#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
//#include <EEPROM.h>
#include <Ticker.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "FS.h"

extern "C" {
#include "user_interface.h" //Needed for the reset command
}

//***** Settings declare *********************************************************************************************************
String hostName = "Armtronix"; //The MQTT ID -> MAC adress will be added to make it kind of unique
int iotMode = 0; //IOT mode: 0 = Web control, 1 = MQTT (No const since it can change during runtime)
//select GPIO's
#define OUTPIN 13 //output pin
#define INPIN 0  // input pin (push button)
#define PIR_INPIN 14  // input pin (Sensor)
#define SWITCH_INPIN 4  // input pin (push button)
#define OUTLED 12
#define RESTARTDELAY 3 //minimal time in sec for button press to reset
#define HUMANPRESSDELAY 50 // the delay in ms untill the press should be handled as a normal push by human. Button debounce. !!! Needs to be less than RESTARTDELAY & RESETDELAY!!!
#define RESETDELAY 20 //Minimal time in sec for button press to reset all settings and boot to config mode

//##### Object instances #####
MDNSResponder mdns;
ESP8266WebServer server(80);
WiFiClient wifiClient;
PubSubClient mqttClient;
Ticker btn_timer;
Ticker otaTickLoop;

//##### Flags ##### They are needed because the loop needs to continue and cant wait for long tasks!
int rstNeed = 0; // Restart needed to apply new settings
int toPub = 0; // determine if state should be published.
int configToClear = 0; // determine if config should be cleared.
int otaFlag = 0;
boolean inApMode = 0;
//##### Global vars #####
int webtypeGlob;
int otaCount = 300; //imeout in sec for OTA mode
int current; //Current state of the button
int switch_status; //Physical state of the switch
int state_sw ;
int send_status;
unsigned long count = 0; //Button press time counter
String st; //WiFi Stations HTML list
String state; //State of light
char buf[40]; //For MQTT data recieve
char* host; //The DNS hostname
String javaScript,XML;
//To be read from Config file
String esid = "";
String epass = "";
String pubTopic;
String subTopic;
String mqttServer = "";
const char* otaServerIndex = "<form method='POST' action='/update' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>";


//-------------------------------- Help functions ---------------------------


String macToStr(const uint8_t* mac)
{
  String result;
  for (int i = 3; i < 6; ++i) {
    result += String(mac[i], 16);
    if (i < 5)
      result += ':';
  }
  return result;
}


void otaCountown(){
    if(otaCount>0 && otaFlag==1) {
      otaCount--;
      Serial.println(otaCount); 
    }
}
#ifdef WEBOTA
void ota(){
   //Debugln("OTA."); 
   if (OTA.parsePacket()) {
    IPAddress remote = OTA.remoteIP();
    int cmd  = OTA.parseInt();
    int port = OTA.parseInt();
    int size   = OTA.parseInt();

    Serial.print("Update Start: ip:");
    Serial.print(remote);
    Serial.printf(", port:%d, size:%d\n", port, size);
    uint32_t startTime = millis();

    WiFiUDP::stopAll();

    if(!Update.begin(size)){
      Serial.println("Update Begin Error");
      return;
    }

    WiFiClient clientWiFi;
    if (clientWiFi.connect(remote, port)) {

      uint32_t written;
      while(!Update.isFinished()){
        written = Update.write(clientWiFi);
        if(written > 0) clientWiFi.print(written, DEC);
      }
      Serial.setDebugOutput(false);

      if(Update.end()){
        clientWiFi.println("OK");
        Serial.printf("Update Success: %u\nRebooting...\n", millis() - startTime);
        ESP.restart();
      } else {
        Update.printError(clientWiFi);
        Update.printError(Serial);
      }
    } else {
      Serial.printf("Connect Failed: %u\n", millis() - startTime);
    }
  }
  //IDE Monitor (connected to Serial)
  if (TelnetServer.hasClient()){
    if (!Telnet || !Telnet.connected()){
      if(Telnet) Telnet.stop();
      Telnet = TelnetServer.available();
    } else {
      WiFiClient toKill = TelnetServer.available();
      toKill.stop();
    }
  }
  if (Telnet && Telnet.connected() && Telnet.available()){
    while(Telnet.available())
      Serial.write(Telnet.read());
  }
  if(Serial.available()){
    size_t len = Serial.available();
    uint8_t * sbuf = (uint8_t *)malloc(len);
    Serial.readBytes(sbuf, len);
    if (Telnet && Telnet.connected()){
      Telnet.write((uint8_t *)sbuf, len);
      yield();
    }
    free(sbuf);
  }
}
#endif

bool loadConfig() {
  File configFile = SPIFFS.open("/config.json", "r");
  if (!configFile) {
    Serial.println("Failed to open config file");
    return false;
  }

  size_t size = configFile.size();
  if (size > 1024) {
    Serial.println("Config file size is too large");
    return false;
  }

  // Allocate a buffer to store contents of the file.
  std::unique_ptr<char[]> buf(new char[size]);

  // We don't use String here because ArduinoJson library requires the input
  // buffer to be mutable. If you don't use ArduinoJson, you may as well
  // use configFile.readString instead.
  configFile.readBytes(buf.get(), size);

  StaticJsonBuffer<400> jsonBuffer;
  JsonObject& json = jsonBuffer.parseObject(buf.get());

  if (!json.success()) {
    Serial.println("Failed to parse config file");
    return false;
  }

  int otaFlagC = json["otaFlag"];
  String esidC = json["esid"];
  String epassC = json["epass"];
  int iotModeC = json["iotMode"];
  String pubTopicC = json["pubTopic"];
  String subTopicC = json["subTopic"];
  String mqttServerC = json["mqttServer"];

  // Real world application would store these values in some variables for
  // later use.
  otaFlag = otaFlagC;
  esid = esidC;
  epass = epassC;
  iotMode = iotModeC;
  pubTopic = pubTopicC;
  subTopic = subTopicC;
  mqttServer = mqttServerC;
  Serial.print("otaFlag: "); 
  Serial.println(otaFlag);
  Serial.print("esid: ");
  Serial.println(esid);
  Serial.print("epass: ");
  Serial.println(epass);
  Serial.print("iotMode: ");
  Serial.println(iotMode);
  Serial.print("pubTopic: ");
  Serial.println(pubTopic);
  Serial.print("subTopic: ");
  Serial.println(subTopic);
  Serial.print("mqttServer: ");
  Serial.println(mqttServer);
  Serial.print("esid: ");
  Serial.println(esid);
  return true;
}

bool saveConfig() {
  StaticJsonBuffer<400> jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
  json["otaFlag"] = otaFlag;
  json["esid"] = esid;
  json["epass"] = epass;
  json["iotMode"] = iotMode;
  json["pubTopic"] = pubTopic;
  json["subTopic"] = subTopic;
  json["mqttServer"] = mqttServer;

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println("Failed to open config file for writing");
    return false;
  }

  json.printTo(configFile);
  return true;
}


void setOtaFlag(int intOta){
  otaFlag=intOta;
  saveConfig();
  yield();
}

bool clearConfig(){
    Debugln("DEBUG: In config clear!");
    return SPIFFS.format();  
}

//-------------- void's -------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(10);
  // prepare GPIO2
  pinMode(OUTPIN, OUTPUT);
  pinMode(OUTLED, OUTPUT);
  pinMode(INPIN, INPUT_PULLUP);
  pinMode(PIR_INPIN, INPUT);
  pinMode(SWITCH_INPIN, INPUT);
  digitalWrite(OUTLED, HIGH);
  //attachInterrupt(PIR_INPIN, pir_sensor_int, CHANGE);//add if pir connected
  btn_timer.attach(0.05, btn_handle);
  Debugln("DEBUG: Entering loadConfig()");
  if (!SPIFFS.begin()) {
    Serial.println("Failed to mount file system");
  }

  uint8_t mac[6];
  WiFi.macAddress(mac);
  hostName += "-";
  hostName += macToStr(mac);
  String hostTemp = hostName;
  hostTemp.replace(":", "-");
  host = (char*) hostTemp.c_str();
  loadConfig();
  //loadConfigOld();
  Debugln("DEBUG: loadConfig() passed");

  // Connect to WiFi network
  Debugln("DEBUG: Entering initWiFi()");
  initWiFi();
  Debugln("DEBUG: initWiFi() passed");
  Debug("iotMode:");
  Debugln(iotMode);
  Debug("webtypeGlob:");
  Debugln(webtypeGlob);
  Debug("otaFlag:");
  Debugln(otaFlag);
  Debugln("DEBUG: Starting the main loop");
}

void pir_sensor_int()  // function to be fired at the zero crossing to dim the light
{
  if (digitalRead(PIR_INPIN))
  {
    Serial.println("PIR High");
    digitalWrite(OUTLED, HIGH);
    digitalWrite(OUTPIN, HIGH);
    state="1";

    toPub = 1;

  }
  else if (!digitalRead(PIR_INPIN))
  {
    Serial.println("PIR Low");
    digitalWrite(OUTLED, LOW);
    digitalWrite(OUTPIN, LOW);
    state="0";
 
    toPub = 1;
    
  }
  //Serial.println("PIR High");


}
void btn_handle()
{
  if (!digitalRead(INPIN)) {
    ++count; // one count is 50ms
  } else {
    if (count > 1 && count < HUMANPRESSDELAY / 5) { //push between 50 ms and 1 sec
      Serial.print("button pressed ");
      Serial.print(count * 0.05);
      Serial.println(" Sec.");

      Serial.print("Light is ");
      Serial.println(digitalRead(OUTPIN));

      Serial.print("Switching light to ");
      Serial.println(!digitalRead(OUTPIN));
      digitalWrite(OUTPIN, !digitalRead(OUTPIN));
      state = digitalRead(OUTPIN);
      if (iotMode == 1 && mqttClient.connected()) {
        toPub = 1;
        Debugln("DEBUG: toPub set to 1");
      }
    } else if (count > (RESTARTDELAY / 0.05) && count <= (RESETDELAY / 0.05)) { //pressed 3 secs (60*0.05s)
      Serial.print("button pressed ");
      Serial.print(count * 0.05);
      Serial.println(" Sec. Restarting!");
      setOtaFlag(!otaFlag);
      ESP.reset();
    } else if (count > (RESETDELAY / 0.05)) { //pressed 20 secs
      Serial.print("button pressed ");
      Serial.print(count * 0.05);
      Serial.println(" Sec.");
      Serial.println(" Clear settings and resetting!");
      configToClear = 1;
    }
    count = 0; //reset since we are at high
  }
}


boolean connectMQTT(){
  if (mqttClient.connected()){
    return true;
  }  
  
  Serial.print("Connecting to MQTT server ");
  Serial.print(mqttServer);
  Serial.print(" as ");
  Serial.println(host);
  
  if (mqttClient.connect(host)) {
    Serial.println("Connected to MQTT broker");
    if(mqttClient.subscribe((char*)subTopic.c_str())){
      Serial.println("Subsribed to topic.");
    } else {
      Serial.println("NOT subsribed to topic!");      
    }
    return true;
  }
  else {
    Serial.println("MQTT connect failed! ");
    return false;
  }
}

void disconnectMQTT(){
  mqttClient.disconnect();
}

void mqtt_handler(){
  if (toPub==1){
    Debugln("DEBUG: Publishing state via MWTT");
    if(pubState()){
     toPub=0; 
    }
  }
  mqttClient.loop();
  delay(100); //let things happen in background
}

void mqtt_arrived(char* subTopic, byte* payload, unsigned int length) { // handle messages arrived 
  int i = 0;
  Serial.print("MQTT message arrived:  topic: " + String(subTopic));
    // create character buffer with ending null terminator (string)
    
  for(i=0; i<length; i++) {    
    buf[i] = payload[i];
  }
  buf[i] = '\0';
  String msgString = String(buf);
  Serial.println(" message: " + msgString);
  if ((msgString == "R13_ON"))
  {
      Serial.print("Light is ");
      Serial.println(digitalRead(OUTPIN));      
      Serial.print("Switching light to "); 
      Serial.println("high");
      if(switch_status==1)
      { 
      state_sw=0;
      }
      else
      { 
      state_sw=1;
      
      }
      send_status=1;
      
  } 
    else if ((msgString == "R13_OFF")){
      Serial.print("Light is ");
      Serial.println(digitalRead(OUTPIN));    
      Serial.print("Switching light to "); 
      Serial.println("low");
       if(switch_status==0)
      { 
      state_sw=0;
      }
      else
      { 
      state_sw=1;   
      }
      send_status=1;
      send_status=1;
      //digitalWrite(OUTPIN, 0); 
  }
  if (msgString == "Led_on"){
      Serial.print("Led is ");
      Serial.println(digitalRead(OUTLED));      
      Serial.print("Switching LED to "); 
      Serial.println("high");
      digitalWrite(OUTLED, 1); 
  } else if (msgString == "Led_off"){
      Serial.print("Led is ");
      Serial.println(digitalRead(OUTLED));    
      Serial.print("Switching LED to "); 
      Serial.println("low");
      digitalWrite(OUTLED, 0); 
  } 
  else if (msgString == "Status")
  {
       send_status=1;
  }
  else if (msgString == "Sensor_Enable"){
      Serial.print("Sensor Enabled");
      attachInterrupt(PIR_INPIN, pir_sensor_int, CHANGE);       
  } 
   else if (msgString == "Sensor_Disable"){
      Serial.print("Sensor Disable");
      detachInterrupt(PIR_INPIN);    
  }        
}

boolean pubState(){ //Publish the current state of the light    
  if (!connectMQTT()){
      delay(100);
      if (!connectMQTT){                            
        Serial.println("Could not connect MQTT.");
        Serial.println("Publish state NOK");
        return false;
      }
    }
    if (mqttClient.connected()){      
        //String state = (digitalRead(OUTPIN))?"1":"0";
        Serial.println("To publish state " + state );  
      if (mqttClient.publish((char*)pubTopic.c_str(), (char*) state.c_str())) {
        Serial.println("Publish state OK");        
        return true;
      } else {
        Serial.println("Publish state NOK");        
        return false;
      }
     } else {
         Serial.println("Publish state NOK");
         Serial.println("No MQTT connection.");        
     } 
     delay(10);   
}

void initWiFi(){
  Serial.println();
  Serial.println();
  Serial.println("Startup");
  
  // test esid 
  WiFi.disconnect();
  delay(100);
  WiFi.mode(WIFI_STA);
  Serial.print("Connecting to WiFi ");
  Serial.println(esid);
  Debugln(epass);
  WiFi.begin((char*)esid.c_str(), (char*)epass.c_str());
  if ( testWifi() == 20 ) { 
      launchWeb(0);
      return;
  }
  Serial.println("Opening AP");
  setupAP();   
}

int testWifi(void) {
  int c = 0;
  Debugln("Wifi test...");  
  while ( c < 30 ) {
    if (WiFi.status() == WL_CONNECTED) { return(20); } 
    delay(500);
    Serial.print(".");    
    c++;
  }
  Serial.println("WiFi Connect timed out!");
  return(10);
} 


void setupAP(void) {
  
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  int n = WiFi.scanNetworks();
  Serial.println("scan done");
  if (n == 0){
    Serial.println("no networks found");
    st ="<b>No networks found:</b>";
  } else {
    Serial.print(n);
    Serial.println(" Networks found");
    st = "<ul>";
    for (int i = 0; i < n; ++i)
     {
      // Print SSID and RSSI for each network found
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(")");
      Serial.println((WiFi.encryptionType(i) == ENC_TYPE_NONE)?" (OPEN)":"*");
      
      // Print to web SSID and RSSI for each network found
      st += "<li>";
      st +=i + 1;
      st += ": ";
      st += WiFi.SSID(i);
      st += " (";
      st += WiFi.RSSI(i);
      st += ")";
      st += (WiFi.encryptionType(i) == ENC_TYPE_NONE)?" (OPEN)":"*";
      st += "</li>";
      delay(10);
     }
    st += "</ul>";
  }
  Serial.println(""); 
  WiFi.disconnect();
  delay(100);
  WiFi.mode(WIFI_AP);

  
  WiFi.softAP(host);
  WiFi.begin(host); // not sure if need but works
  Serial.print("Access point started with name ");
  Serial.println(host);
  inApMode=1;
  launchWeb(1);
}

void launchWeb(int webtype) {
    Serial.println("");
    Serial.println("WiFi connected");    
    //Start the web server or MQTT
    if(otaFlag==1 && !inApMode){
      Serial.println("Starting OTA mode.");    
      Serial.printf("Sketch size: %u\n", ESP.getSketchSize());
      Serial.printf("Free size: %u\n", ESP.getFreeSketchSpace());
      MDNS.begin(host);
      server.on("/", HTTP_GET, [](){
        server.sendHeader("Connection", "close");
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.send(200, "text/html", otaServerIndex);
      });
      server.on("/update", HTTP_POST, [](){
        server.sendHeader("Connection", "close");
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.send(200, "text/plain", (Update.hasError())?"FAIL":"OK");
        setOtaFlag(0); 
        ESP.restart();
      },[](){
        HTTPUpload& upload = server.upload();
        if(upload.status == UPLOAD_FILE_START){
          //Serial.setDebugOutput(true);
          WiFiUDP::stopAll();
          Serial.printf("Update: %s\n", upload.filename.c_str());
          otaCount=300;
          uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
          if(!Update.begin(maxSketchSpace)){//start with max available size
            Update.printError(Serial);
          }
        } else if(upload.status == UPLOAD_FILE_WRITE){
          if(Update.write(upload.buf, upload.currentSize) != upload.currentSize){
            Update.printError(Serial);
          }
        } else if(upload.status == UPLOAD_FILE_END){
          if(Update.end(true)){ //true to set the size to the current progress
            Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
          } else {
            Update.printError(Serial);
          }
          Serial.setDebugOutput(false);
        }
        yield();
      });
      server.begin();
      Serial.printf("Ready! Open http://%s.local in your browser\n", host);
      MDNS.addService("http", "tcp", 80);
      otaTickLoop.attach(1, otaCountown);
    } else { 
      //setOtaFlag(1); 
      if (webtype==1 || iotMode==0){ //in config mode or WebControle
          if (webtype==1) {           
            webtypeGlob == 1;
            Serial.println(WiFi.softAPIP());
            server.on("/", webHandleConfig);
            server.on("/a", webHandleConfigSave);
            server.on("/gpio", webHandleGpio);//naren          
          } else {
            //setup DNS since we are a client in WiFi net
            if (!MDNS.begin(host)) {
              Serial.println("Error setting up MDNS responder!");
            } else {
              Serial.println("mDNS responder started");
              MDNS.addService("http", "tcp", 80);
            }          
            Serial.println(WiFi.localIP());
            server.on("/", webHandleRoot);  
            server.on("/cleareeprom", webHandleClearRom);
            server.on("/gpio", webHandleGpio);
            server.on("/xml",handleXML);
          }
          //server.onNotFound(webHandleRoot);
          server.begin();
          Serial.println("Web server started");   
          webtypeGlob=webtype; //Store global to use in loop()
        } else if(webtype!=1 && iotMode==1){ // in MQTT and not in config mode     
          //mqttClient.setBrokerDomain((char*) mqttServer.c_str());//naren
          //mqttClient.setPort(1883);//naren
          mqttClient.setServer((char*) mqttServer.c_str(),1883);
          mqttClient.setCallback(mqtt_arrived);
          mqttClient.setClient(wifiClient);
          if (WiFi.status() == WL_CONNECTED){
            if (!connectMQTT()){
                delay(2000);
                if (!connectMQTT()){                            
                  Serial.println("Could not connect MQTT.");
                  Serial.println("Starting web server instead.");
                  iotMode=0;
                  launchWeb(0);
                  webtypeGlob=webtype;
                }
              }                    
            }
      }
    }
}

void webHandleConfig(){
  IPAddress ip = WiFi.softAPIP();
  String ipStr = String(ip[0]) + '.' + String(ip[1]) + '.' + String(ip[2]) + '.' + String(ip[3]);
  String s;
 
  s = "Configuration of " + hostName + " at ";
  s += ipStr;
  s += "<p><a href=\"/gpio\">Control GPIO</a><br />";
  s += st;
  s += "<form method='get' action='a'>";
  s += "<label>SSID: </label><input name='ssid' length=32><label> Pass: </label><input name='pass' type='password' length=64></br>";
  s += "The following is not ready yet!</br>";
  s += "<label>IOT mode: </label><input type='radio' name='iot' value='0'> HTTP<input type='radio' name='iot' value='1' checked> MQTT</br>";
  s += "<label>MQTT Broker IP/DNS: </label><input name='host' length=15></br>";
  s += "<label>MQTT Publish topic: </label><input name='pubtop' length=64></br>";
  s += "<label>MQTT Subscribe topic: </label><input name='subtop' length=64></br>";
  s += "<input type='submit'></form></p>";
  s += "\r\n\r\n";
  Serial.println("Sending 200");  
  server.send(200, "text/html", s); 
}

void webHandleConfigSave(){
  // /a?ssid=blahhhh&pass=poooo
  String s;
  s = "<p>Settings saved to eeprom and reset to boot into new settings</p>\r\n\r\n";
  server.send(200, "text/html", s); 
  Serial.println("clearing EEPROM.");
  clearConfig();
  String qsid; 
  qsid = server.arg("ssid");
  qsid.replace("%2F","/");
  Serial.println("Got SSID: " + qsid);
  esid = (char*) qsid.c_str();
  
  String qpass;
  qpass = server.arg("pass");
  qpass.replace("%2F","/");
  Serial.println("Got pass: " + qpass);
  epass = (char*) qpass.c_str();

  String qiot;
  qiot= server.arg("iot");
  Serial.println("Got iot mode: " + qiot);
  qiot=="0"? iotMode = 0 : iotMode = 1 ;
  
  String qsubTop;
  qsubTop = server.arg("subtop");
  qsubTop.replace("%2F","/");
  Serial.println("Got subtop: " + qsubTop);
  subTopic = (char*) qsubTop.c_str();
  
  String qpubTop;
  qpubTop = server.arg("pubtop");
  qpubTop.replace("%2F","/");
  Serial.println("Got pubtop: " + qpubTop);
  pubTopic = (char*) qpubTop.c_str();
  
  mqttServer = (char*) server.arg("host").c_str();
  Serial.print("Got mqtt Server: ");
  Serial.println(mqttServer);

  Serial.print("Settings written ");
  saveConfig()? Serial.println("sucessfully.") : Serial.println("not succesfully!");;
  Serial.println("Restarting!"); 
  delay(1000);
  ESP.reset();
}

void webHandleRoot(){
  String s;
  s = "<p>Hello from ESP8266";
  s += "</p>";
  s += "<a href=\"/gpio\">Controle GPIO</a><br />";
  s += "<a href=\"/cleareeprom\">Clear settings an boot into Config mode</a><br />";
  s += "\r\n\r\n";
  Serial.println("Sending 200");  
  server.send(200, "text/html", s); 
}

void webHandleClearRom(){
  String s;
  s = "<p>Clearing the config and reset to configure new wifi<p>";
  s += "</html>\r\n\r\n";
  Serial.println("Sending 200"); 
  server.send(200, "text/html", s); 
  Serial.println("clearing config");
  clearConfig();
  delay(10);
  Serial.println("Done, restarting!");
  ESP.reset();
}

void webHandleGpio(){
  String s;
  int state_sensor;
   // Set GPIO according to the request
    if (server.arg("state_sw")=="1" || server.arg("state_sw")=="0" ) 
    {

     state_sw = server.arg("state_sw").toInt();
//      if(((state_sw)&&(!switch_status))||((!state_sw)&&(switch_status)))  //exor logic
//      {
//      //digitalWrite(OUTLED, HIGH);
//      digitalWrite(OUTPIN, HIGH);
//      //Serial.print("Light switched via web request to  ");      
//      //Serial.println(digitalWrite(OUTPIN, HIGH));      
//      }
//      else
//      {
//      digitalWrite(OUTPIN, LOW);
//      //Serial.print("Light switched via web request to  ");      
//      //Serial.println(digitalWrite(OUTPIN, LOW)); 
//      }
      
    }
     if (server.arg("state_led")=="1" || server.arg("state_led")=="0" ) {
      int state_led = server.arg("state_led").toInt();
      Serial.print("Light switched via web request to  ");      
      Serial.println(state_led);      
    }

    else if(server.arg("reboot")=="1")
    {
     ESP.reset(); 
    }
//    if (server.arg("state_sensor")=="1") {
//       state_sensor = server.arg("state_sensor").toInt();
//      attachInterrupt(PIR_INPIN, pir_sensor_int, CHANGE);
//      Serial.print("Sensor Enabled");           
//    }
//     else if ( server.arg("state_sensor")=="0")
//   {
//      state_sensor = server.arg("state_sensor").toInt();
//      detachInterrupt(PIR_INPIN);
//      Serial.print("Sensor Disabled");      
        
//    }
     buildJavascript(); 
   
    s="<!DOCTYPE HTML>\n";
    s+=javaScript;
    s+="<BODY onload='process()'>\n";
    s+="<BR>Wifi Single Relay Board<BR>\n";
    s+="<BR><BR>\n";
    s+="<p>Light is <A id='runtime'></A>\n</p>";
    s+="</BODY>\n";
    
//    s += (digitalRead(OUTPIN))?"on":"off";
    s += "<p>Change to <form action='gpio'><input type='radio' name='state_sw' value='1' ";
    s += (digitalRead(OUTPIN))?"checked":"";
    s += ">Relay_ON<input type='radio' name='state_sw' value='0' ";
    s += (digitalRead(OUTPIN))?"":"checked";
    s += ">Relay_OFF <input type='submit' value='Submit'></form></p>";   
    s += "LED is now ";
    s += (digitalRead(OUTLED))?"ON":"OFF";
    s += "<p>Change to <form action='gpio'><input type='radio' name='state_led' value='1' ";
    s += (digitalRead(OUTLED))?"checked":"";
    s += ">LED_ON <input type='radio' name='state_led' value='0' ";
    s += (digitalRead(OUTLED))?"":"checked";
    s += ">LED_OFF <input type='submit' value='Submit'></form></p>"; 
    s +="<p><a href=\"gpio?reboot=1\">Reboot</a></p>";     
    s+="</HTML>\n";
//    s += "Sensor is now ";
//    s += "<p>Change to <form action='gpio'><input type='radio' name='state_sensor' value='1' ";
//    s += ("state_sensor")?"checked":"";
//    s += ">SENSOR_ON <input type='radio' name='state_sensor' value='0' ";
//    s += ("state_sensor")?"":"checked";
//    s += ">SENSOR_OFF <input type='submit' value='Submit'></form></p>"; 
    server.send(200, "text/html", s);    
}


 

void buildJavascript(){
  javaScript="<SCRIPT>\n";
  javaScript+="var xmlHttp=createXmlHttpObject();\n";

  javaScript+="function createXmlHttpObject(){\n";
  javaScript+=" if(window.XMLHttpRequest){\n";
  javaScript+="    xmlHttp=new XMLHttpRequest();\n";
  javaScript+=" }else{\n";
  javaScript+="    xmlHttp=new ActiveXObject('Microsoft.XMLHTTP');\n";
  javaScript+=" }\n";
  javaScript+=" return xmlHttp;\n";
  javaScript+="}\n";

  javaScript+="function process(){\n";
  javaScript+=" if(xmlHttp.readyState==0 || xmlHttp.readyState==4){\n";
  javaScript+="   xmlHttp.open('PUT','xml',true);\n";
  javaScript+="   xmlHttp.onreadystatechange=handleServerResponse;\n"; // no brackets?????
  javaScript+="   xmlHttp.send(null);\n";
  javaScript+=" }\n";
  javaScript+=" setTimeout('process()',1000);\n";
  javaScript+="}\n";
 
  javaScript+="function handleServerResponse(){\n";
  javaScript+=" if(xmlHttp.readyState==4 && xmlHttp.status==200){\n";
  javaScript+="   xmlResponse=xmlHttp.responseXML;\n";
  javaScript+="   xmldoc = xmlResponse.getElementsByTagName('response');\n";
  javaScript+="   message = xmldoc[0].firstChild.nodeValue;\n";
  javaScript+="   document.getElementById('runtime').innerHTML=message;\n";
  javaScript+=" }\n";
  javaScript+="}\n";
  javaScript+="</SCRIPT>\n";
}

void buildXML(){
  XML="<?xml version='1.0'?>";
  XML+="<response>";
  //XML+=millis2time();
  XML+=(digitalRead(OUTPIN))?"ON":"OFF";
  XML+="</response>";
}

//String millis2time(){
//  String Time="";
//  unsigned long ss;
//  byte mm,hh;
//  ss=millis()/1000;
//  hh=ss/3600;
//  mm=(ss-hh*3600)/60;
//  ss=(ss-hh*3600)-mm*60;
//  if(hh<10)Time+="0";
//  Time+=(String)hh+":";
//  if(mm<10)Time+="0";
//  Time+=(String)mm+":";
//  if(ss<10)Time+="0";
//  Time+=(String)ss;
//  return Time;
//}

void handleXML(){
  buildXML();
  server.send(200,"text/xml",XML);
}


void Scan_Wifi_Networks()
{

  int n = WiFi.scanNetworks();


  if (n == 0)
  {
   // Serial.println("no networks found");
  }
  else
  {
    Serial.print(n);
    Serial.println(" networks found");
    for (int i = 0; i < n; ++i)
    {
      if (esid == WiFi.SSID(i))

      {
        Serial.print("Old Network Found ");
        Do_Connect();
        //Fl_MyNetwork = true;
      }
      else
      {
        Serial.print("Old Network Not Found");
      }
     
    }
  }
  Serial.println("");
}


void Do_Connect()                  // Try to connect to the Found WIFI Network!
{

ESP.wdtDisable();
  ESP.restart(); 
  
}
    
//-------------------------------- Main loop ---------------------------
void loop() {


   if(switch_status==(digitalRead(SWITCH_INPIN)))// to read the status of physical switch
   {
        // send_status=0;
   }
   else
   {
    switch_status=(digitalRead(SWITCH_INPIN));
     send_status=1;
   }

  if(send_status==1)
  {
     send_status=0;
     toPub = 1;   
  }
  else
  {   
     toPub = 0;
  }
   

  
  if(((state_sw)&&(!switch_status))||((!state_sw)&&(switch_status)))  //exor logic
      {
      //digitalWrite(OUTLED, HIGH);
      digitalWrite(OUTPIN, HIGH);
     // toPub = 1;
       state="Light is ON";
      //Serial.print("Light switched via web request to  ");      
      //Serial.println(digitalWrite(OUTPIN, HIGH));      
      }
      else
      {
      digitalWrite(OUTPIN, LOW);
      //toPub = 1;
       state="Light is OFF";
      //Serial.print("Light switched via web request to  ");      
      //Serial.println(digitalWrite(OUTPIN, LOW)); 
      }
  
  //Debugln("DEBUG: loop() begin");

  if (configToClear == 1) {
    //Debugln("DEBUG: loop() clear config flag set!");
    clearConfig() ? Serial.println("Config cleared!") : Serial.println("Config could not be cleared");
    delay(1000);
    ESP.reset();
  }
  //Debugln("DEBUG: config reset check passed");
  if (WiFi.status() == WL_CONNECTED && otaFlag) {
    if (otaCount <= 1) {
      Serial.println("OTA mode time out. Reset!");
      setOtaFlag(0);
      ESP.reset();
      delay(100);
    }
    server.handleClient();
    delay(1);
  } else if (WiFi.status() == WL_CONNECTED || webtypeGlob == 1) {
    //Debugln("DEBUG: loop() wifi connected & webServer ");
    if (iotMode == 0 || webtypeGlob == 1) {
      //Debugln("DEBUG: loop() Web mode requesthandling ");
      server.handleClient();
      delay(1);
      if(esid != "" && WiFi.status() != WL_CONNECTED) //wifi reconnect part
      {
        Scan_Wifi_Networks();
      }
    } else if (iotMode == 1 && webtypeGlob != 1 && otaFlag != 1) {
      //Debugln("DEBUG: loop() MQTT mode requesthandling ");
      if (!connectMQTT()) {
        delay(200);
      }
      if (mqttClient.connected()) {
        //Debugln("mqtt handler");
        mqtt_handler();
      } else {
        Debugln("mqtt Not connected!");
      }
    }
  }

  else {
    Debugln("DEBUG: loop - WiFi not connected");
    delay(1000);
    initWiFi(); //Try to connect again
  }

  //Debugln("DEBUG: loop() end");
}
