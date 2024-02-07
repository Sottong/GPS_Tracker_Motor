#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <String.h>

#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>         // https://github.com/tzapu/WiFiManager

#include <PubSubClient.h>

#define PINrelay3 D6 //kontak
#define PINrelay4 D7 //starter

WiFiManager wm;
WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

bool tugasSudahDikerjakan = false; // Menandai apakah tugas sudah dikerjakan

// const char* mqtt_server = "test.mosquitto.org"; // Alamat broker MQTT
const char* mqtt_server = "broker.hivemq.com"; // Alamat broker MQTT
const int mqttPort = 1883; // Port MQTT default

const char* topic_set = "gps/motor/set"; //topic untuk menerima pesan
const char* topic_get = "gps/motor/get"; //topic untuk membalas pesan

double lat_val, lng_val;
bool loc_valid;
bool fl_device_ready = 0;
uint8_t fl_count_before_send = 0;

static const int RXPin = D4, TXPin = D3;
static const uint32_t GPSBaud = 9600;
int LED = LED_BUILTIN;

volatile float minutes, seconds;
volatile int degree, secs, mins;

String link;
String link1;

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);




void Wifi_connetion(){
 
  if(wm.getWiFiSSID() == ""){
    bool res;
    res = wm.autoConnect("AutoConnectAP"); // anonymous ap

    if(!res) {
        Serial.println("Failed to connect");
        // ESP.restart();
    } 
    else {
        //if you get here you have connected to the WiFi    
        Serial.println("connected...yeey :)");
    }
  } 
  else {
    Serial.println();
    Serial.print("Menghubungkan ke WiFi: ");
    Serial.println(wm.getWiFiSSID());

    WiFi.begin(wm.getWiFiSSID(), wm.getWiFiPass());

    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }

    Serial.println("");
    Serial.println("Koneksi WiFi berhasil");
    Serial.print("Alamat IP: ");
    Serial.println(WiFi.localIP());
  }

}

void mqtt_connet(){
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // ... and resubscribe
      client.subscribe(topic_set);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if(strcmp(topic, topic_set) == 0){
    Serial.println("topic ok");
    if(payload[0] == '0'){
      Serial.println("hidupkan motor");
      digitalWrite(PINrelay3, LOW);
      delay(3000);
      digitalWrite(PINrelay4, LOW);
      delay(3000);
      digitalWrite(PINrelay4, HIGH);
      client.publish(topic_get, "\"0\"");

    }
    else if(payload[0] == '1'){
      Serial.println("matikan motor");
      digitalWrite(PINrelay3, HIGH);
      client.publish(topic_get, "\"1\"");
    }
    else if(payload[0] == '2'){
      Serial.println("nyalakan kontak");
      digitalWrite(PINrelay3, LOW);
      client.publish(topic_get, "\"2\"");
    }
    else if(payload[0] == '3'){
      Serial.println("nyalakan mesin");
      digitalWrite(PINrelay4, LOW);
      delay(3000);
      digitalWrite(PINrelay4, HIGH);
      client.publish(topic_get, "\"3\"");
    }
    else if(payload[0] == '4'){
      Serial.println("lokasi");

      String lokasi = String(lat_val, 8) + "," + String(lng_val,8);
      client.publish(topic_get, lokasi.c_str());

    }

  }
  else{
    Serial.println("topic Nok abaikan pesan");
  }
}

void setup(){
  Serial.begin(9600);
  WiFi.mode(WIFI_STA); 

  // reset settings - wipe stored credentials for testing
  // these are stored by the esp library
  // wm.resetSettings();

  Wifi_connetion();

  client.setServer(mqtt_server, mqttPort);
  client.setCallback(callback);

  mqtt_connet();
  
  pinMode(LED, OUTPUT);
  pinMode(PINrelay3, OUTPUT);
  pinMode(PINrelay4, OUTPUT);
  digitalWrite(PINrelay3, HIGH); // relay aktiv low
  digitalWrite(PINrelay4, HIGH); // relay aktiv low
}

void wifi_reconnet(){

  Serial.println();
  Serial.print("Menghubungkan ke WiFi: ");
  Serial.println(wm.getWiFiSSID());
  WiFi.begin(wm.getWiFiSSID(), wm.getWiFiPass());

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("Koneksi WiFi berhasil");
  Serial.print("Alamat IP: ");
  Serial.println(WiFi.localIP());

}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  ss.begin(GPSBaud);
  do 
  {
    while (ss.available())  /* Encode data read from GPS while data is available on serial port */
      gps.encode(ss.read());
/* Encode basically is used to parse the string received by the GPS and to store it in a buffer so that information can be extracted from it */
  } while (millis() - start < ms);
}
void DegMinSec( double tot_val)   /* Convert data in decimal degrees into degrees minutes seconds form */
{  
  degree = (int)tot_val;
  minutes = tot_val - degree;
  seconds = 60 * minutes;
  minutes = (int)seconds;
  mins = (int)minutes;
  seconds = seconds - minutes;
  seconds = 60 * seconds;
  secs = (int)seconds;
}

void loop(){
  unsigned long currentMillis = millis(); // Waktu sekarang

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Koneksi WiFi terputus, mencoba untuk menyambung kembali...");
    wifi_reconnet();
  }
  if (!client.connected()){
      mqtt_connet();
  }
  client.loop();

  smartDelay(1000);
  unsigned long start;
        
  lat_val = gps.location.lat(); /* Get latitude data */
  loc_valid = gps.location.isValid(); /* Check if valid location data is available */
  lng_val = gps.location.lng(); /* Get longtitude data */

  if (!loc_valid)
  {          
    Serial.print("Latitude : ");
    Serial.println("*****");
    Serial.print("Longitude : ");
    Serial.println("*****");
  }
  else
  {
    DegMinSec(lat_val);
    Serial.print("Latitude in Decimal Degrees : ");
    Serial.println(lat_val, 6);
    DegMinSec(lng_val); /* Convert the decimal degree value into degrees minutes seconds form */
    Serial.print("Longitude in Decimal Degrees : ");
    Serial.println(lng_val, 6);
    if(fl_device_ready == 0){
      fl_count_before_send++;
      if(fl_count_before_send == 10){
        fl_device_ready = 1;
        fl_count_before_send = 0;
        client.publish(topic_get, "\"6\"");
        Serial.println("kirim pesan, perangkat siap");
      }
    }
  }
}