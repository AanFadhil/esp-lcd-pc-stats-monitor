#include "config.h"
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include <DHT.h>
#include <ArduinoJson.h>

#define DHTTYPE DHT11
#define DHT_PIN 27

#define ETHERNET_CS 5
// For NodeMCU - use pin numbers in the form PIN_Dx where Dx is the NodeMCU pin designation
#define SPI_MISO 19  // Automatically assigned with ESP8266 if not defined
#define SPI_MOSI 23  // Automatically assigned with ESP8266 if not defined
#define SPI_SCLK 18  // Automatically assigned with ESP8266 if not defined

#define FONT_PIXEL_HEIGHT 8

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
#define FONT_SIZE 2
#define MAX_UDP_PACKET_SIZE 50

//# Region Types
struct StatsDTO {
  int cpu;
  int cput;
  int mem;
  int hh;
  int mm;
};

struct StatsTemp {
  float t;
  float h;
  float hic;
  bool read;
};
//# EndRegion Types

//# Region Method declarations
void captureDHT();
void connectTCP();
void padString(String &str);
void ipPrinter(IPAddress *ipToPrint);
void padStringSymmetrical(String *input, int length);
void setSensorTextColor(int &value);
void drawStatsText(StatsDTO *data, StatsTemp *tempData);
void ipPrinter(IPAddress *ipToPrint);
void readUdp();
void readSerial();
void pingServer();
void createMagicPacket(const char *macAddress, uint8_t *magicPacket);
//# EndRegion Method declarations

//# Region Constants
const int centerX = SCREEN_WIDTH / 2;
const int centerY = SCREEN_HEIGHT / 2;

byte ethernet_mac[6] = ETHERNET_MAC;

const unsigned int wifiUdp_localPort = WIFI_LOCAL_UDP_PORT;
const char *wifiUdp_ssid = WIFI_SSID;
const char *wifiUdp_password = WIFI_PASSWORD;
const unsigned int localUDPPort = WIFI_LOCAL_UDP_PORT;  // local port to listen on
const unsigned int echoUDPPort = ECHO_UDP_PORT;

const char *udp_server_addr = UDP_SERVER_ADDR;
const uint16_t udp_server_port = UDP_SERVER_PORT;

const int32_t x = 25, y = 20, space = 10;
const uint8_t dispfont = 1, size = 3;

const char *macAddress = PC_MAC_ADDRESS;
//# EndRegion Constants

//# Region Global Lib Instances
StaticJsonDocument<200> doc;

DHT dht(DHT_PIN, DHTTYPE);

TFT_eSPI tft = TFT_eSPI();

EthernetClient client;
const IPAddress ip(ETHERNET_IP);
const IPAddress gateway(ETHERNET_GATEWAY_IP);
const IPAddress broadcast(ETHERNET_BC);
const IPAddress subnetmask(ETHERNET_SUBNET_MASK);

EthernetUDP Udp;
WiFiUDP wifi_udp;
//# EndRegion Global Lib Instances


//# Region Public variables
StatsDTO stats;
StatsTemp tempStats;

int screenHeight;
int screenWidth;

float lastDHTCaptured = 0;
float lastMessageReceivedAt = 0;
float lastRenderAt = 0;
float lastPingAt = 0;
bool blinkFlop = false;
bool reading = false;
String bufferPrint = String("");

char packetBuffer[MAX_UDP_PACKET_SIZE];  // buffer to hold incoming packet,
char ReplyBuffer[] = "ack";              // a string to send back

uint8_t magicPacket[102];
//# EndRegion Public variables

void setup() {
  Serial.begin(9600);

  SPI.begin(SPI_SCLK, SPI_MISO, SPI_MOSI);
  // Initialize SPI pins
  pinMode(TFT_CS, OUTPUT);
  pinMode(ETHERNET_CS, OUTPUT);

  WiFi.begin(wifiUdp_ssid, wifiUdp_password);

  wifi_udp.begin(wifiUdp_localPort);

  // Deactivate both devices initially
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(ETHERNET_CS, HIGH);

  digitalWrite(ETHERNET_CS, LOW);
  Ethernet.init(ETHERNET_CS);

  digitalWrite(TFT_CS, HIGH);
  digitalWrite(ETHERNET_CS, LOW);

  Ethernet.begin(ethernet_mac);
  Ethernet.setLocalIP(ip);
  Ethernet.setSubnetMask(subnetmask);

  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
  } else if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  } else {
    Serial.print("Ethernet.localIP : ");
    Serial.println(Ethernet.localIP());
    Serial.print("Ethernet.gatewayIP : ");
    Serial.println(Ethernet.gatewayIP());
    Serial.print("Ethernet.dnsServerIP : ");
    Serial.println(Ethernet.dnsServerIP());
    Serial.print("Ethernet.subnetMask : ");
    Serial.println(Ethernet.subnetMask());
  }

  Udp.begin(localUDPPort);

  digitalWrite(ETHERNET_CS, HIGH);

  dht.begin();

  digitalWrite(TFT_CS, LOW);
  tft.init();  // Initialize the display
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_TRANSPARENT);

  screenHeight = tft.height();
  screenWidth = tft.width();
  digitalWrite(TFT_CS, HIGH);


  Serial.print("h:");
  Serial.println(screenHeight);
  Serial.print("w:");
  Serial.println(screenWidth);

  //convertKey(enc_key_str.c_str(), enc_key);
  createMagicPacket(macAddress, magicPacket);
}

void padStringSymmetrical(String *input, int length) {
  int totalPadding = length - input->length();
  int leftPadding = totalPadding / 2;
  int rightPadding = totalPadding - leftPadding;

  for (int i = 0; i < leftPadding; i++) {
    *input = ' ' + *input;
  }
  for (int i = 0; i < rightPadding; i++) {
    *input += ' ';
  }
}

void captureDHT() {
  float deltaTime = millis() - lastDHTCaptured;

  if (deltaTime > 2000) {
    lastDHTCaptured = millis();
    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (isnan(h) || isnan(t)) {
      Serial.println(F("Failed to read from DHT sensor!"));

      tempStats.read = false;

      return;
    }

    float hic = dht.computeHeatIndex(t, h, false);

    tempStats.read = true;
    tempStats.h = h;
    tempStats.t = t;
    tempStats.hic = hic;
  }
}

void setSensorTextColor(int &value) {
  if (value > 85) {
    tft.setTextColor(TFT_RED, TFT_BLACK);
  } else if (value > 50) {
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  } else if (value < 0) {
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  } else {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
  }
}

void drawStatsText() {

  float deltaTime = millis() - lastRenderAt;

  if (deltaTime < 1000) {
    return;
  }

  digitalWrite(TFT_CS, LOW);
  try {
    int16_t fontheight = size * 8;
    int16_t spaceMultiplier = fontheight + space;

    tft.setTextSize(size);
    setSensorTextColor(stats.cpu);
    tft.drawString("CPU       : " + String(stats.cpu) + String("% "), x, y, dispfont);

    setSensorTextColor(stats.cput);
    tft.drawString("CPU Temp  : " + String(stats.cput) + String("C "), x, y + (spaceMultiplier * 1) + 1, dispfont);

    setSensorTextColor(stats.mem);
    tft.drawString("Memory    : " + String(stats.mem) + String("% "), x, y + (spaceMultiplier * 2) + 1, dispfont);

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setTextSize(4);

    char timeString[6];                                    // Buffer to hold the formatted time string "hh:mm"
    sprintf(timeString, "%02d:%02d", stats.hh, stats.mm);  // Format with leading zeros

    tft.drawString(String(timeString), x, y + (spaceMultiplier * 3) + 20, dispfont);

    String hicDisp = tempStats.read ? (String(tempStats.hic, 1) + "C") : "--";
    padStringSymmetrical(&hicDisp, 6);

    String tDisp = tempStats.read ? (String(tempStats.t, 1) + "C") : "--";  //6 char
    String hDisp = tempStats.read ? (String(tempStats.h, 1) + "%") : "--";  //6 char
    String tempDetailDisp = hDisp + " | " + tDisp;                          // 6 + 6 + 3
    padStringSymmetrical(&tempDetailDisp, 6 + 6 + 3);

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(4);
    tft.drawCentreString(hicDisp, 240, y + (spaceMultiplier * 3) + 20, dispfont);
    tft.setTextSize(1);
    tft.drawCentreString(tempDetailDisp, 240, y + (spaceMultiplier * 3) + 20 + (3 * 8) + 8, dispfont);
  } catch (...) {
  }

  digitalWrite(TFT_CS, HIGH);

  lastRenderAt = millis();
}

void ipPrinter(IPAddress *ipToPrint) {
  for (int i = 0; i < 4; i++) {
    Serial.print((*ipToPrint)[i], DEC);
    if (i < 3) {
      Serial.print(".");
    }
  }
}

void createMagicPacket(const char *macaddr, uint8_t *mgcPac) {
  // Parse the MAC address string into bytes
  uint8_t macBytes[6];
  sscanf(macaddr, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
         &macBytes[0], &macBytes[1], &macBytes[2],
         &macBytes[3], &macBytes[4], &macBytes[5]);

  // Fill the first 6 bytes of the packet with 0xFF
  memset(mgcPac, 0xFF, 6);

  // Repeat the MAC address 16 times
  for (int i = 0; i < 16; i++) {
    memcpy(mgcPac + 6 + i * 6, macBytes, 6);
  }
}

void printMagicPacket(const uint8_t *mgPac, size_t length) {
    // Print the magic packet in hexadecimal format
    for (size_t i = 0; i < length; i++) {
        if (i > 0 && i % 6 == 0) Serial.print(" "); // Add a space every 6 bytes for readability
        Serial.printf("%02X", mgPac[i]);
    }
    Serial.println();
}

void readUdp() {
  digitalWrite(ETHERNET_CS, LOW);
  int packetSize = wifi_udp.parsePacket();
  if (packetSize) {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remote = wifi_udp.remoteIP();
    ipPrinter(&remote);
    Serial.print(", port ");
    Serial.println(wifi_udp.remotePort());

    // read the packet into packetBuffer
    wifi_udp.read(packetBuffer, MAX_UDP_PACKET_SIZE);
    Serial.print("Contents:");
    Serial.println(packetBuffer);

    // send a reply to the IP address and port that sent us the packet we received
    if (String(packetBuffer) == "wakey") {
      memset(packetBuffer, '\0', sizeof(packetBuffer));
      Serial.println("Wakeup Call Received");
      printMagicPacket(magicPacket, sizeof(magicPacket));
      Udp.beginPacket(gateway, echoUDPPort);
      Udp.write((const char *)magicPacket);
      Udp.endPacket();

      Udp.beginPacket(broadcast, 9);
      Udp.write((const char *)magicPacket);
      Udp.endPacket();
    } else if (String(packetBuffer) != "pong") {
      Serial.print("non wakeup reply");
      wifi_udp.beginPacket(udp_server_addr, udp_server_port);
      String fw_msg = String("forwading: ") + packetBuffer;
      wifi_udp.printf(fw_msg.c_str());
      wifi_udp.endPacket();

      Udp.beginPacket(broadcast, echoUDPPort);
      Udp.write(packetBuffer);
      Udp.endPacket();

      Udp.beginPacket(gateway, echoUDPPort);
      Udp.write(packetBuffer);
      Udp.endPacket();
      memset(packetBuffer, '\0', sizeof(packetBuffer));
    } 
  }

  digitalWrite(ETHERNET_CS, HIGH);
}

void readSerial() {

  float deltaTime = millis() - lastMessageReceivedAt;

  digitalWrite(TFT_CS, LOW);
  if (deltaTime > 5000) {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setTextSize(2);
    tft.drawString("connection lost    ", x, 225, dispfont);
  } else {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setTextSize(2);
    tft.drawString("device connected   ", x, 225, dispfont);
  }
  digitalWrite(TFT_CS, HIGH);

  if (!reading || bufferPrint.length() > 256) {
    bufferPrint.clear();
  }

  if (Serial.available()) {

    char c = Serial.read();

    if (c != '\n') {
      reading = true;
      bufferPrint.concat(c);
    } else {
      if (bufferPrint == "PING_THE_CLOCK") {
        Serial.print("theclock_123\n");
      } else {
        Serial.print("serializing : ");
        Serial.println(bufferPrint.c_str());

        // Deserialize the JSON document
        DeserializationError error = deserializeJson(doc, bufferPrint.c_str());

        // Test if parsing succeeds.
        if (error) {
          Serial.print("serialization error : ");
          Serial.println(error.c_str());
          return;
        }

        stats.cpu = doc["cpu"];
        stats.cput = doc["cput"];
        stats.mem = doc["mem"];
        stats.hh = doc["hh"];
        stats.mm = doc["mm"];

        lastMessageReceivedAt = millis();
      }
      reading = false;
    }
  }
}

void pingServer() {
  float deltaTime = millis() - lastPingAt;

  if (deltaTime <= 10000 || WiFi.status() != WL_CONNECTED) return;

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("wifi not connected");
    return;
  }

  lastPingAt = millis();
  String pingMsg = (String("ping_") + String(lastPingAt));  //encryptToBase64((String("ping_") + String(lastPingAt)));

  wifi_udp.beginPacket(udp_server_addr, udp_server_port);
  wifi_udp.printf(pingMsg.c_str());
  wifi_udp.endPacket();
}

void loop() {
  captureDHT();
  readSerial();
  readUdp();
  drawStatsText();
  pingServer();
}