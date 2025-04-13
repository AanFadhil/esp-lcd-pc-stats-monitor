#include <Base64.h>

#include <AESLib.h>

#include <SPI.h>
#include <TFT_eSPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

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

DHT dht(DHT_PIN, DHTTYPE);

TFT_eSPI tft = TFT_eSPI();

int centerX = SCREEN_WIDTH / 2;
int centerY = SCREEN_HEIGHT / 2;

int screenHeight;
int screenWidth;

float lastDHTCaptured = 0;
float lastMessageReceivedAt = 0;
float lastRenderAt = 0;
bool blinkFlop = false;


bool reading = false;
String bufferPrint = String("");

void captureDHT();
void connectTCP();
void padString(String &str);
void ipPrinter(IPAddress *ipToPrint);

StaticJsonDocument<200> doc;

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

StatsDTO stats;
StatsTemp tempStats;

// Define MAC address and optional static IP
byte ethernet_mac[6] = { 0x6F, 0xFC, 0x31, 0x35, 0xA1, 47 };
EthernetClient client;
IPAddress ip(192, 168, 99, 99);
IPAddress myDns(192, 168, 99, 1);
IPAddress gateway(192, 168, 99, 1);
IPAddress subnet(255, 255, 0, 0);

unsigned int localUDPPort = 8888;  // local port to listen on
EthernetUDP Udp;

AESLib aesLib;

byte aes_key[32];

// General initialization vector (you must use your own IV's in production for full security!!!)
byte aes_iv[16];

String server_b64iv = "MTIzNDU2Nzg5MDEyMzQ1Ng==";  // same as aes_iv  but in Base-64 form as received from server
String server_key = "MTIzNDU2Nzg5MDEyMzQ1Njc4OTAxMjM0NTY3ODkwMTI=";

char packetBuffer[MAX_UDP_PACKET_SIZE];  // buffer to hold incoming packet,
char ReplyBuffer[] = "ack";                 // a string to send back

char cleartext[256] = { 0 };
char ciphertext[512];

String encrypt_impl(char *msg, byte iv[]) {
  int msgLen = strlen(msg);
  char encrypted[2 * msgLen] = { 0 };
  aesLib.encrypt64((const byte *)msg, msgLen, encrypted, aes_key, sizeof(aes_key), iv);
  return String(encrypted);
}

String decrypt_impl(char *msg, byte iv[]) {
  int msgLen = strlen(msg);
  char decrypted[msgLen] = { 0 };  // half may be enough
  aesLib.decrypt64(msg, msgLen, (byte *)decrypted, aes_key, sizeof(aes_key), iv);
  return String(decrypted);
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }

  SPI.begin(SPI_SCLK, SPI_MISO, SPI_MOSI);
  // Initialize SPI pins
  pinMode(TFT_CS, OUTPUT);
  pinMode(ETHERNET_CS, OUTPUT);

  // Deactivate both devices initially
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(ETHERNET_CS, HIGH);

  digitalWrite(ETHERNET_CS, LOW);
  Ethernet.init(ETHERNET_CS);

  digitalWrite(TFT_CS, HIGH);
  digitalWrite(ETHERNET_CS, LOW);

  Ethernet.begin(ethernet_mac);
  Ethernet.setLocalIP(ip);
  Ethernet.setGatewayIP(gateway);
  Ethernet.setDnsServerIP(myDns);
  Ethernet.setSubnetMask(subnet);

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

  aesLib.set_paddingmode(paddingMode::CMS);
  base64_decode((char *)aes_iv, (char *)server_b64iv.c_str(), server_b64iv.length());
  base64_decode((char *)aes_key, (char *)server_key.c_str(), server_key.length());
}

int32_t x = 25, y = 20, space = 10;
uint8_t dispfont = 1, size = 3;

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

void drawStatsText(StatsDTO *data, StatsTemp *tempData) {

  digitalWrite(TFT_CS, LOW);
  try {
    int16_t fontheight = size * 8;
    int16_t spaceMultiplier = fontheight + space;

    tft.setTextSize(size);
    setSensorTextColor(data->cpu);
    tft.drawString("CPU       : " + String(data->cpu) + String("% "), x, y, dispfont);

    setSensorTextColor(data->cput);
    tft.drawString("CPU Temp  : " + String(data->cput) + String("C "), x, y + (spaceMultiplier * 1) + 1, dispfont);

    setSensorTextColor(data->mem);
    tft.drawString("Memory    : " + String(data->mem) + String("% "), x, y + (spaceMultiplier * 2) + 1, dispfont);

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setTextSize(4);

    char timeString[6];                                    // Buffer to hold the formatted time string "hh:mm"
    sprintf(timeString, "%02d:%02d", data->hh, data->mm);  // Format with leading zeros

    tft.drawString(String(timeString), x, y + (spaceMultiplier * 3) + 20, dispfont);

    String hicDisp = tempData->read ? (String(tempData->hic, 1) + "C") : "--";
    padStringSymmetrical(&hicDisp, 6);

    String tDisp = tempData->read ? (String(tempData->t, 1) + "C") : "--";  //6 char
    String hDisp = tempData->read ? (String(tempData->h, 1) + "%") : "--";  //6 char
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
}

void ipPrinter(IPAddress *ipToPrint) {
  for (int i = 0; i < 4; i++) {
    Serial.print((*ipToPrint)[i], DEC);
    if (i < 3) {
      Serial.print(".");
    }
  }
}

void readUdp() {
  digitalWrite(ETHERNET_CS, LOW);
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remote = Udp.remoteIP();
    ipPrinter(&remote);
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    // read the packet into packetBuffer
    Udp.read(packetBuffer, MAX_UDP_PACKET_SIZE);
    Serial.println("Contents:");
    Serial.println(packetBuffer);
    // send a reply to the IP address and port that sent us the packet we received
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(ReplyBuffer);
    Udp.write(":");
    Udp.write(packetBuffer);
    Udp.endPacket();
    memset(packetBuffer, '\0', sizeof(packetBuffer));
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
      // else {
      //   Serial.print("decoding : ");
      //   Serial.println(bufferPrint.c_str());

      //   Serial.print("Key : ");
      //   Serial.println((char *)aes_key);
      //   Serial.print("IV : ");
      //   Serial.println((char *)aes_iv);

      //   Serial.print("decrypt result : ");
      //   char msgCopy[bufferPrint.length()];
      //   strcpy(msgCopy, bufferPrint.c_str());
      //   String result = decrypt_impl(msgCopy, aes_iv);
      //   Serial.println(result);

      //   Serial.print("encrypt result : ");
      //   String enc_result = encrypt_impl(msgCopy, aes_iv);
      //   Serial.println(enc_result);
      // }

      reading = false;
    }
  }
}

void loop() {
  captureDHT();
  readSerial();
  readUdp();

  float deltaTime = millis() - lastRenderAt;

  if (deltaTime > 1000) {
    drawStatsText(&stats, &tempStats);
    lastRenderAt = millis();
  }
}