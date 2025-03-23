#include <TFT_eSPI.h>
#include <DHT.h>
#include <ArduinoJson.h>

#define DHTTYPE DHT11
#define DHT_PIN 27
#define FONT_PIXEL_HEIGHT 8

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
#define FONT_SIZE 2

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


void setup() {
  Serial.begin(9600);
  dht.begin();

  tft.init();  // Initialize the display
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_TRANSPARENT);

  screenHeight = tft.height();
  screenWidth = tft.width();

  Serial.print("h:");
  Serial.println(screenHeight);
  Serial.print("w:");
  Serial.println(screenWidth);
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
  
  String tDisp = tempData->read ? (String(tempData->t, 1) + "C") : "--"; //6 char
  String hDisp = tempData->read ? (String(tempData->h, 1) + "%") : "--"; //6 char
  String tempDetailDisp = hDisp + " | " + tDisp; // 6 + 6 + 3 
  padStringSymmetrical(&tempDetailDisp, 6 + 6 + 3);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(4);
  tft.drawCentreString(hicDisp, 240, y + (spaceMultiplier * 3) + 20, dispfont);
  tft.setTextSize(1);
  tft.drawCentreString(tempDetailDisp, 240, y + (spaceMultiplier * 3) + 20 + (3*8) + 8, dispfont);
}

void readSerial() {

  float deltaTime = millis() - lastMessageReceivedAt;

  if (deltaTime > 5000) {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setTextSize(2);
    tft.drawString("connection lost    ", x, 225, dispfont);
  } else {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setTextSize(2);
    tft.drawString("device connected   ", x, 225, dispfont);
  }

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

        tft.setTextSize(size);

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

void loop() {
  captureDHT();
  readSerial();

  float deltaTime = millis() - lastRenderAt;

  if (deltaTime > 1000) {  
    drawStatsText(&stats, &tempStats);
    lastRenderAt = millis();
  }
}
