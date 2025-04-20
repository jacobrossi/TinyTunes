#include <Adafruit_SPIFlash.h>
#include <Adafruit_TinyUSB.h>
#include "SdFat.h"
#include "SPI.h"
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h> 
#include <Adafruit_ImageReader.h> 
#include <WiFi.h>
#include <ArduinoHttpClient.h>
#include <PubSubClient.h>
#include "PrivateConfig.h"
#include <TJpg_Decoder.h>  
// PrivateConfig.h is not included in repo
// Create it in the root directory and define variables:  
//    const char *ssid = "";
//    const char *wifiPassword = "";
//    const char *mqttHost = "homeassistant.local"; // Change this to your home assistant server
//    const int mqttPort = 8123; // Change this to your home assistant mqtt port, typically 1883
//    const char *mqttClientName = ""; // Create a name for your mqtt client (can be same as user)
//    const char *mqttUser = ""; // Create a home assistant user with local access
//    const char *mqttPassword = ""; // Password for home assistant user with local access

// FLASH FILESYSTEM STUFF -----------------------------------------------------------
// External flash macros for QSPI or SPI are defined in board variant file.
#if defined(ARDUINO_ARCH_ESP32)
static Adafruit_FlashTransport_ESP32 flashTransport;
#elif defined(EXTERNAL_FLASH_USE_QSPI)
Adafruit_FlashTransport_QSPI flashTransport;
#elif defined(EXTERNAL_FLASH_USE_SPI)
Adafruit_FlashTransport_SPI flashTransport(EXTERNAL_FLASH_USE_CS,
                                           EXTERNAL_FLASH_USE_SPI);
#else
#error No QSPI/SPI flash are defined in your board variant.h!
#endif

Adafruit_SPIFlash flash(&flashTransport);
FatFileSystem filesys;     // Filesystem object from SdFat
Adafruit_ImageReader reader(filesys); 
Adafruit_USBD_MSC usb_msc; // USB mass storage object


// LED MATRIX CONFIG - default is for Adafruit Matrix Portal S3 --------------------
#define R1_PIN 42
#define G1_PIN 41
#define B1_PIN 40
#define R2_PIN 38
#define G2_PIN 39
#define B2_PIN 37
#define A_PIN  45
#define B_PIN  36
#define C_PIN  48
#define D_PIN  35
#define E_PIN  21 
#define LAT_PIN 47
#define OE_PIN  14
#define CLK_PIN 2

HUB75_I2S_CFG::i2s_pins _pins={R1_PIN, G1_PIN, B1_PIN, R2_PIN, G2_PIN, B2_PIN, A_PIN, B_PIN, C_PIN, D_PIN, E_PIN, LAT_PIN, OE_PIN, CLK_PIN};

// Configure size and number of panels
#define PANEL_RES_X 64      // Number of pixels wide of each INDIVIDUAL panel module. 
#define PANEL_RES_Y 64     // Number of pixels tall of each INDIVIDUAL panel module.
#define PANEL_CHAIN 1      // Total number of panels chained one to another
 
// MatrixPanel_I2S_DMA dma_display;
MatrixPanel_I2S_DMA *dma_display = nullptr;

uint16_t myBLACK = dma_display->color565(0, 0, 0);
uint16_t myWHITE = dma_display->color565(255, 255, 255);
uint16_t myRED = dma_display->color565(255, 0, 0);
uint16_t myGREEN = dma_display->color565(0, 255, 0);
uint16_t myBLUE = dma_display->color565(0, 0, 255);

File bmpFile;

void initDisplay() {
  HUB75_I2S_CFG mxconfig(
    PANEL_RES_X,   // module width
    PANEL_RES_Y,   // module height
    PANEL_CHAIN,    // Chain length
    _pins
  );

  // Display Setup
  dma_display = new MatrixPanel_I2S_DMA(mxconfig);
  dma_display->begin();
  dma_display->setBrightness8(128); //0-255
  dma_display->clearScreen();
  dma_display->fillScreen(myWHITE);
  dma_display->begin();
}

// USB MASS STORAGE HELPERS --------------------------------------------------------
static bool msc_changed = true; // Is set true on filesystem changes

// Initialize
void initFilesystem() {
  // USB mass storage / filesystem setup (do BEFORE Serial init)
  flash.begin();
  // Set disk vendor id, product id and revision
  usb_msc.setID("Adafruit", "External Flash", "1.0");
  // Set disk size, block size is 512 regardless of spi flash page size
  usb_msc.setCapacity(flash.pageSize() * flash.numPages() / 512, 512);
  usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);
  usb_msc.setUnitReady(true); // MSC is ready for read/write
  usb_msc.begin();
  filesys.begin(&flash); // Start filesystem on the flash
}

// Callback on READ10 command.
int32_t msc_read_cb(uint32_t lba, void *buffer, uint32_t bufsize) {
  return flash.readBlocks(lba, (uint8_t *)buffer, bufsize / 512) ? bufsize : -1;
}

// Callback on WRITE10 command.
int32_t msc_write_cb(uint32_t lba, uint8_t *buffer, uint32_t bufsize) {
  digitalWrite(LED_BUILTIN, HIGH);
  return flash.writeBlocks(lba, buffer, bufsize / 512) ? bufsize : -1;
}

// Callback on WRITE10 completion.
void msc_flush_cb(void) {
  flash.syncBlocks();   // Sync with flash
  filesys.cacheClear(); // Clear filesystem cache to force refresh
  digitalWrite(LED_BUILTIN, LOW);
  msc_changed = true;
}

// MQTT PubSub ---------------------------------------------------------------------
char url[256];
char payload_cpy[256];
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("MQTT Message [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
  //  Serial.print((char)payload[i]);
    url[i] = (char)payload[i];
  }
  url[length] = '\0';
  //String message = (char*)payload;
  //url = haURL + message;

  //sprintf(url,"%s%s",haURL,payload_cpy);
  Serial.printf("%s\n",url);

  if(strcmp(topic,"tinytunes_newmedia")==0) {
      fetchImg();
  }
}

WiFiClient wifi_mqtt;
WiFiClient wifi_http;
PubSubClient mqttClient(wifi_mqtt);

void mqttReconnect() {
  if ((WiFi.status() == WL_CONNECTED)) {
    // Loop until we're reconnected
    while (!mqttClient.connected()) {
      Serial.print("Connecting to MQTT...");
      // Attempt to connect
      if (mqttClient.connect(mqttClientName,mqttUser,mqttPassword)) {
        Serial.println("connected.");
        // Once connected, publish an announcement...
        mqttClient.publish("tinytunes_status","connected");
        // ... and resubscribe
        mqttClient.subscribe("tinytunes_newmedia");
      } else {
        Serial.print("failed, rc=");
        Serial.print(mqttClient.state());
        Serial.println(" try again in 3 seconds");
        // Wait 5 seconds before retrying
        delay(3000);
      }
    }
  }else{
    Serial.println("WiFi Disconnected");
  }
}

// HTTP ---------------------------------------------------------------------
#define FETCHED_IMG_BUFFER_SIZE 200000 //Max bytes of the image
//const int FETCH_CHUNK_SIZE = 128; //Max bytes to read at a time
uint8_t fetchedImg[FETCHED_IMG_BUFFER_SIZE] = {0};
// Number of milliseconds to wait without receiving any data before we give up
const int kNetworkTimeout = 30*1000;
// Number of milliseconds to wait if no data is available before trying again
const int kNetworkDelay = 0;
// Retry settings
const int kRetryDelay = 1500;
const int kRetryTries = 2;
int retryCount=0;

HttpClient http = HttpClient(wifi_http, haURL, haPort);

void fetchImg() {

  //Check WiFi connection
  if ((WiFi.status() == WL_CONNECTED)) {
    // Request image
    Serial.printf("[HTTP] GET: %s\n", url);
    int err = 0;
    err = http.get(url);
    if(err == 0) {
      retryCount = 0;
      int httpCode = http.responseStatusCode();
      Serial.printf("Response Code: %d\nDownloading ", httpCode);
      unsigned long timeoutStart = millis();
      unsigned long downloadStart = timeoutStart;
      // While we haven't timed out & haven't reached the end of the body
      int nextByte = 0;
      while ((http.connected() || http.available()) && (!http.endOfBodyReached()) && ((millis() - timeoutStart) < kNetworkTimeout)) {
        if (http.available()) {
          fetchedImg[nextByte++] = (uint8_t)http.read();
          timeoutStart = millis();
        } else {
          // We haven't got any data, so let's pause to allow some to arrive
          //Serial.print(".");
          //delay(kNetworkDelay);
        }
      }
        unsigned long duration = millis()-downloadStart;
        unsigned long speed = (nextByte/1024)/(duration/1000);
        Serial.printf("\nComplete. Duration: %dms Avg Speed: %dKb/s\n",duration,speed);

        uint16_t w = 0, h = 0;
        TJpgDec.getJpgSize(&w, &h, fetchedImg, nextByte);
        Serial.printf("Width: %dpx, Height:%dpx, Size: %d bytes\n",w,h,nextByte);

        // Draw the image, top left at 0,0
        dma_display->clearScreen();
        TJpgDec.drawJpg(0, 0, fetchedImg, nextByte);
        Serial.println("Draw Complete");
    }else{
      Serial.printf("Request Error: %d\n",err);
      http.endRequest();
      http.stop();
      if (retryCount++<kRetryTries) {
         delay(kRetryDelay);
         Serial.println("Retrying");
         fetchImg();
      } else {
        Serial.printf("Failed to fetch img after %d attempts\n",retryCount);
        retryCount=0;
      }
    }
  } else {
    Serial.println("Not connected to Wifi");
  }
}

// GRAPHICS  ----------------------------------------------------------------
GFXcanvas16* bmpcanvas;

void drawBMPFile(char *fname) {
  ImageReturnCode stat; // Status from image-reading functions
  Adafruit_Image img;
  stat = reader.loadBMP(fname, img);
  bmpcanvas = static_cast<GFXcanvas16*>(img.getCanvas());
  uint16_t pixel; 
  for(int x=0; x<img.width(); x++) {
    for(int y=0; y<img.height(); y++) {
      pixel = bmpcanvas->getPixel(x,y);
      dma_display->drawPixel(x,y,pixel);
    }
  }
}

bool drawImgBlock(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap)
{
  // Stop further decoding as image is running off bottom of screen
  if ( y >= PANEL_RES_Y ) return 0;

  // This function will clip the image block rendering automatically at the TFT boundaries
  //tft.pushImage(x, y, w, h, bitmap);
  //Serial.printf("Drawing block [%d,%d,%d,%d]\n",x,y,w,h);
  dma_display->drawRGBBitmap(x,y,bitmap,w,h);
  
  // Return 1 to decode next block
  return 1;
}

// ARDUINO SETUP FUNCTION -----------------------------------------------------------
void setup() {

  // Init HW buttons
  pinMode(LED_BUILTIN, OUTPUT);
#if defined(BACK_BUTTON)
  pinMode(BACK_BUTTON, INPUT_PULLUP);
#endif
#if defined(NEXT_BUTTON)
  pinMode(NEXT_BUTTON, INPUT_PULLUP);
#endif

  delay(3000);

  initFilesystem();

  Serial.begin(115200);

  initDisplay();
  
  // Draw Boot Logo
  Serial.println("Displaying Boot Logo");
  drawBMPFile("/img/TinyTunes.bmp");

  // Connect to WiFi
  Serial.printf("Connecting to WiFi: %s", ssid);
  WiFi.begin(ssid, wifiPassword);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi Connected: "); 
  Serial.print(WiFi.localIP());
  Serial.println(" ");

  // Connect to MQTT
  mqttClient.setServer(mqttHost, mqttPort);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(0xFFFF);

  // Setup jpg decoder
  // The jpeg image can be scaled by a factor of 1, 2, 4, or 8
  TJpgDec.setJpgScale(8);

  // The byte order can be swapped (set true for TFT_eSPI)
  TJpgDec.setSwapBytes(false);

  // The decoder must be given the exact name of the rendering function above
  TJpgDec.setCallback(drawImgBlock);

  //TJpgDec.drawSdJpg(0, 0, filesys.open("/test.jpg", FILE_READ)

  // Allow the hardware to sort itself out
  //delay(1500); 
}

// ARDUINO MAIN LOOP ------------------------------------------------------------------
void loop() 
{
  if (!mqttClient.connected()) {
    mqttReconnect();
  }
  mqttClient.loop();
}