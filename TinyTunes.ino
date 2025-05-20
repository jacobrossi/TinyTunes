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
  Serial.printf("MQTT Message [%s]\n",topic);
  if (length < sizeof(url)) {
    memcpy(url, payload, length);
    url[length] = '\0';
  } else {
    Serial.println("MQTT payload too large");
  }

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
    unsigned long startAttemptTime = millis();
    while (!mqttClient.connected() && millis() - startAttemptTime < 10000) { // 10-second timeout to avoid heap fragmentation
      Serial.print("Connecting to MQTT...");
      // Attempt to connect
      if (mqttClient.connect(mqttClientName,mqttUser,mqttPassword)) {
        Serial.println("connected.");
        // Once connected, publish an announcement...
        mqttClient.publish("tinytunes_status","connected");
        // ... and resubscribe
        mqttClient.subscribe("tinytunes_newmedia");
      } else {
        Serial.printf("failed, code=%d  WiFi RSSI=%d try again in 3 seconds\n", mqttClient.state(), WiFi.RSSI());
        // Wait 3 seconds before retrying
        yield();
        delay(3000);
      }
    }
  }else{
    Serial.println("WiFi Disconnected");
    connectToNetwork();
  }
}

// HTTP ---------------------------------------------------------------------
#define FETCHED_IMG_BUFFER_SIZE 350000 //Max bytes of the image
//const int FETCH_CHUNK_SIZE = 128; //Max bytes to read at a time
uint8_t *fetchedImg;
#define DECODED_IMG_BUFFER_SIZE 640 //Max size of image from Spotify we'll accept
uint16_t *decodedImg;
// Number of milliseconds to wait without receiving any data before we give up
const int kNetworkTimeout = 30*1000;
// Number of milliseconds to wait if no data is available before trying again
const int kNetworkDelay = 0;
// Retry settings
const int kRetryDelay = 1500;
const int kRetryTries = 4;
int retryCount=0;

void resizeImg(uint16_t *orig_img, int orig_width, int orig_height, int final_width, int final_height) {
  // Calculate how many pixels from the original image are in each block to be averaged into a single pixel of the final image
  int block_width = orig_width/final_width;
  int block_height = orig_height/final_height;

  for(int final_x=0; final_x<final_width; final_x++) {  //Each column in final image
    for(int final_y=0; final_y<final_height; final_y++) {  //Each row in final image
      //For each block, avg the pixels from the original image
      unsigned long sum = 0;
      for(int block_x=0; block_x<block_width; block_x++) { //Each column in the block
        for(int block_y=0; block_y<block_height; block_y++) { //Each row in the block
          sum += orig_img[(final_y*block_height+block_y)*orig_width +(final_x*block_width+block_x)]; //Trust me bro
        }
      }
      uint16_t avg_color = sum/(block_width*block_height);
      //Draw final pixel
      dma_display->drawPixel(final_x, final_y, avg_color);
    }
  }
}

void fetchImg() {
  Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("WiFi RSSI: %d\n",WiFi.RSSI());
  HttpClient http = HttpClient(wifi_http, haURL, haPort);
  //Check WiFi connection
  if ((WiFi.status() == WL_CONNECTED)) {
    // Request image
    Serial.printf("[HTTP] GET: %s\n", url);
    int err = 0;
    err = http.get(url);
    if(err == 0) {
      retryCount = 0;
      int httpCode = http.responseStatusCode();
      if(httpCode < 200) {
        Serial.printf("Response Code: %d\n Retrying...\n", httpCode);
        delay(kRetryDelay);
        http.stop();
        fetchImg();
        return;
      }
      Serial.printf("Response Code: %d\nDownloading ", httpCode);
      unsigned long timeoutStart = millis();
      unsigned long downloadStart = timeoutStart;
      // While we haven't timed out & haven't reached the end of the body
      int nextByte = 0;
     // Use a buffer to read chunks of data
     const int bufferSize = 1024; // Adjust as needed
     uint8_t buffer[bufferSize];

     while ((http.connected() || http.available()) && (!http.endOfBodyReached()) && ((millis() - timeoutStart) < kNetworkTimeout) && (nextByte+bufferSize<FETCHED_IMG_BUFFER_SIZE)) {
       if (http.available()) {
         int bytesRead = http.read(buffer, bufferSize); // Read a chunk
         if (bytesRead > 0) {
           memcpy(fetchedImg + nextByte, buffer, bytesRead); // Copy to fetchedImg
           nextByte += bytesRead;
           if(nextByte+bufferSize>FETCHED_IMG_BUFFER_SIZE) {
            Serial.println("- Error: exceeds buffer size.");
           }
           timeoutStart = millis();
         }
       }
       yield();
     }
      http.stop();
        unsigned long duration = millis()-downloadStart;
        unsigned long speed = (nextByte/1024)/(duration/1000);
        Serial.printf("\nComplete. Duration: %dms Avg Speed: %dKb/s\n",duration,speed);

        uint16_t w = 0, h = 0;
        TJpgDec.getJpgSize(&w, &h, fetchedImg, nextByte);
        Serial.printf("Width: %dpx, Height:%dpx, Size: %d bytes\n",w,h,nextByte);

        // Draw the image, top left at 0,0
        dma_display->clearScreen();
        JRESULT decoderResult = TJpgDec.drawJpg(0, 0, fetchedImg, nextByte);
        if(decoderResult==JDR_OK) {
          Serial.println("JPG Decoded");
          // Now resize
          resizeImg(decodedImg,DECODED_IMG_BUFFER_SIZE,DECODED_IMG_BUFFER_SIZE,PANEL_RES_X,PANEL_RES_Y);
        }else{
          if(decoderResult==JDR_INTR) {
            Serial.println("JPG Decoder Error: JDR_INTR - The decompression process was interrupted by output function.");
          }else if(decoderResult==JDR_INP) {
            Serial.println("JPG Decoder Error: JDR_INP - An error occured in input function due to hard error or wrong stream termination.");
          }else if(decoderResult==JDR_PAR) {
            Serial.println("JPG Decoder Error: JDR_PAR - Parameter error. Given scale factor is invalid.");
          }else if(decoderResult==JDR_FMT1) {
            Serial.println("JPG Decoder Error: JDR_FMT1 - Data format error. The input JPEG data can be collapted.");
          }else{
            Serial.println("JPG Decode Error: Unknown");
          }
          // fetchImg();
          return;
        }
    }else{
      Serial.printf("Request Error: %d\n",err);
      http.stop();
      if (retryCount++<kRetryTries) {
         delay(kRetryDelay);
         Serial.println("Retrying");
         fetchImg();
         return;
      } else {
        Serial.printf("Failed to fetch img after %d attempts\n",retryCount);
        retryCount=0;
      }
    }
  } else {
    Serial.println("WiFi Disconnected");
    connectToNetwork();
  }
  http.stop();
}

// GRAPHICS  ----------------------------------------------------------------
void drawBMPFile(char *fname) {
  GFXcanvas16* bmpcanvas;
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
  //if ( y >= PANEL_RES_Y ) return 1;

  // This function will clip the image block rendering automatically at the matrix boundaries
  //dma_display->drawRGBBitmap(x,y,bitmap,w,h);
  for(int i=0; i<w && (x+i)<DECODED_IMG_BUFFER_SIZE; i++) {
    for(int j=0; j<h && (y+j)<DECODED_IMG_BUFFER_SIZE; j++) {
      decodedImg[DECODED_IMG_BUFFER_SIZE*y+x+i] = bitmap[j*w+i];
    }
  }
  // Return 1 to decode next block
  return 1;
}

void connectToNetwork() {
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
  Serial.printf(" RSSI: %d\n",WiFi.RSSI());

  // Connect to MQTT
  mqttClient.setServer(mqttHost, mqttPort);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(60);
}

int mqttPulse = 0;

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

  fetchedImg = (uint8_t *)ps_malloc(FETCHED_IMG_BUFFER_SIZE * sizeof(uint8_t));
  decodedImg = (uint16_t *)ps_malloc(DECODED_IMG_BUFFER_SIZE * DECODED_IMG_BUFFER_SIZE * sizeof(uint16_t));
  
  // Draw Boot Logo
  Serial.println("\n\n==========================BOOT=======================================");
  Serial.println("Displaying Boot Logo");
  drawBMPFile("/img/TinyTunes.bmp");

  connectToNetwork();

  // Setup jpg decoder
  // The jpeg image can be scaled by a factor of 1, 2, 4, or 8
  TJpgDec.setJpgScale(1);
  // The byte order can be swapped (set true for TFT_eSPI)
  TJpgDec.setSwapBytes(false);
  // The decoder must be given the exact name of the rendering function above
  TJpgDec.setCallback(drawImgBlock);
  mqttPulse = millis();
}

// ARDUINO MAIN LOOP ------------------------------------------------------------------
char keepAlive[10];
void loop() 
{
  // Reconnect to wifi if disconnected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi Disconnected. Reconnecting...");
    connectToNetwork();
  }
  //Process MQTT for new messages
  if (!mqttClient.connected()) {
    mqttReconnect();
  }
  mqttClient.loop();
  //Publish a message to keep mqtt alive (workaround bug)
  if(millis()-mqttPulse>10000) {
    sprintf(keepAlive,"%d",random());
    mqttClient.publish("tinytunes_keepalive", keepAlive);
    mqttPulse = millis();
  }
}