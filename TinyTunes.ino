#include <Adafruit_SPIFlash.h>
#include <Adafruit_TinyUSB.h>
#include "SdFat.h"
#include "SPI.h"
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h> 
#include <Adafruit_ImageReader.h> // Image-reading functions
#include <WiFi.h>
#include <PubSubClient.h>
#include "PrivateConfig.h"

// CONFIGURABLE SETTINGS ---------------------------------------------------
#define WIDTH  64             // Matrix width in pixels
#define HEIGHT 64             // Matrix height in pixels
char BMPpath[] = "/img";     // Absolute path to BMPs on CIRCUITPY drive
uint16_t BMPminimumTime = 10; // Min. repeat time (seconds) until next BMP

// FLASH FILESYSTEM STUFF --------------------------------------------------

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

// LED Matrix pinout config - default is for Adafruit Matrix Portal S3
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
 
//MatrixPanel_I2S_DMA dma_display;
MatrixPanel_I2S_DMA *dma_display = nullptr;

uint16_t myBLACK = dma_display->color565(0, 0, 0);
uint16_t myWHITE = dma_display->color565(255, 255, 255);
uint16_t myRED = dma_display->color565(255, 0, 0);
uint16_t myGREEN = dma_display->color565(0, 255, 0);
uint16_t myBLUE = dma_display->color565(0, 0, 255);

File bmpFile;

// FUNCTIONS REQUIRED FOR USB MASS STORAGE ---------------------------------

static bool msc_changed = true; // Is set true on filesystem changes

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

// Get number of files in a specified path that match extension ('filter').
// Pass in absolute path (e.g. "/" or "/img") and extension WITHOUT period
// (e.g. "bmp", NOT ".bmp").
int16_t numFiles(const char *path, const char *filter) {
  File dir = filesys.open(path);
  if (!dir) return -1;
  char filename[256];
  for(int16_t num_files = 0;;) {
    File entry = dir.openNextFile();
    if (!entry) return num_files; // No more files
    entry.getName(filename, sizeof(filename) - 1);
    entry.close();
    if (!entry.isDirectory() &&       // Skip directories
        strncmp(filename, "._", 2)) { // and Mac junk files
      char *extension = strrchr(filename, '.');
      if (extension && !strcasecmp(&extension[1], filter)) num_files++;
    }
  }
  return -1;
}

// Return name of file (matching extension) by index (0 to numFiles()-1)
char *filenameByIndex(const char *path, const char *filter, int16_t index) {
  static char filename[256]; // Must be static, we return a pointer to this!
  File entry, dir = filesys.open(path);
  if (!dir) return NULL;
  while(entry = dir.openNextFile()) {
    entry.getName(filename, sizeof(filename) - 1);
    entry.close();
    if(!entry.isDirectory() &&       // Skip directories
       strncmp(filename, "._", 2)) { // and Mac junk files
      char *extension = strrchr(filename, '.');
      if (extension  && !strcasecmp(&extension[1], filter)) {
        if(!index--) {
          return filename;
        }
      }
    }
  }
  return NULL;
}

GFXcanvas16* bmpcanvas;

void drawBMPFile(char *fname) {
  ImageReturnCode stat; // Status from image-reading functions
  Adafruit_Image img;
  stat = reader.loadBMP(fname, img);
  reader.printStatus(stat);

  
  Serial.printf("Size: %d,%d",img.width(), img.height());
  bmpcanvas = static_cast<GFXcanvas16*>(img.getCanvas());
  uint16_t pixel; 
  for(int x=0; x<img.width(); x++) {
    for(int y=0; y<img.height(); y++) {
      pixel = bmpcanvas->getPixel(x,y);
      dma_display->drawPixel(x,y,pixel);
    }
  }
}

/************************* Arduino Sketch Setup and Loop() *******************************/
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
#if defined(BACK_BUTTON)
  pinMode(BACK_BUTTON, INPUT_PULLUP);
#endif
#if defined(NEXT_BUTTON)
  pinMode(NEXT_BUTTON, INPUT_PULLUP);
#endif

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

  Serial.begin(115200);

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
  
  /* all other pixel drawing functions can only be called after .begin() */
  drawBMPFile("/img/TinyTunes.bmp");

  delay(1000); 
}

// LOOP FUNCTION - RUNS REPEATEDLY UNTIL RESET / POWER OFF -----------------
int16_t BMPindex = -1;     // Current file index in GIFpath
int8_t BMPincrement = 1;   // +1 = next GIF, -1 = prev, 0 = same

void loop() 
{
 
delay(5000);
}