/***************************************************
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/
#include <EEPROM.h>
#define EEPROM_SIZE 64
int address = 1;

double timestamp = 0;
#include <SD.h>
#include "Adafruit_EPD.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
Adafruit_BMP3XX bmp; // I2C
float pressureNow = 0.0;
int total = 0;
#ifdef ESP32
  #define SD_CS       14
  #define SRAM_CS     32
  #define EPD_CS      15
  #define EPD_DC      33  
#endif
float pressureBaseline = 0.0;
int  percent = 0;
#define EPD_RESET   -1 // can set to -1 and share with microcontroller Reset!
#define EPD_BUSY    -1 // can set to -1 to not use a pin (will wait a fixed delay)


/* Uncomment the following line if you are using 2.13" tricolor EPD */
Adafruit_IL0373 epd(212, 104 ,EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY);
/* Uncomment the following line if you are using 2.13" monochrome 250*122 EPD */
//Adafruit_SSD1675 epd(250, 122, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY);


bool drawBitmap = false;

void setup() {
  Serial.begin(115200);
  //while (!Serial);
  Serial.println("2.13 inch EInk Featherwing test");

  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }
  
  if (!EEPROM.begin(EEPROM_SIZE))
  {
    Serial.println("failed to initialise EEPROM"); delay(1000000);
  }
  if(EEPROM.read(0) == 1){
    total = EEPROMReadlong(address);
  } else {
    Serial.print("going down the right tube");
    EEPROMWritelong( address,  total);
    delay(20);
    EEPROM.write(0, 1);
    EEPROM.commit();
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  //bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CS)) {
    Serial.println("failed!");
  } else {
    drawBitmap = true;
  }

  File root = SD.open("/");
  printDirectory(root, 0);
  Serial.println("done!");

  Serial.println("begin");
  epd.begin();
  epd.setTextWrap(true);
  epd.setTextSize(2);

   if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  delay(100);
  pressureBaseline = bmp.pressure / 100.0;
  
  // Draw a bitmap
 
  if (drawBitmap) {
    epd.clearBuffer();  
    epd.fillScreen(EPD_WHITE);
    bmpDraw("/tinylogo.bmp",0,0);
  }
  float result = 100 * (float(total) / 29000);
  percent = int(result);
  epd.setCursor(130, 10);
  epd.setTextColor(EPD_BLACK);
  epd.print(percent);
  epd.setCursor(180, 10);
  epd.setTextColor(EPD_RED);
  epd.print("%");
  
  
  epd.setCursor(130, 40);
  epd.setTextColor(EPD_RED);
  epd.print("TTL FT");
  epd.setCursor(130, 70);
  epd.setTextColor(EPD_BLACK);
  epd.print(total);
  epd.display();
  timestamp = millis();
}

void loop() {
  if((millis()-timestamp) > 5 * 60 * 1000){
   //total = total  + 5000;
  printOut();
  timestamp = millis();

  }
 
 
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  delay(100);
  pressureNow = bmp.pressure / 100.0;
  if (pressureBaseline > pressureNow) {
    
    total = total + (pressureBaseline - pressureNow) * 27.8;
  }
  
  pressureBaseline = pressureNow;
 float result = 100 * (float(total) / 29000);
  percent = int(result);
  if(total >= 29000){
    endofeverest();
  }
  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");
  
 

}
void endofeverest(){
  if (drawBitmap) {
    epd.clearBuffer();  
    epd.fillScreen(EPD_WHITE);
    bmpDraw("/top.bmp",0,0);
  } 
  epd.setCursor(85, 6);
  epd.setTextColor(EPD_RED);
  epd.print("Join Us!");
  epd.display();
  delay(5 * 60 * 1000);
  total = 0;
  percent = 0;
  printOut();
}
void printOut(){
  EEPROMWritelong( address,  total);
    delay(20);
    Serial.print("Saved Total");
    Serial.println(EEPROMReadlong(address));
  
    epd.clearBuffer();  
    epd.fillScreen(EPD_WHITE);
    bmpDraw("/tinylogo.bmp",0,0);
  epd.setCursor(130, 10);
  epd.setTextColor(EPD_BLACK);
  epd.print(percent);
  epd.setCursor(180, 10);
  epd.setTextColor(EPD_RED);
  epd.print("%");
  
  
  epd.setCursor(130, 40);
  epd.setTextColor(EPD_RED);
  epd.print("TTL FT");
  epd.setCursor(130, 70);
  epd.setTextColor(EPD_BLACK);
  epd.print(total);
  epd.display();
}
void EEPROMWritelong(int address, unsigned long value)
{
  //Decomposition from a long to 4 bytes by using bitshift.
  //One = Most significant -> Four = Least significant byte
  byte four = (value & 0xFF);
  byte three = ((value >> 8) & 0xFF);
  byte two = ((value >> 16) & 0xFF);
  byte one = ((value >> 24) & 0xFF);
  
  //Write the 4 bytes into the eeprom memory.
  EEPROM.write(address, four);
  EEPROM.write(address + 1, three);
  EEPROM.write(address + 2, two);
  EEPROM.write(address + 3, one);
  EEPROM.commit();
}

//This function will return a 4 byte (32bit) long from the eeprom
//at the specified address to adress + 3.
unsigned long EEPROMReadlong(long address)
{
  //Read the 4 bytes from the eeprom memory.
  long four = EEPROM.read(address);
  long three = EEPROM.read(address + 1);
  long two = EEPROM.read(address + 2);
  long one = EEPROM.read(address + 3);
  
  //Return the recomposed long by using bitshift.
  return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}



// This function opens a Windows Bitmap (BMP) file and
// displays it at the given coordinates.  It's sped up
// by reading many pixels worth of data at a time
// (rather than pixel by pixel).  Increasing the buffer
// size takes more of the Arduino's precious RAM but
// makes loading a little faster.  20 pixels seems a
// good balance.

#define BUFFPIXEL 20

bool bmpDraw(char *filename, int16_t x, int16_t y) {

  File     bmpFile;
  int      bmpWidth, bmpHeight;   // W+H in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  uint8_t  sdbuffer[3*BUFFPIXEL]; // pixel buffer (R+G+B per pixel)
  uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
  boolean  goodBmp = false;       // Set to true on valid header parse
  boolean  flip    = true;        // BMP is stored bottom-to-top
  int      w, h, row, col, x2, y2, bx1, by1;
  uint8_t  r, g, b;
  uint32_t pos = 0, startTime = millis();

  if((x >= epd.width()) || (y >= epd.height())) return false;

  Serial.println();
  Serial.print(F("Loading image '"));
  Serial.print(filename);
  Serial.println('\'');

  // Open requested file on SD card
  if ((bmpFile = SD.open(filename)) == NULL) {
    Serial.print(F("File not found"));
    return false;
  }

  // Parse BMP header
  if(read16(bmpFile) == 0x4D42) { // BMP signature
    Serial.print(F("File size: ")); Serial.println(read32(bmpFile));
    (void)read32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
    Serial.print(F("Image Offset: ")); Serial.println(bmpImageoffset, DEC);
    // Read DIB header
    Serial.print(F("Header size: ")); Serial.println(read32(bmpFile));
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);
    if(read16(bmpFile) == 1) { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel
      Serial.print(F("Bit Depth: ")); Serial.println(bmpDepth);
      if((bmpDepth == 24) && (read32(bmpFile) == 0)) { // 0 = uncompressed

        goodBmp = true; // Supported BMP format -- proceed!
        Serial.print(F("Image size: "));
        Serial.print(bmpWidth);
        Serial.print('x');
        Serial.println(bmpHeight);

        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bmpWidth * 3 + 3) & ~3;

        // If bmpHeight is negative, image is in top-down order.
        // This is not canon but has been observed in the wild.
        if(bmpHeight < 0) {
          bmpHeight = -bmpHeight;
          flip      = false;
        }

        // Crop area to be loaded
        x2 = x + bmpWidth  - 1; // Lower-right corner
        y2 = y + bmpHeight - 1;
        if((x2 >= 0) && (y2 >= 0)) { // On screen?
          w = bmpWidth; // Width/height of section to load/display
          h = bmpHeight;
          bx1 = by1 = 0; // UL coordinate in BMP file
  
          for (row=0; row<h; row++) { // For each scanline...
  
            // Seek to start of scan line.  It might seem labor-
            // intensive to be doing this on every line, but this
            // method covers a lot of gritty details like cropping
            // and scanline padding.  Also, the seek only takes
            // place if the file position actually needs to change
            // (avoids a lot of cluster math in SD library).
            if(flip) // Bitmap is stored bottom-to-top order (normal BMP)
              pos = bmpImageoffset + (bmpHeight - 1 - (row + by1)) * rowSize;
            else     // Bitmap is stored top-to-bottom
              pos = bmpImageoffset + (row + by1) * rowSize;
            pos += bx1 * 3; // Factor in starting column (bx1)
            if(bmpFile.position() != pos) { // Need seek?
              bmpFile.seek(pos);
              buffidx = sizeof(sdbuffer); // Force buffer reload
            }
            for (col=0; col<w; col++) { // For each pixel...
              // Time to read more pixel data?
              if (buffidx >= sizeof(sdbuffer)) { // Indeed
                bmpFile.read(sdbuffer, sizeof(sdbuffer));
                buffidx = 0; // Set index to beginning
              }
              // Convert pixel from BMP to EPD format, push to display
              b = sdbuffer[buffidx++];
              g = sdbuffer[buffidx++];
              r = sdbuffer[buffidx++];

              uint8_t c = 0;
              if ((r < 0x80) && (g < 0x80) && (b < 0x80)) {
                 c = EPD_BLACK; // try to infer black
              } else if ((r >= 0x80) && (g >= 0x80) && (b >= 0x80)) {
                 c = EPD_WHITE;
              } else if (r >= 0x80) {
                c = EPD_RED; //try to infer red color
              }
              
              epd.writePixel(col, row, c);
            } // end pixel
          } // end scanline
        } // end onscreen
        epd.display();
        Serial.print(F("Loaded in "));
        Serial.print(millis() - startTime);
        Serial.println(" ms");
      } // end goodBmp
    }
  }

  bmpFile.close();
  if(!goodBmp) {
    Serial.println(F("BMP format not recognized."));
    return false;
  }
  return true;
}

// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.

uint16_t read16(File &f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

uint32_t read32(File &f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}


void printDirectory(File dir, int numTabs) {
  while (true) {

    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}
