/***************************************************************************
Weather Station, displays Temperature, Pressure, Altitude, and Humidity on an OLED
Programmer: Zohaib Amir
Date: 28.06.2024


Hardware:
BMP280 (Optional)
AHT20
128x64 OLED
ESP32 DOIT DEVKIT 30Pin

 ***************************************************************************/

#include <Wire.h>  //I2C Library
#include <SPI.h>    //SPI Library
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>


/***************************************************************************
OLED 
 ***************************************************************************/

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)

/***************************************************************************
BMP280 Sensor 
 ***************************************************************************/

Adafruit_BMP280 bmp; // I2C address is 0x76 or 0x77
float TempCalibrated; // Variable declared to Calibrate the Sensor Readings

/***************************************************************************
AHT20 Sensor 
 ***************************************************************************/
Adafruit_AHTX0 aht;


/***************************************************************************
Logo
 ***************************************************************************/

unsigned char temperature_icon[] ={
  0b00000001, 0b11000000, //        ###      
  0b00000011, 0b11100000, //       #####     
  0b00000111, 0b00100000, //      ###  #     
  0b00000111, 0b11100000, //      ######     
  0b00000111, 0b00100000, //      ###  #     
  0b00000111, 0b11100000, //      ######     
  0b00000111, 0b00100000, //      ###  #     
  0b00000111, 0b11100000, //      ######     
  0b00000111, 0b00100000, //      ###  #     
  0b00001111, 0b11110000, //     ########    
  0b00011111, 0b11111000, //    ##########   
  0b00011111, 0b11111000, //    ##########   
  0b00011111, 0b11111000, //    ##########   
  0b00011111, 0b11111000, //    ##########   
  0b00001111, 0b11110000, //     ########    
  0b00000111, 0b11100000, //      ######     
};



void setup() {

/***************************************************************************
Serial Setup 
 ***************************************************************************/
  Serial.begin(9600);
  while ( !Serial ) delay(100);   // wait for native usb
  Serial.println(F("BMP280 test"));
  unsigned status;

  /***************************************************************************
OLED Setup 
 ***************************************************************************/
 // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.setTextColor(WHITE);
  delay(1000); // Pause for 1 Sec
 
/***************************************************************************
AHT20 Setup 
 ***************************************************************************/
    if (! aht.begin()) {
    Serial.println("Could not find AHT? Check wiring");
      display.clearDisplay();// Clear the buffer.
      display.setCursor(0, 0);
      display.setTextSize(1);
      display.print("Could not find AHT? Check wiring");
      display.display(); //Sends Output to Display
    while (1) delay(10);
  }
  Serial.println("AHT10 or AHT20 found");
      display.clearDisplay();// Clear the buffer.
      display.setCursor(0, 0);
      display.setTextSize(1);
      display.print("AHT10 or AHT20 found");
      display.display(); //Sends Output to Display
      delay(2000); // Pause for 1 Sec


 /***************************************************************************
 Start BMP280
 ***************************************************************************/

  status = bmp.begin(0x77); //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
          display.clearDisplay();// Clear the buffer.
          display.setTextSize(1);
          display.setCursor(0, 20);
          display.print("Could not find a valid BMP280 sensor");
          display.setCursor(0, 40);
          display.print("SensorID was: 0x");
          display.setCursor(97, 40);
          display.print(bmp.sensorID(),16);
          display.display(); //Sends Output to Display

    while (1) delay(10);
  }
  Serial.println("BMP280 found");
      display.setCursor(0, 20);
      display.setTextSize(1);
      display.print("BMP280 found");
      display.display(); //Sends Output to Display
      delay(2000); // Pause for 1 Sec
      
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */


}



void loop() {

   
  sensors_event_t humidity, temp; //Get Data from AHT20
  aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
  TempCalibrated = bmp.readTemperature() - 0.5;  //Calibration Correction Value

  display.clearDisplay();
  display.setTextColor(WHITE);

/***************************************************************************
 Draw Images and Sensor Data on Display
 ***************************************************************************/

  display.setTextSize(2);
  display.drawBitmap(15, 5, temperature_icon, 16, 16 ,1);
  display.setCursor(35, 5);
  display.print(TempCalibrated);
  display.cp437(true); //Import CharacterSet for Celcius Symbol
  display.setTextSize(1);
  display.print(" ");
  display.write(167); //Celcius Symbol
  display.print("C");

  display.setCursor(0, 34);
  display.setTextSize(1);
  display.print("Pressure: ");
  display.print(bmp.readPressure());
  display.print(" hpa");

  display.setCursor(0, 44);
  display.setTextSize(1);
  display.print("Altitude: ");
  display.print(bmp.readAltitude(1013.25));  //Standard pressure at sea level is defined as 1013hPa
  display.print(" m");

  display.setCursor(0, 54);
  display.setTextSize(1);
  display.print("Humidity: ");
  display.print(humidity.relative_humidity);
  display.print(" % rH");

  display.display(); 


  /***************************************************************************
 Serial Monitor Data Output
 ***************************************************************************/

    Serial.print(F("Pressure BMP280 = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print(F("Temperature BMP = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print(F("Temperature Calibrated BMP = "));
    Serial.print(TempCalibrated);
    Serial.println(" *C");

    Serial.print(F("Approx altitude BMP280 = "));
    Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
    Serial.println(" m");

    Serial.print("Temperature AHT20: ");
    Serial.print(temp.temperature);
    Serial.println(" *C");
    
    Serial.print("Humidity AHT20: ");
    Serial.print(humidity.relative_humidity);
    Serial.println(" rH %");

    Serial.println();
    
    delay(2000);  //Update all values every 2 Seconds
}
