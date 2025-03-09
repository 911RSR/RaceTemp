/*
  This sketch creates a WiFi access point, waits for WiFi connection to a client and then sends RC3 messages to that client
  ESP32s3 Arduino sketch, created on 2024-02-17 by DrMotor for NiNo-racing
*/
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include <SPI.h>
//#include <esp_adc_cal.h>
#include <esp_adc/adc_cali.h>
#include "cred.h" //WiFi ssid and password
#include "NTC.h"
#define NEOPIX_BRIGHTNESS 8

const std::uint16_t port = 333;
std::uint16_t RC3_count = 0; // counting RC3 messages 0 to 0xffff overflowing
char rc3Message[300]; // text buffer for writing/storing RC3 messages
WiFiServer server(port); 
RTC_DATA_ATTR int bootCount = 0;

// The SPI pinout description for the ESP32S3 is fantastic... but is it readable?
// Easier pinout info: www.luisllamas.es/en/esp32-s3-hardware-details-pinout/ 
// reveals the default pins for SPI3:  MOSI=GPIO11, MISO=GPIO13, CLK=GPIO12, CS=GPIO10
// Some pins can be changed/remapped.  See SPI examples! 
#define VSPI FSPI
struct MAX31855{
  const int CS_Pin;
  SPIClass * spi; 
  union {
    std::uint32_t u32;
    std::int32_t i32;   // [1/2^20 degC] TC = thermocouple.  Note: Only 14 MSB contain usefull data -- but the rest vanishes in small decimals
    struct{
      std::int16_t ic_temp;  // [1/256 degC] IC = the chip.  Note: 4 LSBs are "reserved" and error flags 
      std::int16_t tc_temp;  // [1/16 K, delta from chip] TC = thermocouple.  Note: 1 LSB is "reserved"
    };
  };
};

struct MAX31855 EGT = { 
  .CS_Pin = GPIO_NUM_5, // We will connect the chip select to GPIO5. 
  .spi = new SPIClass( VSPI ) 
};  

struct NTC_IO{
  const adc_unit_t ADC_dev;
  const gpio_num_t Vref; 
  const adc_channel_t ADC_ch;
  const gpio_num_t GND;
};

struct NTC_IO Water_IO = {
  .ADC_dev = ADC_UNIT_1, 
  .Vref = GPIO_NUM_7,
  .ADC_ch = ADC_CHANNEL_6, 
  .GND = GPIO_NUM_8
};

NTC WaterTemp;

// Setup function for the EGT
void setup( struct MAX31855 *a ){
  Serial.print("\nConfiguring EGT sensor\n");
  //pinMode(GPIO_NUM_13, INPUT_PULLUP );  // does this help or work?
  pinMode(a->CS_Pin, OUTPUT);
  digitalWrite(a->CS_Pin, HIGH);
  a->spi->begin(SCK, MISO, MOSI, a->CS_Pin);
}

void setup_WiFi(){
  Serial.print("\nConfiguring access point\n");
  //Serial1.setRxBufferSize(1024);  // Serial 1 and pins 17 and 18 will be used for input
  //Serial1.begin(230400,SERIAL_8N1,18,17);  //  only RX is used in this sketch
  // RX1 = U1RXD = connector J1 pin 11, name "18" according to devkitc user guide
  // https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/hw-reference/esp32s3/user-guide-devkitc-1.html 
  // Same pin according to https://github.com/vcc-gnd/YD-ESP32-S3 
  if (!WiFi.softAP(ssid, password)) 
  {
    log_e("Soft AP creation failed.");
    while(1);
  }
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address:");
  Serial.println( myIP );
  Serial.print("Data port: ");
  Serial.println( port );
  Serial.print("startup WiFi TxPower: ");
  Serial.println( WiFi.getTxPower() );  
  //WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.setTxPower(WIFI_POWER_5dBm); // lower power --> less noisy thermocouple output, or ?
  delay(100);
  Serial.print("new WiFi TxPower: ");
  Serial.println( WiFi.getTxPower() );  
  server.begin();
  Serial.print("Server started\n");
}

void setup( NTC_IO *a ) {
  Serial.print("\nConfiguring NTC sensor\n");

  // We will output Vref to a GPIO pin and connect a pull-up resistor from this pin to the NTC 
  // esp_err_t status = adc_vref_to_gpio( a->ADC_dev, a->Vref );
  //if (status == ESP_OK) Serial.println("Vref routed to GPIO");
  //else Serial.println("Failed to route Vref");

  //pinMode( a->GND, OUTPUT );
  //digitalWrite( a->GND, LOW );
  WaterTemp.R0=4700.0f;
	WaterTemp.T0=29.0 + 273.15; // should it be 25 degC ?
	WaterTemp.Beta=3650.0;
	WaterTemp.Rs=1000.0;
}

void setup() {
  neopixelWrite(RGB_BUILTIN,0,NEOPIX_BRIGHTNESS,0); // Green light for startup
  Serial.begin(115200);    // Using a USB (Serial) to PC for debug messages
  delay( 100 );
  Serial.println(__FILE__);
  Serial.print("Boot number: ");
  Serial.println(++bootCount);  //Increment boot number and print it
  delay( 250 );
  setup( &EGT );
  setup( &Water_IO );
  setup_WiFi();

  /*// for wakeup
  pinMode(GPIO_NUM_33, INPUT_PULLDOWN );
  esp_sleep_enable_ext0_wakeup( GPIO_NUM_33 , 1); //1 = High, 0 = Low
  //esp_deep_sleep_start();
  */
}

uint8_t nmea_checksum(const char *s) {  // s should not include the leading $-sign and the ending *-sign
    int c = 0;
    while(*s) c ^= *s++;
    return c;
}

float read( struct MAX31855 * a ) {
  a->spi->beginTransaction( SPISettings( 1000000, MSBFIRST, SPI_MODE0)  ); // max 5MHz clock, ref. MAX61855 datasheet
  digitalWrite( a->CS_Pin, LOW ); // is a delay needed after this to ensure tCSS > 100 ns, ref. MAX61855 datasheet  ?
  delay(1);
  a->u32 = a->spi->transfer32( a->u32 ); 
  //Serial.println( a->u32 );          // for debug
  digitalWrite( a->CS_Pin, HIGH );
  a->spi->endTransaction();
  return 953.6743164e-9f * a->i32; // [degC]   magic = 1/2^20 
}

void RC3(){
  float EGT_degC = read( &EGT );
  uint16_t Water_raw = analogRead( Water_IO.ADC_ch );
  // RaceChrono output in $RC3 format ------------------------------------------------------------------------ 
  /* $RC3,[time],[count],[xacc],[yacc],[zacc],[gyrox],[gyroy],[gyroz],[rpm/d1],[d2],
     [a1],[a2],[a3],[a4],[a5],[a6],[a7],[a8],[a9],[a10],[a11],[a12],[a13],[a14],[a15]*checksum
     - $ is message start character
     - RC2 and RC3 are message identifiers
     - time stamp is not used (empty). (for blended GNS support this should be a GNS synchronized realtime timestamp in NMEA 0183 format).
     - count is an overflowing line counter 0-65535. Can be left empty if GNS timestamp is provided.
     - acc fields: -1.000 = -1G, 1.000 = +1G
     - gyro fields: degrees per second, -1.000 = -1 deg/s, 1.000 = +1 deg/s
     - dx are digital channel fields, range -2000000.000 - 2000000.000
     - ax are analog channel fields, range -2000000.000 - 2000000.000
     - * is message separator character
     - NMEA 0183 type checksum, with two uppercase hexadecimal digits (one byte)
     - each line is terminated with CR plus LF
   */

  /*sprintf(rc3Message,"$RC3,,%lu,,,,,,,,,%1.1f,%1.1f,%1.1f,,,,,,,,,,,,",
    RC3_count++, // $RC3,time,count[-]
    EGT_degC, // 
    Water_raw/4095.0f*3.3f, // a2 
    WaterTemp.degC( Water_raw ) // a3
  );*/

  // $RC2,[time],[count],[xacc],[yacc],[zacc],[rpm/d1],[d2],[a1],[a2],[a3],[a4],[a5],[a6],[a7],[a8]*checksum
  sprintf(rc3Message,"$RC2,,%lu,,,,,,%1.1f,%1.1f,,,,,,",
    RC3_count++, // $RC3,time,count[-]
    EGT_degC, // 
    WaterTemp.degC( Water_raw ) // a3
  );
  
  sprintf(rc3Message + strlen(rc3Message), "*%02X\r\n", nmea_checksum(rc3Message+1));
  Serial.print(rc3Message);          // print the rc message to serial port for debug
}

void loop() {
  WiFiClient client = server.accept();      // listen for incoming clients
  if (client) {                             // if you get a client,
    Serial.print("New Client.\n");          // print a message out the serial port
    unsigned long nextRC3 = millis() + 200;
    neopixelWrite(RGB_BUILTIN,NEOPIX_BRIGHTNESS,0,NEOPIX_BRIGHTNESS); // purple LED
    while (client.connected())  // loop while the client's connected
    {
      if ( millis() > nextRC3 ){
        RC3(); // write a new RC3 message into the buffer
        client.write(rc3Message, strlen(rc3Message)); // Send the RC message over the WiFi connection
        nextRC3 += 40;
      } 
    }
    client.stop();  // close the connection
    neopixelWrite(RGB_BUILTIN,0,NEOPIX_BRIGHTNESS,0); // green led
    Serial.print("Client Disconnected.\n");
  }
  // ToDo: Update RC3 message count also when not connected -- to indicate loss of messages
  // ToDo: RPM and sleep/wake-up, implement and test
  // ToDo: Water temperature, connect, test & debug
}
