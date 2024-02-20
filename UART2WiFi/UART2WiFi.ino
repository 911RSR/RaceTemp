/*
  UART2WiFi.ino creates a WiFi access point, waits for WiFi connection and then forwards UART input to the connection.
  ESP32s3 Arduino sketch, created on 2024-02-17 by DrMotor for NiNo-racing
*/
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
const char *ssid = "yourAP";   // Set these to your desired credentials.
const char *password = "yourPass";  // A valid password must have more than 7 characters
const uint16_t port = 333;
#define RGB_BRIGHTNESS 8

WiFiServer server(port);
void setup() {
  neopixelWrite(RGB_BUILTIN,0,RGB_BRIGHTNESS,0); // Green light for starup
  Serial.begin(115200);    // Using a USB (Serial) to PC for debug messages
  Serial.print("\nConfiguring access point\n");
  Serial1.setRxBufferSize(1024);
  Serial1.begin(230400,SERIAL_8N1,18,17);  //  only RX is used in this sketch
  // RX1 = U1RXD = connector J1 pin 11, name "18" according to devkitc user guide
  // https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/hw-reference/esp32s3/user-guide-devkitc-1.html 
  if (!WiFi.softAP(ssid, password)) {
    log_e("Soft AP creation failed.");
    while(1);
  }
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address:");
  Serial.println( myIP );
  Serial.print("Data port: ");
  Serial.println( port );
  server.begin();
  Serial.print("Server started\n");
}

void loop() {
  WiFiClient client = server.available();   // listen for incoming clients
  if (client) {                             // if you get a client,
    Serial.print("New Client.\n");          // print a message out the serial port
    while (client.connected())  // loop while the client's connected
    {            
        if( Serial1.available() )
        {
          size_t len = Serial1.available();
          uint8_t sbuf[len];
          Serial1.readBytes(sbuf, len);  // read data from uart
          client.write(sbuf, len);  // push data to the client
          Serial.write(sbuf, len); // to serial (USB) for debug
          neopixelWrite(RGB_BUILTIN,RGB_BRIGHTNESS,0,RGB_BRIGHTNESS); // LED on
        } else
        {
          neopixelWrite(RGB_BUILTIN,0,0,0); // no LED = no data
        }
    }

    client.stop();  // close the connection
    neopixelWrite(RGB_BUILTIN,0,RGB_BRIGHTNESS,0); // green led
    Serial.print("Client Disconnected.\n");
  }
}
