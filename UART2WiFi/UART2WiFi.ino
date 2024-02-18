/*
  UART2WiFi.ino creates a WiFi access point, waits for WiFi connection and then forwards UART input to the connection.
  ESP32s3 Arduino sketch, created on 2024-02-17 by DrMotor for NiNo-racing
*/
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
const char *ssid = "yourAP";   // Set these to your desired credentials.
const char *password = "yourPass";  // A valid password must have more than 7 characters
volatile uint8_t toggle=0;

WiFiServer server(333);
void setup() {
  digitalWrite(LED_BUILTIN,1);  // Flash the LED to show activity
  delay(100);
  digitalWrite(LED_BUILTIN,0);
  // Using a USB (Serial) to PC for debug messages
  Serial.begin(115200);
  Serial.print("\nConfiguring access point\n");
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
  server.begin();
  Serial.print("Server started\n");
}

void loop() {
  WiFiClient client = server.available();   // listen for incoming clients
  if (client) {                             // if you get a client,
    digitalWrite(LED_BUILTIN,1);
    Serial.print("New Client.\n");          // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected())  // loop while the client's connected
    {            
        if(Serial1.available())
        {
          size_t len = Serial1.available();
          uint8_t sbuf[len];
          Serial1.readBytes(sbuf, len);
          //push UART data to all connected client
          client.write(sbuf, len);
          //Serial.write(sbuf, len); // to serial (USB) for debug
          toggle = (toggle==1) ? 0:1; 
          digitalWrite(LED_BUILTIN, toggle );  // toggle the LED
        }
    }
    // close the connection:
    client.stop();
    digitalWrite(LED_BUILTIN,0);
    Serial.print("Client Disconnected.\n");
  }
}
