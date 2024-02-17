/*
  UART2WiFi.ino creates a WiFi access point, waits for WiFi connection and then forwards UART input to the connection.
  ESP32s3 Arduino sketch, created on 2024-02-17 by DrMotor for NiNo-racing
*/
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
const char *ssid = "YourAP";   // Set these to your desired credentials.
const char *password = "YourPass";  // A valid password must have more than 7 characters

WiFiServer server(333);
void setup() {
  // Using a USB (Serial) to PC for debug messages
  Serial.begin(115200);
  Serial.print("\nConfiguring access point\n");
  Serial1.begin(1000000,SERIAL_8N1,17,18);  // GPIO pins: RX1=17, TX1=18 ?  only RX is used in this sketch

  if (!WiFi.softAP(ssid, password)) {
    log_e("Soft AP creation failed.");
    while(1);
  }
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address:");
  Serial.println("myIP");
  server.begin();
  Serial.print("Server started\n");
}

void loop() {
  WiFiClient client = server.available();   // listen for incoming clients
  if (client) {                             // if you get a client,
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
          //Serial.write(sbuf, len); // echo to serial (USB) termainal for debug 
        }
    }
    // close the connection:
    client.stop();
    Serial.print("Client Disconnected.\n");
  }
}
