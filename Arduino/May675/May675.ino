// Include the necessary libraries to set up the WiFi access point
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>

// Define the credentials for the WiFi access point
const char *ssid = "RaceChronoESP32";
const char *password = "yourPass";
const uint16_t port = 333;

// Create a server object on the specified port
WiFiServer server(port);

// Declare the 'count' variable that will be used in the RC3 messages
unsigned long count = 0;
char rc3Message[128];

void setup() {
  // Start serial communication at 115200 bits per second
  Serial.begin(115200);

  // Configure the WiFi access point
  Serial.print("\nConfiguring access point\n");
  if (!WiFi.softAP(ssid, password)) {
    log_e("Soft AP creation failed.");
    while(1);
  }

  // Print the IP address of the access point
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address:");
  Serial.println(myIP);

  // Print the data port
  Serial.print("Data port: ");
  Serial.println(port);

  // Start the server
  server.begin();
  Serial.print("Server started\n");
}

// NMEA 0183 defines a max message length of 82, including $ *checksum CR and LF
#define NMEA_MAX_LENGTH 82
static char nmea_buf[NMEA_MAX_LENGTH];
uint8_t nmea_checksum(const char *s) {  // s should not include the leading $-sign and the ending *-sign
    int c = 0;
    while(*s) c ^= *s++;
    return c;
}

void May(){
  // Read the analog value of the rear potentiometer (from 0 to 1023)
  int valorPotenciometroTR = analogRead(33);

  // Read the analog value of the front potentiometer (from 0 to 1023)
  int valorPotenciometroFRT = analogRead(34);

  // Map the potentiometer value to the desired range (0.08V to 3.42V -> 0cm to 12.5cm)
  float voltajeTR = valorPotenciometroTR * (5.0 / 1023.0);
  float longitudEnCmTR = map(voltajeTR * 100, 1, 2000, 6, 125) / 10.0;
  float voltajeFRT = valorPotenciometroFRT * (5.0 / 1023.0);
  float longitudEnCmFRT = map(voltajeFRT * 100, 1, 2000, 6, 125) / 10.0;

  // Print the voltage and length in centimeters
  Serial.print(voltajeTR, 2); // Print with 2 decimals
  Serial.print("TR "); // name of the rear potentiometer
  Serial.println(longitudEnCmTR, 2); // Print with 2 decimals
  Serial.print(voltajeFRT, 2); // Print with 2 decimals
  Serial.print("FRT "); // name of the front potentiometer
  Serial.println(longitudEnCmFRT, 2); // Print with 2 decimals

  // Generate and send an RC3 message
  sprintf(rc3Message,"$RC3,,%lu,,,,,,,%1.1f,%1.1f,,,,,,,,,,,,,,,,,",
  //sprintf(rc3Message, "$RC3,,%lu,,,,,,,%1.1f,%1.1f",
    count++, // $RC3,time,count[-]
    longitudEnCmTR, // a1 [cm] = length of the rear potentiometer
    longitudEnCmFRT // a2 [cm] = length of the front potentiometer
  );
  sprintf(rc3Message + strlen(rc3Message), "*%02X\r\n", nmea_checksum(rc3Message+1));
}

void loop() {
  // Check if any client is available and handle it
  WiFiClient client = server.available();
  if (client) {
    Serial.print("New Client.\n");
    unsigned long nextMay = millis() + 100;
    while (client.connected()) {
      /*if (Serial1.available()) {
        size_t len = Serial.available();
        uint8_t sbuf[len];
        Serial.readBytes(sbuf, len);
        client.write(sbuf, len);
        Serial.write(sbuf, len);
      }*/
      if ( millis() > nextMay ){
        May();
        Serial.print("millis: "); 
        Serial.println( millis() ); 
        Serial.print("nextMay: "); 
        Serial.println( nextMay ); 
        // Send the RC3 message over the WiFi connection
        //client.println(rc3Message);
        client.write(rc3Message, strlen(rc3Message));
        nextMay+=100;
      }
    }
    client.stop();
    Serial.print("Client Disconnected.\n");
  }
}