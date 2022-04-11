
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <PID_v1.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define RELAIS_BOILER 12
#define AVG_LENGTH 50

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

int outputpin= A0;
int shall_temp = 96;
int temps[AVG_LENGTH] = {0};
float current_temp = 0;
int dots = 0;

//BREW DETECTION
bool brewing = false;

//FOR PID
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=80, Ki=0, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int WindowSize = 1000;
unsigned long windowStartTime;

const char* ssid = "MeranienDowntown";
const char* password = "WeAreTheBr0ilers";

// Wir setzen den Webserver auf Port 80
WiFiServer server(80);

// Eine Variable um den HTTP Request zu speichern
String header;

// Hier wird der aktuelle Status des Relais festgehalten


// Die verwendeted GPIO Pins
// D1 = GPIO5 und D2 = GPIO4 - einfach bei Google nach "Amica Pinout" suchen  
const int output5 = 5;
const int output4 = 4;

void setup() {
  Serial.begin(9600); 
  
  Serial.println("Testing display");
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  delay(2000);
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.clearDisplay();
  display.setCursor(0, 10);
  display.println("Connecting...");
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
  }

  Serial.println("");
  Serial.println("WLAN verbunden.");
  Serial.println("IP Adresse: ");
  Serial.println(WiFi.localIP());
  server.begin();

  pinMode(RELAIS_BOILER, OUTPUT);
  digitalWrite(RELAIS_BOILER, HIGH);

  //FOR PID
  windowStartTime = millis();

  //initialize the variables we're linked to
  Setpoint = shall_temp;

  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  brewing = false;

}

void loop()       //main loop

{


WiFiClient client = server.available();   // Hört auf Anfragen von Clients

  if (client) {                             // Falls sich ein neuer Client verbindet,
    Serial.println("Neuer Client.");          // Ausgabe auf den seriellen Monitor
    String currentLine = "";                // erstelle einen String mit den eingehenden Daten vom Client
    int cnt = 0;
    while (client.connected() && cnt < 100000) {            // wiederholen so lange der Client verbunden ist
      if (client.available()) {             // Fall ein Byte zum lesen da ist,
        char c = client.read();             // lese das Byte, und dann
        Serial.write(c);                    // gebe es auf dem seriellen Monitor aus
        header += c;
        if (c == '\n') {                    // wenn das Byte eine Neue-Zeile Char ist
          // wenn die aktuelle Zeile leer ist, kamen 2 in folge.
          // dies ist das Ende der HTTP-Anfrage vom Client, also senden wir eine Antwort:
          if (currentLine.length() == 0) {
            // HTTP-Header fangen immer mit einem Response-Code an (z.B. HTTP/1.1 200 OK)
            // gefolgt vom Content-Type damit der Client weiss was folgt, gefolgt von einer Leerzeile:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
            // Hier werden die GPIO Pins ein- oder ausgeschaltet
            if (header.indexOf("GET /inc") >= 0) {
              Serial.println("Increasing shall_temp");
              shall_temp ++;
              
            } else if (header.indexOf("GET /dec") >= 0) {
              Serial.println("Decreasing shall_temp");
              shall_temp --;
              
            } 
            
            // Hier wird nun die HTML Seite angezeigt:
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            client.println("<meta http-equiv=\"refresh\" content=\"2; url=http://192.168.0.66\" >");
            // Es folgen der CSS-Code um die Ein/Aus Buttons zu gestalten
            // Hier können Sie die Hintergrundfarge (background-color) und Schriftgröße (font-size) anpassen
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #333344; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #888899;}</style></head>");
            
            // Webseiten-Überschrift
            client.println("<body><h1>Coffee Server</h1>");
            client.println("<p>Nadine Roegl ist die coolste!</p>");
            // Zeige den aktuellen Status, und AN/AUS Buttons for GPIO 5  
            client.println("<p>Current Temperature: " +  String(current_temp) + "</p>");
            client.println("<p>Shall Temperature: " +  String(shall_temp) + "</p>");
            // wenn output5State = off, zeige den EIN Button
            client.println("<p>");       
            client.println("<a href=\"/inc\"><button class=\"button\">+</button></a>");
            client.println("<a href=\"/dec\"><button class=\"button\">-</button></a>");
            client.println("</body></html>");
            client.println("</p>"); 
            
            // Die HTTP-Antwort wird mit einer Leerzeile beendet
            client.println();
            // und wir verlassen mit einem break die Schleife
            break;
          } else { // falls eine neue Zeile kommt, lösche die aktuelle Zeile
            currentLine = "";
          }
        } else if (c != '\r') {  // wenn etwas kommt was kein Zeilenumbruch ist,
          currentLine += c;      // füge es am Ende von currentLine an
        }
      }
      cnt++;
    }
    cnt = 0;
    // Die Header-Variable für den nächsten Durchlauf löschen
    header = "";
    // Die Verbindung schließen
    client.stop();
    Serial.println("Client getrennt.");
    Serial.println("");
    
  }



  
int analogValue = analogRead(outputpin);
float millivolts = (analogValue/1024.0) * 3300; //3300 is the voltage provided by NodeMCU
current_temp = millivolts/10;
double sum = 0;
for(int i = 0; i < (AVG_LENGTH-1); i++)
{
  temps[(AVG_LENGTH-1)-i] = temps[(AVG_LENGTH-2)-i];
  sum += temps[(AVG_LENGTH-1)-i];
}
temps[0] = current_temp;
sum += temps[0];
current_temp = sum / AVG_LENGTH;
Serial.print("Current Temperature: ");
Serial.println(current_temp);

display.clearDisplay();
display.setCursor(0, 0);
display.setTextSize(1);
display.print("IP: ");
display.println(WiFi.localIP());
display.println();
display.setTextSize(2);
display.print("Soll ");
display.println(shall_temp);
display.print("Ist ");
display.println(current_temp);

/*
if(current_temp >= shall_temp)
{
  digitalWrite(RELAIS_BOILER, LOW);
}
else
{
  digitalWrite(RELAIS_BOILER, HIGH);
  display.print("Heating");
  for(int i = 0; i < dots; i++)
  {
    display.print(".");
  }
  display.println("");
  dots++;
  if(dots > 3)
  {
    dots = 0;
  }
}
*/
//Start-Booster
if(current_temp < 90)
{
  Ki = 0;
}
else
{
  Ki = 600;
}

Setpoint = shall_temp;
Input = current_temp;
myPID.Compute();

if (millis() - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }

Serial.print("Time passed: ");
Serial.print(millis() - windowStartTime);
Serial.print(" Input: ");
Serial.print(Input);
Serial.print(" Output: ");
Serial.print(Output);
Serial.print(" Setpoint: ");
Serial.println(Setpoint);

display.setTextSize(1);
display.print("Heating Power ");
display.print(int((Output/WindowSize)*100));
display.println("%");
display.print("P: ");
display.print(int(Kp));
display.print(" I: ");
display.print(int(Ki));
display.print(" D: ");
display.println(int(Kd));

if (Output < millis() - windowStartTime || current_temp > 120)
{
  digitalWrite(RELAIS_BOILER, LOW);
}
else
{
  digitalWrite(RELAIS_BOILER, HIGH);
}


display.display();


delay(100);
}
