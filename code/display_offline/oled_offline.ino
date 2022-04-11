
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
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
  display.println("Servuuuus");

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
display.print("Nadine ist cool!");
display.setTextSize(2);
display.print("Soll ");
display.println(shall_temp);
display.print("Ist ");
display.println(current_temp);

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
