// LCD library
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);

// DHT library
#include <DHT.h>;

// GPS library
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

//GPS hardware Defination
HardwareSerial mySerial = Serial1;
Adafruit_GPS GPS(&Serial1);
#define GPSECHO  true
boolean usingInterrupt = false;
void useInterrupt(boolean);

// Compass Library 
#include <QMC5883LCompass.h>

//Constants
#define DHT11_DHTPIN 2 // what pin we're connected to
#define DHTTYPE DHT22 // DHT 22 (AM2302)
#define DHTPIN 3
#define DHTTYPE DHT22 // DHT 22 (AM2302)

DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor 

//Variables
int chk;
float hum1; //Stores humidity value
float temp1; //Stores temperature value
float hum2;
float temp2;


//LCD Setup
 lcd.init();
 lcd.backlight();
 delay(250);
 lcd.noBacklight();
 delay(1000);
 lcd.backlight();
 delay(1000);

// Gas sensor setup
    //data pins defination
#define DIGITAL_PIN 4
#define ANALOG_PIN 1

// Gas detection setup.
uint16_t gasVal;
boolean isgas = false;
String gas;

void setup() {
 {
  delay (3000);
  Serial.begin(9600);
  lcd.println("Setup Begins");
  dht.begin();
  delay(2000);
  pinMode(DIGITAL_PIN, INPUT);//sets up gas sensor pin as input

 //Read data and store it to variables hum and temp
delay (2000);
temp1= dht.readTemperature();
temp2= dht.readTemperature();


  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  useInterrupt(true);
   delay(1000);
  mySerial.println(PMTK_Q_RELEASE);
 }}

  SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();

void loop() {

//GPS
  if (! usingInterrupt) {
   
    char c = GPS.read();
    
  if (GPSECHO)
  if (c) Serial.println(c);
  }
 if (GPS.newNMEAreceived()) {
 if (!GPS.parse(GPS.lastNMEA()))   
      return; 
  {
 if (timer > millis())  timer = millis();
 } 
if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer 
}



//Gas sensor reading.{
    gasVal = analogRead(ANALOG_PIN);
    isgas = digitalRead(DIGITAL_PIN);
     } 
    if (isgas) {
    gas = "No- Engine Safe";
                           }
     else {
     gas = "Yes";
}
// Compass reading
    QMC5883LCompass compass;
    Serial.begin(9600);
    compass.init();
    compass.read(); // Read compass values
    byte a = compass.getAzimuth();
}
 
  //Print temp and humidity values to serial monitor
    lcd.print("Engines Oil:");
    lcd.print(hum1);
    lcd.print(" %, Temp: ");
    lcd.print(temp1);
    lcd.println(" Celsius");

    lcd.print("Engine Exhaust:");
    lcd.print(hum2);
    lcd.print(" %, Temp: ");
    lcd.print(temp2);
    lcd.println(" Celsius");

    //Serial monitor Code
    //GPS
    {
    lcd.print("\nTime: ");
    lcd.print(GPS.hour, DEC); lcd.print(':');
    lcd.print(GPS.minute, DEC); lcd.print(':');
    lcd.print(GPS.seconds, DEC); lcd.print('.');
    lcd.println(GPS.milliseconds);
    lcd.print("Date: ");
    lcd.print(GPS.day, DEC); lcd.print('/');
    lcd.print(GPS.month, DEC); lcd.print("/20");
    lcd.println(GPS.year, DEC);
    lcd.print("Fix: "); lcd.print((int)GPS.fix);
    lcd.print(" quality: "); lcd.println((int)GPS.fixquality);}
     
    if (GPS.fix) 
    lcd.print("Location: ");
    lcd.print(GPS.latitude, 4); lcd.print(GPS.lat);
    lcd.print(", "); 
    lcd.print(GPS.longitude, 4); lcd.println(GPS.lon);
      
    lcd.print("Speed (knots): "); lcd.println(GPS.speed);
    lcd.print("Angle: "); lcd.println(GPS.angle);
    lcd.print("Altitude: "); lcd.println(GPS.altitude);
    lcd.print("Satellites: "); lcd.println((int)GPS.satellites);
    
    //Gas sensor
    gasVal = map(gasVal, 0, 1023, 0, 30);
    lcd.print("Gas detected: ");
    lcd.println(gas);
    lcd.print("Gas percentage: ");
    lcd.print(gasVal);
    lcd.print("%\n");

    // Compass
    lcd.print("Azimuth: ");
    lcd.println(a);
    byte d = compass.getBearing(a);
    // Output is a value from 0 - 15
    // based on the direction of the bearing / azimuth
    lcd.print("Direction: ");
    lcd.println(d);
    char compassLetters[3];
    compass.getDirection(compassLetters, a);
    lcd.print(compassLetters[0]);
    lcd.print(compassLetters[1]);
    lcd.println(compassLetters[2]);
   
        
  
delay (2000);
  




}
