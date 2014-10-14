#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_SI1145.h>
#include <DHT.h>
#define DHTPIN 5  // Temp/Humidity connected to Digital Pin 5
#define DHTTYPE DHT11   // DHT 22  (AM2302)

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
Adafruit_SI1145 uv = Adafruit_SI1145();

DHT dht(DHTPIN, DHTTYPE);

//Electret mic is connected to A0
int electretPin = 0;  
int electretReading;

//Sharp Dust sensor is connected to A1
int particulatePin = 1;  
int particulateReading;     
int ledPowerPin=6;       //Wire 3 from the Sharp sensor is connected to D6
//Manage the particulate sensor
int delayTime=280;
int delayTime2=40;
float offTime=50680;
//Calculate particulate information
int i=0;
float ppm=0;
char s[32];
float voltage = 0;
float dustdensity = 0;
float ppmpercf = 0;
float dustdensityMax=0;
float ppmMax = 0;  


int airqualityPin = 2;  ////MQ-135  is connected to A2
float airqualityReading;  

void setup() {
  Serial.begin(57600); 
   // Serial.println("Aethera!");
   
   //Driver for Sharp Dust Sensor LED
   pinMode(ledPowerPin,OUTPUT);
  
    //Begin I2C Sensor Readings, both sensors are connected to SDA and SCL pins
   if(! bmp.begin())  
  {
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
   if (! uv.begin()) {
    Serial.print("Ooops, no Si1145 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  
  dht.begin();
  //initialize timer and particulate reading
  i=0;
  ppm=0;
}

void loop() {

  electretReading = analogRead(electretPin);
  airqualityReading = analogRead(airqualityPin); 
 
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float humidity = dht.readHumidity();
  // Read temperature as Celsius
  float tempC = dht.readTemperature();
  // Read temperature as Fahrenheit
  float tempF = dht.readTemperature(true);
  
  // Check if any reads failed and exit early (to try again).
  if (isnan(humidity) || isnan(tempC) || isnan(tempF)) {
    Serial.print("000"); //000 indicates a failure in the data and on the graph. 
    Serial.print(",");
    return;
  }

  // Compute heat index
  // Must send in temp in Fahrenheit!
  float heatIndex = dht.computeHeatIndex(tempF, humidity);

  // the index is multiplied by 100 so to get the integer index, divide by 100!
  float uvIndex = uv.readUV();
  uvIndex /= 100.0;  
  
 //Serial.print("/Visible Light/"); 
  Serial.print(uv.readVisible());
  Serial.print(",");  
 //Serial.print("/Sound/");
  Serial.print(electretReading);
  Serial.print(","); 
 //Serial.print("/AirQuality/");
  Serial.print(airqualityReading);
  Serial.print(","); 
 //Serial.print("/Temperature/"); 
  Serial.print(tempF);
  Serial.print(","); 
  //Serial.print("/Humidity/"); 
  Serial.print(humidity);
  Serial.print(","); 
 //Serial.print("/HeatIndex/");
  Serial.print(heatIndex);
  Serial.print(","); 
 // Serial.print("/UV Index/: ");  Serial.print(","); 
  Serial.print(uvIndex);
  Serial.print(",");  
  
  sensors_event_t event;
  bmp.getEvent(&event);
 
  /* Display the results (barometric pressure is measure in hPa) */
  if (event.pressure)
  {
    /* Display atmospheric pressue in hPa */
    //Serial.print("/Pressure/");
    Serial.print(event.pressure);
    Serial.print(",");  
    //  Serial.println(" hPa");
    /* Get the current temperature from the BMP085, only for calculating Pressure */
    float temperature;
    bmp.getTemperature(&temperature);
  }
  else
  {
    Serial.println("Sensor error");
  }
  
 
  //Sharp Code
  i=i+1;
  digitalWrite(ledPowerPin,LOW); // power on the LED
  delayMicroseconds(delayTime);
    particulateReading=analogRead(particulatePin); // read the dust value
    ppm = ppm+particulateReading;
    // Serial.println(particulateReading);
  delayMicroseconds(delayTime2);
  digitalWrite(ledPowerPin,HIGH); // turn the LED off
  delayMicroseconds(offTime);
  
  voltage = ppm/i*0.0049; // 5V / 1024 = 0.0049
  dustdensity = 0.17*voltage-0.1;
  ppmpercf = (voltage-0.0256)*120000;
    if (ppmpercf < 0)
      ppmpercf = 0;
    if (ppmpercf > ppmMax) 
      ppmMax = ppmpercf;   //calculate max value over sampled time
    if (dustdensity < 0 )
      dustdensity = 0;
    if (dustdensity > 0.5)
      dustdensity = 0.5;
    if (dustdensity > dustdensityMax) 
      dustdensityMax = dustdensity;
 
    String particulateDensityString;
    String ppmString;
    particulateDensityString = dtostrf(dustdensity, 5, 2, s);
    ppmString = dtostrf(ppmpercf, 8, 0, s);
    
    i=0;
    ppm=0;   
    int dustdensityMax=0;
    int ppmMax=0;  
    
    //Serial.print("/PPM/");
    Serial.print(ppmString);
    Serial.print(",");
    // Serial.print("/Particulate Density/");
    Serial.println(particulateDensityString);
    // Serial.println();
    
    // Wait a few seconds between measurements.
    delay(offTime);
}
