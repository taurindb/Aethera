/***********************************************************
  * AetheraSerial.ino
  *
  * Description:
  *   The Arduino part of the Aethera project.
  *   Read sensor values, output them through the serial
  * Create Date: 2014/10
  * Change Log:  Inital version - seeedstudio
  *
***********************************************************/
/* Including files */
#include <DHT.h>
#include <Wire.h>
#include <Digital_Light_TSL2561.h>

/* Sensor pin */
#define pin_uv              A2      // UV  sensor
#define pin_sound           A0      // Sound sensor
#define pin_dust            7       // Dust sensor
#define pin_air_quality     A1      // Air quality sensor
#define pin_dht             4       // Humidity and temperature sensor
#define DHTTYPE             DHT22   // DHT 22  (AM2302)

//digital light sensor      I2C

#define SER_PUSH      Serial        //change to Serial1 for pushing to openWrt side through bridge

/* sample function */
float iReadTemperature(void);
float iReadHumidity(void);
float iReadDensityDust(void);
unsigned long iReadLux(void);
float iReadUVRawVol(void);
int iReadSoundRawVol(void);

/* dust variables */
unsigned long dust_starttime;
unsigned long duration_starttime;
unsigned long duration;
unsigned long sampletime_ms = 30000;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = -1;
int sensorValue=0;

/* temperature and humidity sensor */
DHT dht(pin_dht, DHTTYPE);

/* sound sensor */
float sound_raw_vol_max = 0.0f;  //mV

/* Air Quality */
enum
{
  AQ_INIT, AQ_WAIT_INIT, AQ_WORK
};
enum
{
  AQ_HIGH_POLLUTION, AQ_POLLUTION, AQ_LOW_POLLUTION, AQ_FRESH, AQ_WARMUP
};
unsigned long air_quality_sensor_init_starttime;
int air_quality_sensor_state = AQ_WAIT_INIT;
unsigned long air_quality_sensor_init_time = 200000;
int aq_first_vol, aq_last_vol, aq_std_vol, aq_std_vol_sum;
static int cntr_aq_avg = 0;
int aq_result = AQ_WARMUP;


/* Global varibles */
boolean valid_dust = false;
unsigned long push_starttime;
unsigned long push_interval = 1000;  //ms




//*****************************************************************************
//
//! \brief setup
//!
//! \param[in]
//! \param[in]
//!
//! \return
//!
//*****************************************************************************
void setup() {

  // for debugging, wait until a serial console is connected
  Serial.begin(115200);    //Serial for debugging
  Serial1.begin(115200);   //Serial1 for outputing data to OpenWrt, serial-bridge
  delay(2000);  //wait usb serial to be ready

  /* Wire Begin */
  Wire.begin();

  Serial.println(F("******Aethera Sensor Node******\r\n"));

  /* Initialize temperature and humidity sensor */
  Serial.println(F("Initialize temper humidity sensor...\r\n"));
  dht.begin();

  /* Initialize Dust sensor */
  digitalWrite(pin_dust, HIGH);
  pinMode(pin_dust,INPUT);
  attachInterrupt(0, dust_interrupt, CHANGE);

  /* Digital Light Sensor */
  TSL2561.init();

  /* Sound Sensor */
  pinMode(pin_sound, INPUT);

  /* UV Sensor */
  pinMode(pin_uv, INPUT);

  /* Air Quality */
  pinMode(pin_air_quality, INPUT);
  air_quality_sensor_init_starttime = millis();


  /* other */
  push_starttime = millis();
  init_timer1(200);  //us, get the value based on Shannon's law, sound freq: 0~3400Hz
}


//*****************************************************************************
//
//! \brief loop
//!
//! \param[in]
//! \param[in]
//!
//! \return
//!
//*****************************************************************************
void loop()
{
  //real time task
  air_quality_state_machine();

  //slow task
  push_data();
}

//*****************************************************************************
//
//! \brief push data to OpenWrt side
//!
//!
//! \return void
//!
//*****************************************************************************
void push_data()
{
  if(millis() - push_starttime > push_interval)
  {
    push_starttime = millis();

    SER_PUSH.print(F("/Visible Light/"));
    SER_PUSH.print(iReadLux());  //>100ms , lux
    SER_PUSH.print(",");

    SER_PUSH.print(F("/Temperature/"));
    SER_PUSH.print(iReadTemperature());  //>250ms, F
    SER_PUSH.print(",");

    SER_PUSH.print(F("/Humidity/"));
    SER_PUSH.print(iReadHumidity());  //>250ms, %
    SER_PUSH.print(",");

    SER_PUSH.print(F("/Particulate Density/"));
    SER_PUSH.print(iReadDensityDust());  // ~ 0ms, pcs/0.01cf or pcs/283ml
    SER_PUSH.print(",");

    SER_PUSH.print(F("/UV Raw/"));
    SER_PUSH.print(iReadUVRawVol());  // > 128ms, mV
    SER_PUSH.print(",");

    SER_PUSH.print(F("/Sound Raw/"));
    SER_PUSH.print(iReadSoundRawVol());  // ~0ms, mV
    SER_PUSH.print(",");

    SER_PUSH.print(F("/Air Quality/"));  //~0ms
    switch (aq_result)
    {
      case AQ_WARMUP:
        SER_PUSH.print(F("WarmUp")); break;
      case AQ_FRESH:
        SER_PUSH.print(F("Fresh")); break;
      case AQ_LOW_POLLUTION:
        SER_PUSH.print(F("LowPollution")); break;
      case AQ_POLLUTION:
        SER_PUSH.print(F("Pollution")); break;
      case AQ_HIGH_POLLUTION:
        SER_PUSH.print(F("HighPollution")); break;
    }

    SER_PUSH.println();  //sum ~ 800ms / push
  }
}

/* Interrupt service */
void dust_interrupt()
{
  if (digitalRead(pin_dust) == 0)  //fall
  {
    duration_starttime = millis();
  }else
  {
    duration = millis() - duration_starttime;
    if (duration > 10 && duration < 200)
    {
      lowpulseoccupancy+=duration;
    }
    if ((millis()-dust_starttime) > sampletime_ms)
    {
      valid_dust = true;
      ratio = lowpulseoccupancy/(sampletime_ms*10.0);  // Integer percentage 0=>100
      concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve
      lowpulseoccupancy = 0;
      dust_starttime = millis();
    }
  }

}

/* Timer1 Service */
static int cntr_aq_sample = 0;
ISR(TIMER1_OVF_vect)
{
  int snd_raw = analogRead(pin_sound);
  float snd_raw_mv = snd_raw * (4980.0 / 1023.0);
  if (snd_raw_mv > sound_raw_vol_max) sound_raw_vol_max = snd_raw_mv;

  if (air_quality_sensor_state == AQ_WORK && ++cntr_aq_sample == 10000)  //200us * 10000 = 2s
  {
    cntr_aq_sample = 0;
    //Serial.println("tm1 isr");
    air_quality_sensor_evaluation();
  }
}


//*****************************************************************************
//
//! \brief push data to OpenWrt side
//!
//!
//! \return void
//!
//*****************************************************************************
void air_quality_state_machine()
{
  switch (air_quality_sensor_state)
  {
    case AQ_WAIT_INIT:
      {
        if (millis() - air_quality_sensor_init_starttime > air_quality_sensor_init_time)
        {
          air_quality_sensor_state = AQ_INIT;
        }
        break;
      }
    case AQ_INIT:
      {
        int v = analogRead(pin_air_quality);
        if (v < 798 && v > 10)  //the init voltage is ok
        {
          //Serial.print("init:");
          //Serial.println(v);

          aq_first_vol = v;
          aq_last_vol = v;
          aq_std_vol = v;
          aq_std_vol_sum = 0;
          air_quality_sensor_state = AQ_WORK;
        }
        else
        {
          air_quality_sensor_init_starttime = millis();
          air_quality_sensor_state = AQ_WAIT_INIT;
        }
        break;
      }
    case AQ_WORK:
      {
        break;
      }
    default:
      break;
  }
}

//*****************************************************************************
//
//! \brief do a window average
//!
//!
//! \return void
//!
//*****************************************************************************
void air_quality_sensor_window_avg()
{
  if (++cntr_aq_avg >= 150)  //sum for 5 minutes
  {
    cntr_aq_avg = 0;
    aq_std_vol = aq_std_vol_sum / 150;
    aq_std_vol_sum = 0;
  } else
  {
    aq_std_vol_sum += aq_first_vol;
  }
}

//*****************************************************************************
//
//! \brief do a window average
//!
//!
//! \return void
//!
//*****************************************************************************
void air_quality_sensor_evaluation()
{
  aq_last_vol = aq_first_vol;
  aq_first_vol = analogRead(pin_air_quality);

  //Serial.println(aq_std_vol);
  //Serial.println(aq_first_vol);

  if (aq_first_vol - aq_last_vol > 400 || aq_first_vol > 700)
  {
    aq_result = AQ_HIGH_POLLUTION;
  } else if ((aq_first_vol - aq_last_vol > 400 && aq_first_vol < 700) || aq_first_vol - aq_std_vol > 150)
  {
    aq_result = AQ_POLLUTION;
  } else if ((aq_first_vol - aq_last_vol > 200 && aq_first_vol < 700) || aq_first_vol - aq_std_vol > 50)
  {
    aq_result = AQ_LOW_POLLUTION;
  } else
  {
    aq_result = AQ_FRESH;
  }
  air_quality_sensor_window_avg();
}

//*****************************************************************************
//
//! \brief setup the time 1 of 32u4
//!
//! \param[in]
//! \param[in]
//!
//! \return  voltage mV
//!
//*****************************************************************************
#define RESOLUTION 65536    // Timer1 is 16 bit
void init_timer1(long us)
{
  TCCR1A = 0;                 // clear control register A
  TCCR1B = _BV(WGM13);        // set mode as phase and frequency correct pwm, stop the timer

  long cycles;
  long microseconds = us;   //setup microseconds here
  unsigned char clockSelectBits;
  cycles = (F_CPU / 2000000) * microseconds;                                // the counter runs backwards after TOP, interrupt is at BOTTOM so divide microseconds by 2
  if (cycles < RESOLUTION)              clockSelectBits = _BV(CS10);              // no prescale, full xtal
  else if ((cycles >>= 3) < RESOLUTION) clockSelectBits = _BV(CS11);              // prescale by /8
  else if ((cycles >>= 3) < RESOLUTION) clockSelectBits = _BV(CS11) | _BV(CS10);  // prescale by /64
  else if ((cycles >>= 2) < RESOLUTION) clockSelectBits = _BV(CS12);              // prescale by /256
  else if ((cycles >>= 2) < RESOLUTION) clockSelectBits = _BV(CS12) | _BV(CS10);  // prescale by /1024
  else        cycles = RESOLUTION - 1, clockSelectBits = _BV(CS12) | _BV(CS10);  // request was out of bounds, set as maximum

  ICR1 = cycles;
  TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
  TCCR1B |= clockSelectBits;                                          // reset clock select register, and starts the clock

  TIMSK1 = _BV(TOIE1);
  TCNT1 = 0;
  sei();                      //enable global interrupt
}

//*****************************************************************************
//
//! \brief Read temperature
//!
//! cost time: > 250ms
//!
//! \param[in]
//! \param[in]
//!
//! \return  temperature
//! \refer to http://www.seeedstudio.com/wiki/File:Humidity_Temperature_Sensor_pro.zip
//*****************************************************************************
float iReadTemperature(void) {
    float temper;
    temper = dht.readTemperature(false); //true: get F, false: get oC
    return temper;
}
//*****************************************************************************
//
//! \brief Read Humidity
//!
//! cost time: > 250ms
//!
//! \param[in]
//! \param[in]
//!
//! \return  Humidity
//!
//*****************************************************************************
float iReadHumidity(void) {
    float humidity;
    humidity = dht.readHumidity();
    return humidity;
}

//*****************************************************************************
//
//! \brief Read Dust Density
//!
//! cost time: ~0ms , unit: pcs/0.01cf or pcs/283ml
//!
//! \param[in]
//! \param[in]
//!
//! \return  temperature
//!
//*****************************************************************************
float iReadDensityDust(void) {
    return concentration;
}

//*****************************************************************************
//
//! \brief Read Lux value of visible light
//!
//! \param[in]
//! \param[in]
//!
//! \return  Luminance
//!
//*****************************************************************************
unsigned long iReadLux(void) {
    //cost time: > 100ms
    return TSL2561.readVisibleLux();
}
//*****************************************************************************
//
//! \brief Read the raw voltage signal of UV sensor
//!
//! \param[in]
//! \param[in]
//!
//! \return  voltage mV
//!
//*****************************************************************************
float iReadUVRawVol(void) {
    unsigned long sum=0;
    for(int i=0; i<128; i++)
    {
      sum += analogRead(pin_uv);
      delay(1);
    }
    sum >>= 7;
    return sum*(4980.0f/1023.0f);
}

//*****************************************************************************
//
//! \brief Read the raw voltage signal of sound sensor
//!
//! \param[in]
//! \param[in]
//!
//! \return  voltage mV
//!
//*****************************************************************************
int iReadSoundRawVol()
{
  int tmp = sound_raw_vol_max;
  sound_raw_vol_max = 0;
  return tmp;
}

