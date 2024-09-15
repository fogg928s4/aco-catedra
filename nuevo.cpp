#pragma region DHT_header
/*!
 *  @file DHT.h
 *
 *  This is a library for DHT series of low cost temperature/humidity sensors.
 *
 *  You must have Adafruit Unified Sensor Library library installed to use this
 * class.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  Written by Adafruit Industries.
 *
 *  MIT license, all text above must be included in any redistribution
 */

#ifndef DHT_H
#define DHT_H


/* Uncomment to enable printing out nice debug messages. */
//#define DHT_DEBUG

#define DEBUG_PRINTER                                                          \
  Serial /**< Define where debug output will be printed.                       \
          */

/* Setup debug printing macros. */
#ifdef DHT_DEBUG
#define DEBUG_PRINT(...)                                                       \
  { DEBUG_PRINTER.print(__VA_ARGS__); }
#define DEBUG_PRINTLN(...)                                                     \
  { DEBUG_PRINTER.println(__VA_ARGS__); }
#else
#define DEBUG_PRINT(...)                                                       \
  {} /**< Debug Print Placeholder if Debug is disabled */
#define DEBUG_PRINTLN(...)                                                     \
  {} /**< Debug Print Line Placeholder if Debug is disabled */
#endif

/* Define types of sensors. */
static const uint8_t DHT11{11};  /**< DHT TYPE 11 */
static const uint8_t DHT12{12};  /**< DHY TYPE 12 */
static const uint8_t DHT21{21};  /**< DHT TYPE 21 */
static const uint8_t DHT22{22};  /**< DHT TYPE 22 */
static const uint8_t AM2301{21}; /**< AM2301 */

#if defined(TARGET_NAME) && (TARGET_NAME == ARDUINO_NANO33BLE)
#ifndef microsecondsToClockCycles
/*!
 * As of 7 Sep 2020 the Arduino Nano 33 BLE boards do not have
 * microsecondsToClockCycles defined.
 */
#define microsecondsToClockCycles(a) ((a) * (SystemCoreClock / 1000000L))
#endif
#endif

/*!
 *  @brief  Class that stores state and functions for DHT
 */
class DHT {
public:
  DHT(uint8_t pin, uint8_t type, uint8_t count = 6);
  void begin(uint8_t usec = 55);
  float readTemperature(bool S = false, bool force = false);
  float convertCtoF(float);
  float convertFtoC(float);
  float computeHeatIndex(bool isFahrenheit = true);
  float computeHeatIndex(float temperature, float percentHumidity,
                         bool isFahrenheit = true);
  float readHumidity(bool force = false);
  bool read(bool force = false);

private:
  uint8_t data[5];
  uint8_t _pin, _type;
#ifdef __AVR
  // Use direct GPIO access on an 8-bit AVR so keep track of the port and
  // bitmask for the digital pin connected to the DHT.  Other platforms will use
  // digitalRead.
  uint8_t _bit, _port;
#endif
  uint32_t _lastreadtime, _maxcycles;
  bool _lastresult;
  uint8_t pullTime; // Time (in usec) to pull up data line before reading

  uint32_t expectPulse(bool level);
};

/*!
 *  @brief  Class that defines Interrupt Lock Avaiability
 */
class InterruptLock {
public:
  InterruptLock() {
#if !defined(ARDUINO_ARCH_NRF52)
    noInterrupts();
#endif
  }
  ~InterruptLock() {
#if !defined(ARDUINO_ARCH_NRF52)
    interrupts();
#endif
  }
};

#endif
#pragma endregion DHT_header

#pragma region DHT_cpp
/*!
 *  @file DHT.cpp
 *
 *  @mainpage DHT series of low cost temperature/humidity sensors.
 *
 *  @section intro_sec Introduction
 *
 *  This is a library for DHT series of low cost temperature/humidity sensors.
 *
 *  You must have Adafruit Unified Sensor Library library installed to use this
 * class.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  @section author Author
 *
 *  Written by Adafruit Industries.
 *
 *  @section license License
 *
 *  MIT license, all text above must be included in any redistribution
 */


#define MIN_INTERVAL 2000 /**< min interval value */
#define TIMEOUT                                                                \
  UINT32_MAX /**< Used programmatically for timeout.                           \
                   Not a timeout duration. Type: uint32_t. */

/*!
 *  @brief  Instantiates a new DHT class
 *  @param  pin
 *          pin number that sensor is connected
 *  @param  type
 *          type of sensor
 *  @param  count
 *          number of sensors
 */
DHT::DHT(uint8_t pin, uint8_t type, uint8_t count) {
  (void)count; // Workaround to avoid compiler warning.
  _pin = pin;
  _type = type;
#ifdef __AVR
  _bit = digitalPinToBitMask(pin);
  _port = digitalPinToPort(pin);
#endif
  _maxcycles =
      microsecondsToClockCycles(1000); // 1 millisecond timeout for
                                       // reading pulses from DHT sensor.
  // Note that count is now ignored as the DHT reading algorithm adjusts itself
  // based on the speed of the processor.
}

/*!
 *  @brief  Setup sensor pins and set pull timings
 *  @param  usec
 *          Optionally pass pull-up time (in microseconds) before DHT reading
 *starts. Default is 55 (see function declaration in DHT.h).
 */
void DHT::begin(uint8_t usec) {
  // set up the pins!
  pinMode(_pin, INPUT_PULLUP);
  // Using this value makes sure that millis() - lastreadtime will be
  // >= MIN_INTERVAL right away. Note that this assignment wraps around,
  // but so will the subtraction.
  _lastreadtime = millis() - MIN_INTERVAL;
  DEBUG_PRINT("DHT max clock cycles: ");
  DEBUG_PRINTLN(_maxcycles, DEC);
  pullTime = usec;
}

/*!
 *  @brief  Read temperature
 *  @param  S
 *          Scale. Boolean value:
 *					- true = Fahrenheit
 *					- false = Celcius
 *  @param  force
 *          true if in force mode
 *	@return Temperature value in selected scale
 */
float DHT::readTemperature(bool S, bool force) {
  float f = NAN;

  if (read(force)) {
    switch (_type) {
    case DHT11:
      f = data[2];
      if (data[3] & 0x80) {
        f = -1 - f;
      }
      f += (data[3] & 0x0f) * 0.1;
      if (S) {
        f = convertCtoF(f);
      }
      break;
    case DHT12:
      f = data[2];
      f += (data[3] & 0x0f) * 0.1;
      if (data[2] & 0x80) {
        f *= -1;
      }
      if (S) {
        f = convertCtoF(f);
      }
      break;
    case DHT22:
    case DHT21:
      f = ((word)(data[2] & 0x7F)) << 8 | data[3];
      f *= 0.1;
      if (data[2] & 0x80) {
        f *= -1;
      }
      if (S) {
        f = convertCtoF(f);
      }
      break;
    }
  }
  return f;
}

/*!
 *  @brief  Converts Celcius to Fahrenheit
 *  @param  c
 *					value in Celcius
 *	@return float value in Fahrenheit
 */
float DHT::convertCtoF(float c) { return c * 1.8 + 32; }

/*!
 *  @brief  Converts Fahrenheit to Celcius
 *  @param  f
 *					value in Fahrenheit
 *	@return float value in Celcius
 */
float DHT::convertFtoC(float f) { return (f - 32) * 0.55555; }

/*!
 *  @brief  Read Humidity
 *  @param  force
 *					force read mode
 *	@return float value - humidity in percent
 */
float DHT::readHumidity(bool force) {
  float f = NAN;
  if (read(force)) {
    switch (_type) {
    case DHT11:
    case DHT12:
      f = data[0] + data[1] * 0.1;
      break;
    case DHT22:
    case DHT21:
      f = ((word)data[0]) << 8 | data[1];
      f *= 0.1;
      break;
    }
  }
  return f;
}

/*!
 *  @brief  Compute Heat Index
 *          Simplified version that reads temp and humidity from sensor
 *  @param  isFahrenheit
 * 					true if fahrenheit, false if celcius
 *(default true)
 *	@return float heat index
 */
float DHT::computeHeatIndex(bool isFahrenheit) {
  float hi = computeHeatIndex(readTemperature(isFahrenheit), readHumidity(),
                              isFahrenheit);
  return hi;
}

/*!
 *  @brief  Compute Heat Index
 *  				Using both Rothfusz and Steadman's equations
 *					(http://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml)
 *  @param  temperature
 *          temperature in selected scale
 *  @param  percentHumidity
 *          humidity in percent
 *  @param  isFahrenheit
 * 					true if fahrenheit, false if celcius
 *	@return float heat index
 */
float DHT::computeHeatIndex(float temperature, float percentHumidity,
                            bool isFahrenheit) {
  float hi;

  if (!isFahrenheit)
    temperature = convertCtoF(temperature);

  hi = 0.5 * (temperature + 61.0 + ((temperature - 68.0) * 1.2) +
              (percentHumidity * 0.094));

  if (hi > 79) {
    hi = -42.379 + 2.04901523 * temperature + 10.14333127 * percentHumidity +
         -0.22475541 * temperature * percentHumidity +
         -0.00683783 * pow(temperature, 2) +
         -0.05481717 * pow(percentHumidity, 2) +
         0.00122874 * pow(temperature, 2) * percentHumidity +
         0.00085282 * temperature * pow(percentHumidity, 2) +
         -0.00000199 * pow(temperature, 2) * pow(percentHumidity, 2);

    if ((percentHumidity < 13) && (temperature >= 80.0) &&
        (temperature <= 112.0))
      hi -= ((13.0 - percentHumidity) * 0.25) *
            sqrt((17.0 - abs(temperature - 95.0)) * 0.05882);

    else if ((percentHumidity > 85.0) && (temperature >= 80.0) &&
             (temperature <= 87.0))
      hi += ((percentHumidity - 85.0) * 0.1) * ((87.0 - temperature) * 0.2);
  }

  return isFahrenheit ? hi : convertFtoC(hi);
}

/*!
 *  @brief  Read value from sensor or return last one from less than two
 *seconds.
 *  @param  force
 *          true if using force mode
 *	@return float value
 */
bool DHT::read(bool force) {
  // Check if sensor was read less than two seconds ago and return early
  // to use last reading.
  uint32_t currenttime = millis();
  if (!force && ((currenttime - _lastreadtime) < MIN_INTERVAL)) {
    return _lastresult; // return last correct measurement
  }
  _lastreadtime = currenttime;

  // Reset 40 bits of received data to zero.
  data[0] = data[1] = data[2] = data[3] = data[4] = 0;

#if defined(ESP8266)
  yield(); // Handle WiFi / reset software watchdog
#endif

  // Send start signal.  See DHT datasheet for full signal diagram:
  //   http://www.adafruit.com/datasheets/Digital%20humidity%20and%20temperature%20sensor%20AM2302.pdf

  // Go into high impedence state to let pull-up raise data line level and
  // start the reading process.
  pinMode(_pin, INPUT_PULLUP);
  delay(1);

  // First set data line low for a period according to sensor type
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
  switch (_type) {
  case DHT22:
  case DHT21:
    delayMicroseconds(1100); // data sheet says "at least 1ms"
    break;
  case DHT11:
  default:
    delay(20); // data sheet says at least 18ms, 20ms just to be safe
    break;
  }

  uint32_t cycles[80];
  {
    // End the start signal by setting data line high for 40 microseconds.
    pinMode(_pin, INPUT_PULLUP);

    // Delay a moment to let sensor pull data line low.
    delayMicroseconds(pullTime);

    // Now start reading the data line to get the value from the DHT sensor.

    // Turn off interrupts temporarily because the next sections
    // are timing critical and we don't want any interruptions.
    InterruptLock lock;

    // First expect a low signal for ~80 microseconds followed by a high signal
    // for ~80 microseconds again.
    if (expectPulse(LOW) == TIMEOUT) {
      DEBUG_PRINTLN(F("DHT timeout waiting for start signal low pulse."));
      _lastresult = false;
      return _lastresult;
    }
    if (expectPulse(HIGH) == TIMEOUT) {
      DEBUG_PRINTLN(F("DHT timeout waiting for start signal high pulse."));
      _lastresult = false;
      return _lastresult;
    }

    // Now read the 40 bits sent by the sensor.  Each bit is sent as a 50
    // microsecond low pulse followed by a variable length high pulse.  If the
    // high pulse is ~28 microseconds then it's a 0 and if it's ~70 microseconds
    // then it's a 1.  We measure the cycle count of the initial 50us low pulse
    // and use that to compare to the cycle count of the high pulse to determine
    // if the bit is a 0 (high state cycle count < low state cycle count), or a
    // 1 (high state cycle count > low state cycle count). Note that for speed
    // all the pulses are read into a array and then examined in a later step.
    for (int i = 0; i < 80; i += 2) {
      cycles[i] = expectPulse(LOW);
      cycles[i + 1] = expectPulse(HIGH);
    }
  } // Timing critical code is now complete.

  // Inspect pulses and determine which ones are 0 (high state cycle count < low
  // state cycle count), or 1 (high state cycle count > low state cycle count).
  for (int i = 0; i < 40; ++i) {
    uint32_t lowCycles = cycles[2 * i];
    uint32_t highCycles = cycles[2 * i + 1];
    if ((lowCycles == TIMEOUT) || (highCycles == TIMEOUT)) {
      DEBUG_PRINTLN(F("DHT timeout waiting for pulse."));
      _lastresult = false;
      return _lastresult;
    }
    data[i / 8] <<= 1;
    // Now compare the low and high cycle times to see if the bit is a 0 or 1.
    if (highCycles > lowCycles) {
      // High cycles are greater than 50us low cycle count, must be a 1.
      data[i / 8] |= 1;
    }
    // Else high cycles are less than (or equal to, a weird case) the 50us low
    // cycle count so this must be a zero.  Nothing needs to be changed in the
    // stored data.
  }

  DEBUG_PRINTLN(F("Received from DHT:"));
  DEBUG_PRINT(data[0], HEX);
  DEBUG_PRINT(F(", "));
  DEBUG_PRINT(data[1], HEX);
  DEBUG_PRINT(F(", "));
  DEBUG_PRINT(data[2], HEX);
  DEBUG_PRINT(F(", "));
  DEBUG_PRINT(data[3], HEX);
  DEBUG_PRINT(F(", "));
  DEBUG_PRINT(data[4], HEX);
  DEBUG_PRINT(F(" =? "));
  DEBUG_PRINTLN((data[0] + data[1] + data[2] + data[3]) & 0xFF, HEX);

  // Check we read 40 bits and that the checksum matches.
  if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
    _lastresult = true;
    return _lastresult;
  } else {
    DEBUG_PRINTLN(F("DHT checksum failure!"));
    _lastresult = false;
    return _lastresult;
  }
}

// Expect the signal line to be at the specified level for a period of time and
// return a count of loop cycles spent at that level (this cycle count can be
// used to compare the relative time of two pulses).  If more than a millisecond
// ellapses without the level changing then the call fails with a 0 response.
// This is adapted from Arduino's pulseInLong function (which is only available
// in the very latest IDE versions):
//   https://github.com/arduino/Arduino/blob/master/hardware/arduino/avr/cores/arduino/wiring_pulse.c
uint32_t DHT::expectPulse(bool level) {
// F_CPU is not be known at compile time on platforms such as STM32F103.
// The preprocessor seems to evaluate it to zero in that case.
#if (F_CPU > 16000000L) || (F_CPU == 0L)
  uint32_t count = 0;
#else
  uint16_t count = 0; // To work fast enough on slower AVR boards
#endif
// On AVR platforms use direct GPIO port access as it's much faster and better
// for catching pulses that are 10's of microseconds in length:
#ifdef __AVR
  uint8_t portState = level ? _bit : 0;
  while ((*portInputRegister(_port) & _bit) == portState) {
    if (count++ >= _maxcycles) {
      return TIMEOUT; // Exceeded timeout, fail.
    }
  }
// Otherwise fall back to using digitalRead (this seems to be necessary on
// ESP8266 right now, perhaps bugs in direct port access functions?).
#else
  while (digitalRead(_pin) == level) {
    if (count++ >= _maxcycles) {
      return TIMEOUT; // Exceeded timeout, fail.
    }
  }
#endif

  return count;
}
#pragma endregion DHT_cpp

#pragma region functionality
#include <IRremote.h>

#include <Servo.h>
#include <LiquidCrystal.h>

// Códigos hexadecimales de la señal de nuestro mando (cada mando tiene unos códigos diferentes)
#define Tecla_1 0xFB040707
#define Tecla_2 0xFA050707
#define Tecla_3 0xF9060707
#define Tecla_4 0xF7080707
#define Tecla_5 0xF6090707
#define Tecla_6 0xF50A0707
#define Tecla_7 0xF30C0707


#define bombPin 13    // Pin digital para la bomba de agua
#define tempPin 12    // Pin digital para el sensor de temperatura
#define LED_PIN 11    // Pin digital para el LED
#define fanPin 10     // Pin digital para el ventilador
#define servoPin 9  // Pin digital para el servomotor


#define LIGHT_SENSOR A1  // Pin al que está conectado el sensor de luz
#define HUMIDITYPIN A3   // Pin al que está conectado el sensor de humedad
#define WATERLEVEL A0
const int IR = 8;        // Asignar el ir al pin 8


int temperatura;// Variable para almacenar la temperatura
int humedad;    // Variable para almacenar la humedad
int luz;        // Variable para almacenar la lectura del sensor de luz  
int water;      // Variable para almacenar el nivel de agua


#define DHTTYPE DHT11     // Definimos el tipo de DHT
DHT dht(tempPin, DHTTYPE);


LiquidCrystal lcd(2, 3, 4, 5, 6, 7);
//Servo servo_9;


void setup() {
    Serial.begin(9600);
    // Si ponemos esto al pulsar cualquier boton se vuelve loco, probablemente haya algun tipo de conflicto entre las librerias
  //  servo_9.attach(servoPin);   
    IrReceiver.begin(IR, DISABLE_LED_FEEDBACK);  
    pinMode(LED_PIN, OUTPUT);  
    pinMode(fanPin, OUTPUT);      // Configurar el pin del ventilador como salida
    pinMode(bombPin, OUTPUT);
    dht.begin();                  // Iniciamos el dht
    lcd.begin(16,2);
}


void loop() {
  // Al iniciar el programa, está el modo manual activado por defecto
  if (IrReceiver.decode()) {
    Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);
    // Imprime el código hexadecimal que ha detectado el infrarrojo


    if (IrReceiver.decodedIRData.decodedRawData == Tecla_1)
        digitalWrite(LED_PIN, HIGH);// Al pulsar 1, encendemos LEDS
    if (IrReceiver.decodedIRData.decodedRawData == Tecla_2)
        digitalWrite(LED_PIN, LOW);// Al pulsar el botón 2, apagamos LEDS
    if (IrReceiver.decodedIRData.decodedRawData == Tecla_3) {
        // Al pulsar el botón 3, se muestran los valores de los sensores en el Serial Monitor, por si no funcionara la pantalla LCD
        // --- Lectura de la humedad ---
        humedad = analogRead(HUMIDITYPIN);// Leemos la humedad del sensor
        float humedad_porcentaje = map(humedad, 0, 1023, 0, 100);
        // Convertimos el valor a un valor porcentual (0-100)
        Serial.print("Humedad: ");
        Serial.print(humedad_porcentaje);
        Serial.println("%");
        delay(1500);

        // --- Lectura de la luminosidad ---
        luz = analogRead(LIGHT_SENSOR);
        Serial.print("Luz: ");
        Serial.println(luz);
        delay(1500);
        // --- Lectura de la temperatura ---
        temperatura = dht.readTemperature(); // Lectura de la temperatura
        if (isnan(temperatura)) {
            // Si el valor de la temperatura es NaN, entonces lo indicamos
            Serial.println("Error obteniendo los datos del sensor DHT11");
        } 
        else {
            Serial.print("Temperatura: ");
            Serial.print(temperatura);
            Serial.println(" grados");
        }
        delay(1500);  
        // --- Lectura del nivel del agua ---
        water = analogRead(WATERLEVEL);   // Leemos el nivel del agua
        Serial.print("Agua: ");
        Serial.println(water);
        delay(1500);
    }
   
    if (IrReceiver.decodedIRData.decodedRawData == Tecla_4) {
      // Al pulsar el botón 4, se enciende el ventilador durante 2 segundos
      // Activar el ventilador durante 2 segundos
      digitalWrite(fanPin, HIGH);
      delay(2000);
      // Desactivar el ventilador durante 2 segundos
      digitalWrite(fanPin, LOW);
      delay(2000);
    }
   
    if (IrReceiver.decodedIRData.decodedRawData == Tecla_5) {
      // Al pulsar el botón 5, se enciende la bomba de agua durante 2 segundos
      water = analogRead(WATERLEVEL);
      Serial.print("Agua: ");
      Serial.println(water);
      if(water > 20) {
        // Si el nivel del agua es mayor a 20, entonces se puede regar
        digitalWrite(bombPin, HIGH);
        delay(2000);
        // Desactivar la bomba de agua después de 2 segundos
        digitalWrite(bombPin, LOW);
        delay(1000);
      } 
      else 
        Serial.println("No hay agua suficiente");

      delay(200);
    }
 
    if (IrReceiver.decodedIRData.decodedRawData == Tecla_6) {
      // Al pulsar el botón 6, se enciende la LCD y muestra los valores de los sensores
      lcd.clear();
      temperatura = dht.readTemperature();
      humedad = analogRead(HUMIDITYPIN);  // Leer la humedad del sensor
      float humedad_porcentaje = map(humedad, 0, 1023, 0, 100);
      // Convertir valor a porcentaje (0-100)
      lcd.setCursor(0,0);
      lcd.print("Temperatura: ");
      if (isnan(temperatura)) {
        // Si el valor de la temperatura es NaN, entonces lo indicamos
        Serial.println("?");
      } else lcd.print(temperatura);
           lcd.setCursor(0,1);
      lcd.print("Humedad:   ");
      lcd.print(humedad_porcentaje);
      delay(5000); // Los mensajes se muestran durante 5 segundos
     
      lcd.clear(); // Luego borramos la LCD y escribimos más valores
      delay(500);


      lcd.setCursor(0,0);
      luz = analogRead(LIGHT_SENSOR);
      lcd.print("     Luz:    ");
      lcd.println(luz);


      lcd.setCursor(0,1);
      water = analogRead(WATERLEVEL);
      lcd.print("     Agua:    ");
      lcd.println(water);
     
      delay(5000);  // Los mensajes se muestran durante 5 segundos
      lcd.clear();
    }


    if (IrReceiver.decodedIRData.decodedRawData == Tecla_7) {
      // Al pulsar el botón 7, se activa el funcionamiento automático hasta que se vuelva a pulsar otro botón
      humedad = analogRead(HUMIDITYPIN);  // Leer la humedad del sensor
      float humedad_porcentaje = map(humedad, 0, 1023, 0, 100);
      // Convertir valor a porcentaje (0-100)
      water = analogRead(WATERLEVEL);
      temperatura = dht.readTemperature(); // Lectura de la temperatura
      luz = analogRead(LIGHT_SENSOR);
     
      if(luz < 120 || temperatura < 10){
        if(luz < 120) Serial.println("Luz insuficiente, activando LEDS");
        if(temperatura < 10) Serial.println("Temperatura baja, activando LEDS");
        digitalWrite(LED_PIN, HIGH);
      } else digitalWrite(LED_PIN, LOW);


      if(humedad_porcentaje < 75){
        Serial.print("Humedad baja, preparando la bomba de agua.");
        Serial.print("Nivel de agua: ");
        Serial.println(water);
        if(water > 20) {
          Serial.println("Agua suficiente. Activando la bomba");
          digitalWrite(bombPin, HIGH);
          delay(2000);
          // Se riega durante 2 segundos
          digitalWrite(bombPin, LOW);
          delay(1000);
        } 
        else 
            Serial.println("No hay agua suficiente. No se puede activar la bomba");
        delay(2000);
      }
     
      if(temperatura > 30){
        Serial.println("Temperatura elevada, activando ventilador");
        // Activar el ventilador durante 2 segundos
        digitalWrite(fanPin, HIGH);
        delay(10000);
        // Desactivar el ventilador durante 2 segundos
        digitalWrite(fanPin, LOW);
        delay(2000);
      }
    }
    IrReceiver.resume();
  }
}
#pragma endregion functionality