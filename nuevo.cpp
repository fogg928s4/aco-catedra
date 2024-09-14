#include <IRremote.h>
#include <ServoTimer2.h>
#include <Servo.h>
#include <DHT.h>
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
    servo_9.attach(servoPin);   
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
      } else {
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
      } else (Serial.println("No hay agua suficiente"));
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
        } else (Serial.println("No hay agua suficiente. No se puede activar la bomba"));
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