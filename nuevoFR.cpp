#pragma region variables
#include <IRremote.h>
#include <Servo.h>
#include <LiquidCrystal.h>

// Códigos hexadecimales de la señal de nuestro mando (cada mando tiene unos códigos diferentes)
#define Tecla_1 0xEF10BF00
#define Tecla_2 0xEE11BF00
#define Tecla_3 0xED12BF00
#define Tecla_4 0xEB14BF00
#define Tecla_5 0xEA15BF00
#define Tecla_6 0xE916BF00
#define Tecla_7 0xE718BF00


#define bombPin 13    // Pin digital para la bomba de agua
#define LED_PIN 11    // Pin digital para el LED
#define fanPin 10     // Pin digital para el ventilador
#define servoPin 9  // Pin digital para el servomotor (ventana)

//para la temperatura
#define tempPin A4    // Pin digital para el sensor de temperatura

#define LIGHT_SENSOR A1  // Pin al que está conectado el sensor de luz
#define HUMIDITYPIN A3   // Pin al que está conectado el sensor de humedad
#define WATERLEVEL A0
const int IR = 8;        // Asignar el ir al pin 8


float temperatura;// Variable para almacenar la temperatura
int humedad;    // Variable para almacenar la humedad
int luz;        // Variable para almacenar la lectura del sensor de luz  
int water;      // Variable para almacenar el nivel de agua
float voltaje; //para la temperatura
Servo servoEng; //Objeto para el servo motor

LiquidCrystal lcd(2, 3, 4, 5, 6, 7);
//Servo servo_9;

bool automatico = false;
bool isLED = false;
bool isFan = false;
bool isBomb = false;
bool isWindow = true;

#pragma endregion variables

#pragma region metodos
void showSerial() {
    
    // --- Lectura de la humedad ---
    humedad = analogRead(HUMIDITYPIN);// Leemos la humedad del sensor
    float humedad_porcentaje = map(humedad, 0, 1023, 0, 100);
    // Convertimos el valor a un valor porcentual (0-100)
    Serial.print("Humedad: ");
    Serial.print(humedad_porcentaje);
    Serial.println("%");
    //delay(1500);
// --- Lectura de la luminosidad ---
    luz = analogRead(LIGHT_SENSOR);
    Serial.print("Luz: ");
    Serial.println(luz);
    //delay(1500);
// --- Lectura de la temperatura ---
    Serial.println(analogRead(tempPin));
    voltaje =  analogRead(tempPin) * (5.0 / 1024.0); //algo con los 5 V
    Serial.println(voltaje);
    temperatura =  (voltaje - 0.5) * 100; // Lectura de la temperatura
    
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

//Muesta datos en LCD
void showLCD() {
    lcd.clear();
    voltaje = analogRead(tempPin) * (5.0 / 1024.0); //algo con los 5 V
    temperatura =  (voltaje - 0.5) * 100.0; // Lectura de la temperatura
    humedad = analogRead(HUMIDITYPIN);  // Leer la humedad del sensor
    float humedad_porcentaje = map(humedad, 0, 1023, 0, 100);
    // Convertir valor a porcentaje (0-100)
    lcd.setCursor(0,0);
    lcd.print("Temperatura: ");
    if (isnan(temperatura)) {
        // Si el valor de la temperatura es NaN, entonces lo indicamos
        Serial.println("?");
    } 
    else 
        lcd.print(temperatura);
    
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

//Enciende o apaga el LED
void turnLED() {
    //TRUE = encendida
    //si esta encendida la apaga
    digitalWrite(LED_PIN, (isLED ? LOW : HIGH));
    isLED = !isLED;
    //msj
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("LUZ ROJA: ");
    lcd.setCursor(0,1);
    Serial.print("Luz de calefaccion: ");
    if(isLED) {
        lcd.print("ON");
        Serial.println("Encendida");
    }
    else {
        lcd.print("OFF");
        Serial.println("Apagada");
    }
    
}

//Enciende o apaga el ventilador
void turnFAN() {
    //TRUE = encendida si esta encendida la apaga
    digitalWrite(fanPin, (isFan ? LOW : HIGH));
    isFan = !isFan;
    //imprime un msj de confirmacion
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("VENTILADOR: ");
    lcd.setCursor(0,1);
    Serial.print("Ventilador: ");
    if(isFan){
        lcd.print("ON");
        Serial.println("Encendido");
    }
    else {
        lcd.print("OFF");
        Serial.println("Apagado");
    }
        
}

//Enciende o apaga la bomba
void turnBOMB() {
    digitalWrite(bombPin, (isBomb ? LOW : HIGH));
    isBomb = !isBomb;
    //imprime un msj de confirmacion
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("BOMBA H2O: ");
    lcd.setCursor(0,1);
    Serial.print("Bomba de agua: ");
    if(isBomb){
        lcd.print("ON");
        Serial.println("Encendida");
    }
    else {
        lcd.print("OFF");
        Serial.println("Apagada");
    }
}

//Abre o cierra la ventana
void turnWindow() {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("VENTANA: ");
    lcd.setCursor(0,1);
    Serial.print("Ventana: ");
    if (isWindow) {
        servoEng.write(90);
        isWindow = !isWindow;
        lcd.print("ABIERTA");
        Serial.println("ABIERTA");
    }
    else {
        servoEng.write(-1);
        isWindow = !isWindow;
        lcd.print("CERRADA");
        Serial.println("CERRADA");
    }
    
}

//Lectura de
void hacer() {
    // Leer la humedad del sensor
    humedad = analogRead(HUMIDITYPIN);  
    float humedad_porcentaje = map(humedad, 0, 1023, 0, 100);
    // Convertir valor a porcentaje (0-100)
    water = analogRead(WATERLEVEL);
    
    //Lee la temp del sensor
    voltaje = analogRead(tempPin) * (5.0 / 1024.0); //algo con los 5 V
    temperatura =  (voltaje - 0.5) * 100; 
    
    //Lectura de la luz
    luz = analogRead(LIGHT_SENSOR);

    if(luz < 120 || temperatura < 10){
        if(luz < 120) {
            isLED = false;
            Serial.println("Luz insuficiente, activando LEDS");
            turnLED();
            delay(5000);
        }
            
        if(temperatura < 10) {
            isLED = false;
            turnLED();
            Serial.println("Temperatura baja, activando LEDS");
            delay(5000);
        }
    } 
    else {
        isLED = true;
        turnLED();
        delay(5000);
    } 
        

    if(humedad_porcentaje < 75)  {
        Serial.print("Humedad baja, preparando la bomba de agua.");
        Serial.print("Nivel de agua: ");
        Serial.println(water);
        // Si el nivel del agua es mayor a 20, entonces se puede regar
        isBomb = false;
        turnBOMB();
        delay(5000);
        turnBOMB();
        delay(100);
    }

    if(temperatura > 30){
        Serial.println("Temperatura elevada, activando ventilador");
        // Activar el ventilador durante 2 segundos
        isFan = false;
        turnFAN();
        delay(5000);
        turnFAN();
        delay(100);
	}
  	else    
        Serial.println("Temperatura adecuada");
}

//Modo automatico del sistema
void autoMODE() {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("MODO AUTO: ");
    lcd.setCursor(0,1);
    Serial.print("Modo automatico: ");
    if (automatico == true) {
        lcd.print("ON");
        Serial.println("ENCENDIDO");
        hacer();
        delay(1500);
        IrReceiver.resume();
        autoMODE();
    }
    else {
        lcd.print("OFF");
        Serial.println("APAGADO");
    }
}

void errorKey() {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("TECLA NO");
    lcd.setCursor(0,1);
    lcd.print("RECONOCIDA");
    Serial.println("Tecla no reconocida, presiones otra");
}


#pragma endregion metodos


#pragma region functionality

void setup() {
    Serial.begin(9600);
    // Si ponemos esto al pulsar cualquier boton se vuelve loco, probablemente haya algun tipo de conflicto entre las librerias
     //  servo_9.attach(servoPin);   
    IrReceiver.begin(IR, DISABLE_LED_FEEDBACK);  
    pinMode(LED_PIN, OUTPUT);  
    pinMode(fanPin, OUTPUT);      // Configurar el pin del ventilador como salida
    pinMode(bombPin, OUTPUT);
    pinMode(tempPin, INPUT);
    lcd.begin(16,2);
  	servoEng.attach(9);//se asigna el pin al servo motor
  	servoEng.write(-1);
}



void loop() {
    // Al iniciar el programa, está el modo manual activado por defecto
    if (IrReceiver.decode()) {
        // Imprime el código hexadecimal que ha detectado el infrarrojo
        //Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);
        
        switch(IrReceiver.decodedIRData.decodedRawData) {
            case Tecla_1: //tecla 1 enciende o apaga el LED
                turnLED();
                break;
            case Tecla_2: //tecla 2 on/off ventilador
                turnFAN();
                break;
            case Tecla_3: //tecla 3 on/off bomba
                turnBOMB();
                break;
            case Tecla_4: //tecla 4 abre cierra ventana
                turnWindow();
                break;
            case Tecla_5: //tecla 6 muestra en serial
                showSerial();
                break;
            case Tecla_6: //tecla 5 enciene la pantalla
                showLCD();
                break;
            case Tecla_7: // modo automatico on/off
                automatico = !automatico;
                autoMODE();
                break;
            default:
                errorKey();
                break;
            
            
        }
        //al presionar el boton 7 inicia el modo auto
      /** if (IrReceiver.decodedIRData.decodedRawData == Tecla_7) {
            IrReceiver.resume();
            
      	    while(automatico) {
                hacer();
                delay(1000);
                if (IrReceiver.decode()) {
                    if (IrReceiver.decodedIRData.decodedRawData == Tecla_7) {
                        automatico = false;
                        Serial.println("entra aqui");
                    }
                }
                IrReceiver.resume();
      	    }
        } 	*/
        IrReceiver.resume();
    }
}

#pragma endregion functionality