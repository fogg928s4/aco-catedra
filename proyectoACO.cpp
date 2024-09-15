// Librerias
#include <LiquidCrystal.h>
#include <Servo.h>
#include <IRremote.h>
//#include <DHT.h>


//Realmente todo esto podria ser un #define pero es lo que hay
//Variables con los pines Analogicos
const int tempPin = A0;     //Pin para el sensor de temperatura
const int LDRPin = A1;      //Pin para el sensor de luz
const int potenPin = A2;    //Pin para el potenciometro
const int humPin = A3;      //Pin para el sensor de humedad

//Var para los pines digitales
const int fanPin = 10;      //pin para el ventilador
const int ledPin = 11;      //pin para el led
const int IRPin = 5;        //pin pa el sensor IR
const int servoPin = 9;     //pin para el servomotor

//LED para errores (los que estan al fono)
const int ledErrHPin = 8;   //error de humedad
const int ledErrTPin = 7;   //error de temperatura
const int ledErrAPin = 6;   //boton misterioso

//booleanos de control
bool winOpen = 1;
bool aguaSufi = 1; //suficiente agua
bool manualMode = 0;

//variables de datos
int luz, potencia, humedad;
int boton = 0;
float temperatura;

//objetos de librerias
//#define DHTTYPE DHT11

//EN EL 74hc595 
//data 2, clock 4, latch 3
Servo servo9;

//conexion con la pantall via SPI
//DHT dht(tmepPin, DHTTYPE); //para el servo (crash)
LiquidCrystal lcd(2,4,3,2,4,3);


/**********************  SETUP   **********************/
void setup() {
    servo9.attach(servoPin);    //un pin 9 para el servo
    servo9.write(0); //angulo 0 en el servo
    //inicializa pines digitales
    pinMode(fanPin, OUTPUT);
    pinMode(ledPin, OUTPUT);

    lcd.begin(16,2);
    IrReceiver.begin(IRPin);
    Serial.begin(9600);

}

/**********************  LEDS   **********************/
//nocturnas
void encenderLED() {
    digitalWrite(ledPin, HIGH);
    Serial.println("Luces noturnas / de calefaccion ON");
}
void apagarLED() {
    digitalWrite(ledPin, LOW);
    Serial.println("Luces noturnas / de calefaccion OFF");

}

//de error
void checkError() {
    //@TODO: arreglar esto para el chile
    if (humedad > 75 ){
        digitalWrite(ledErrHPin, HIGH);
        Serial.println("Error de humedad. Humedad excesiva");
    }
    else {
        digitalWrite(ledErrHPin, LOW);
        Serial.println("Humedad OK");
    }

    // checa la temperatura ok
    if (temperatura > 30 ){
        digitalWrite(ledErrTPin, HIGH);
        Serial.println("Error de temperatura. temperatura excesiva");
    }
    else {
        digitalWrite(ledErrTPin, LOW);
        Serial.println("Temperatura OK");
    }
    //a ver si hay suficiente agua
    if(aguaSufi == false) {
        digitalWrite(ledErrAPin, HIGH);
        Serial.println("Error de agua, insuficiente");
    }
    else {
        digitalWrite(ledErrTPin, LOW);
        Serial.println("Suficiente Agua OK");
    }

}

/**********************  Ventiladores   **********************/
void fanON() {
    digitalWrite(fanPin, HIGH);
    Serial.println("Ventilador ON");
}
void fanOFF() {
    digitalWrite(fanPin, LOW);
    Serial.println("Ventilador OFF");
}

/**********************  VENTANA   **********************/
void abrirWin() {
    for (int ang = 0; ang <= 180;ang++) {
        servo9.write(ang);      //rota el servo ang grados
        delay(100);
    }
    winOpen = 1;
    Serial.println("Ventan abierta");
}
void cerrarWin() {
    for (int ang = 179; ang > 0;ang--) {
        servo9.write(ang);      //rota el servo ang grados
        delay(100);
    }
    winOpen = 0;
    Serial.println("Ventan cerrada");
}


/**********************  PANTALLA   **********************/
// @TODO: optimizar esto dios mio esta seccion es horrible
//sobre el invernadero (?)
void message1() {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Invernadero: ");
    lcd.setCursor(0,1);
    lcd.print("Mueva la rueda");
    delay(1000);
}
//temperatura
void message2() {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Temperatura: ");
    lcd.setCursor(0,1);
    lcd.print(temperatura);
    lcd.print(" C");
    delay(1000);
}
//humedad
void message3() {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Humedad: ");
    lcd.setCursor(0,1);
    lcd.print(humedad);
    lcd.print(" %");
    delay(1000);
}
//agua estbale
void message4() {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Agua de regado: ");
    lcd.setCursor(0,1);
    //te amo operador terniario
    lcd.print(aguaSufi ? "Estable" : "No estable");
    delay(1000);
}
void message5() {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Modo manual: ");
    if(manualMode){
        lcd.print("ON");
        lcd.setCursor(0,1);
        lcd.print("Pulse un boton");
    }
    else {
        lcd.print("OFF");
        lcd.setCursor(0,1);
        lcd.print(" :D"); //les juro que estaba en el original
    }
    delay(1000);
}

/**********************  FUNCIONAMIENTO DEL IR   **********************/
int leerIR(unsigned long codigo) { //lee un codigo recibido
    switch(codigo) {
        case 0xEF10BF00: //boton 1
            fanON();
            boton = 1;
            break;
        case 0xEE11BF00: //boton 2
            fanOFF();
            boton = 2;
            break;
        case 0xED12BF00: //boton 3
            apagarLED();
            boton = 3;
            break;
        case 0xEB14BF00: //boton 4
            encenderLED();
            boton = 4;
            break;
        case 0xEA15BF00: //boton 5
            abrirWin();
            boton = 5;
            break;
        case 0xE916BF00: //boton 6
            cerrarWin();
            boton = 6;
            break;
        default: //muestra un mensaje
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print("Sin funcion");
            delay(1500);
            boton = -1;
            break;
    }
    Serial.println(codigo, HEX);
    return boton;
}

/**********************MODOS ********************** */
void manual() {
    Serial.println("MODO MANUAL");
    //Lectura senal IR
    if(IrReceiver.decode()) {
        boton = leerIR(IrReceiver.decodedIRData.decodedRawData);
        Serial.println("Botoon marcado: " + boton);
        Serial.flush();
        delay(1000);
        IrReceiver.resume();
    }
}
void automatic() {
    Serial.println("MODO AUTOMATICO");
    //mucho calor
    if(temperatura > 30) {
        fanON();
        if(winOpen == false) abrirWin(); //si la temperatura es muy alta
    }
    else {
        fanOFF();
        if(winOpen == true) cerrarWin();
    }
    delay(1000);

    //control luzy muito frio
    if(luz<100 || temperatura< 10)
        encenderLED();
    else
        apagarLED();
    delay(1001);

}

/**********************  AHORA SI LO BUENO   **********************/
// :P
void loop(){
    //lectura humedad
    humedad = analogRead(humPin);
    humedad /= 8.7;
    Serial.print("Humedad: ");
    Serial.print(humedad);
    Serial.print(" %");
    delay(1000);

    //lectura iluminacion
    luz = analogRead(LDRPin);
    Serial.println("Luminosidad: " + luz );
    delay(1000);

    //lectura temperatura no se cual es ambas datasheets dicen algo distinto
    temperatura = (analogRead(tempPin) - 20)* 3.04;
    //temperatura = (analogRead(tempPin) - 500) / 100;
    //temperatura = (analogRead(tempPin) / 9.31;
    Serial.print("Temperatura: ");
    Serial.print(temperatura);
    Serial.print(" C");

    //lectura potenciometro
    potencia = analogRead(potenPin);
    Serial.println("Potenciometro: " + potencia);
    delay(1000);

    //mostrar en pantalla los mensajes
    int eleccionMsg = map(potencia, 0, 1023, 0, 4);
    Serial.print("Mensaje mostrado: " + eleccionMsg);
    Serial.flush();
    //@TODO LIberen este switch de su miseria
    switch(eleccionMsg) {
        case 0:
            message1();
            break;
        case 1:
            message2();
            break;
        case 2:
            message3();
            break;
        case 3:
            message4();
            break;
        case 4:
        //@TODO dios que es estooooo alguien saquelo de aquiiii
            if(manualMode == true){
                !manualMode; //if true ofc itll turn it into false
                Serial.print("Cambiado automatico: modoManual = " + manualMode);
            }
            else {
                !manualMode;
                Serial.print("Cambiado a manual: modoManual = " + manualMode);
            }
            message5();
            break;
        default:
            break;
    }
    delay(1000);
    //now the manual mode or automatic
    if(manualMode == true)
        manual();
    else
        automatic();

    //comprobar errores
    checkError();
    delay(1002);
}