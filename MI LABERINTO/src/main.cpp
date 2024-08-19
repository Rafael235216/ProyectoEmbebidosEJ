#include <Arduino.h>
#include <ESP32Servo.h>        // Biblioteca para manejar servos en ESP32
#include <Wire.h>              // Biblioteca para comunicación I2C
#include <LiquidCrystal_I2C.h> // Biblioteca para controlar la pantalla LCD I2C
#include <WebServer.h>

// Configuración WiFi
const char *ssid = "Redmi Note 11";
const char *password = "";
WebServer server(80);


Servo servo1;
Servo servo2;
int joyX = 34; // Pines analógicos en ESP32
int joyY = 35;
int joyButton = 33; // Pin digital para el pulsador del joystick
int GPIO_SDA = 21;  // GPIO que usaremos como I2C
int GPIO_SCL = 22;  // GPIO que usaremos como I2C
int obstaclePin = 32; // Pin digital para el sensor de obstáculo

int redPin = 23;  // Pin para el LED rojo del RGB
int greenPin = 12; // Pin para el LED verde del RGB

int servoVal;

// Configurar la dirección I2C del LCD, el tamaño (16x2), y crear el objeto LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

unsigned long startMillis; // Variable para almacenar el tiempo de inicio
unsigned long currentMillis; // Variable para almacenar el tiempo actual
bool gameWon = false; // Variable para almacenar el estado del juego

void setup() {
  servo1.attach(18);  // Conectar al pin digital 18
  servo2.attach(19);  // Conectar al pin digital 19

  pinMode(obstaclePin, INPUT);  // Configurar el pin del sensor de obstáculo como entrada
  pinMode(joyButton, INPUT_PULLUP); // Configurar el pin del pulsador del joystick como entrada con resistencia de pull-up

  pinMode(redPin, OUTPUT);  // Configurar el pin del LED rojo como salida
  pinMode(greenPin, OUTPUT); // Configurar el pin del LED verde como salida

  // Inicializar la pantalla LCD
  Wire.begin(GPIO_SDA, GPIO_SCL);
  lcd.begin(16, 2);
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Comenzo el juego");

  startMillis = millis(); // Capturar el tiempo de inicio

  // Encender la luz verde para indicar que el juego está en movimiento
  digitalWrite(greenPin, HIGH);
  digitalWrite(redPin, LOW);


  //Conectar a la red WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Conectando a WiFi...");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Conectando WiFi");
    }
    Serial.println("Conectado a WiFi");

    // Imprimir la IP de la ESP32
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Conectado");
    lcd.setCursor(0, 1);
    lcd.print(WiFi.localIP());

    server.enableCORS();

    // server.on("/set", HTTP_POST, handleLevelSelection);
    server.begin();
    Serial.println("Servidor iniciado");

    Serial.println("Sistema listo");
}


void loop() {
  server.handleClient();
  if (!gameWon) {
    // Leer el valor del joystick y mover el servo1
    int joyValX = analogRead(joyX);
    int joyValY = analogRead(joyY);

    // Mapear los valores del joystick para que correspondan al rango de movimiento del servo (0 a 180)
    int servoValX = map(joyValX, 0, 4095, 120, 180);
    int servoValY = map(joyValY, 0, 4095, 120, 180);

    // Escribir los valores mapeados a los servos
    servo1.write(servoValX);
    servo2.write(servoValY);

    // Leer el estado del sensor de obstáculo
    int obstacleDetected = digitalRead(obstaclePin);

    // Verificar si el sensor detecta un obstáculo (ahora la lógica está invertida)
    if (obstacleDetected == HIGH) { // Si el pin está en HIGH, el obstáculo está presente
      gameWon = true;
      currentMillis = millis();
      unsigned long elapsedTime = (currentMillis - startMillis) / 1000; // Convertir a segundos
      unsigned long points = 100000 - (elapsedTime * 350);

      // Mostrar mensaje de "GANASTE!!!" y los puntos en el LCD
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("GANASTE!!!");
      lcd.setCursor(0, 1);
      lcd.print("Puntos: ");
      lcd.print(points);

      // Cambiar la luz del LED RGB a roja
      digitalWrite(greenPin, LOW);
      digitalWrite(redPin, HIGH);
    } else {
      // Calcular el tiempo transcurrido
      currentMillis = millis();
      unsigned long elapsedTime = (currentMillis - startMillis) / 1000; // Convertir a segundos

      // Mostrar el tiempo transcurrido en el LCD
      lcd.setCursor(0, 1); // Mover el cursor a la segunda línea
      lcd.print("Tiempo: ");
      lcd.print(elapsedTime); // Mostrar el tiempo en segundos
      lcd.print(" s   "); // Espacios adicionales para borrar caracteres anteriores
    }
  }

  // Verificar si se presiona el pulsador del joystick
  if (digitalRead(joyButton) == LOW) {
    ESP.restart(); // Reiniciar la ESP32
  }

  delay(50); // Pausa de 50 ms entre lecturas
}
