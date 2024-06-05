#include <DHT.h>

#define DHTPIN 2
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

int speedPin = 11;
int dirPin1 = 12;
int dirPin2 = 13;
int potPin = A0;       // Pin del potenciómetro
int lightPin = A1;     // Pin de la fotorresistencia

float temperature = 0;
float humidity = 0;
int lightValue = 0;
int motorSpeed = 0;

void setup() {
  Serial.begin(9600);
  pinMode(speedPin, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  dht.begin();
}

void loop() {
  ReadHum();
  ReadLight();
  ReadTemp();
  ControlMotor();
  delay(1000); // Espera 1 segundo antes de la siguiente lectura
}

void ReadHum() {
  humidity = dht.readHumidity();

  if (isnan(humidity)) {
    Serial.println("Error al leer la humedad del sensor DHT!");
    return;
  }

  Serial.print("Humedad: ");
  Serial.print(humidity);
  Serial.println(" %");
}

void ReadLight() {
  lightValue = analogRead(lightPin);
  Serial.print("Luminosidad: ");
  Serial.println(lightValue);
}

void ReadTemp() {  
  temperature = dht.readTemperature();

  if (isnan(temperature)) {
    Serial.println("Error al leer la temperatura del sensor DHT!");
    return;
  }

  Serial.print("Temperatura: ");
  Serial.print(temperature);
  Serial.println(" *C");
}

void ControlMotor() {
  int potValue = analogRead(potPin); // Leer el valor del potenciómetro
  motorSpeed = map(potValue, 0, 1023, 0, 255); // Convertir el valor del potenciómetro a velocidad del motor

  // Ajustar la velocidad en función de la temperatura, luminosidad y humedad
  if (temperature > 29) {
    motorSpeed -= 50; // Reducir velocidad si la temperatura es mayor a 29 grados
  }
  
  if (lightValue < 300) {
    motorSpeed -= 50; // Reducir velocidad si la luminosidad es muy baja
  }

  if (humidity > 80) { // Ajusta este umbral según las características de tu sensor de humedad
    motorSpeed -= 50; // Reducir velocidad si la humedad es muy alta
  }

  // Asegurarse de que la velocidad no sea menor que 0
  if (motorSpeed < 0) {
    motorSpeed = 0;
  }

  // Controlar la dirección del motor
  digitalWrite(dirPin1, LOW);
  digitalWrite(dirPin2, HIGH);

  // Ajustar la velocidad del motor
  analogWrite(speedPin, motorSpeed);

  // Imprimir la velocidad del motor en el monitor serial
  Serial.print("Velocidad del motor: ");
  Serial.println(motorSpeed);
}



