#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <semphr.h>
#include <DHT.h>

#define DHTPIN 2
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

// Pines del motor
int speedPin = 11;
int dirPin1 = 12;
int dirPin2 = 13;
int potPin = A0;       // Pin del potenciómetro
int lightPin = A1;     // Pin de la fotorresistencia

// Variables para las colas
QueueHandle_t tempQueue;
QueueHandle_t humidityQueue;
QueueHandle_t lightQueue;
QueueHandle_t motorSpeedQueue;

// Semáforos
SemaphoreHandle_t xMutex;

// Funciones de las tareas
void TaskReadTemperature(void *pvParameters);
void TaskReadHumidity(void *pvParameters);
void TaskReadLight(void *pvParameters);
void TaskControlMotor(void *pvParameters);

void setup() {
  Serial.begin(9600);
  pinMode(speedPin, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  dht.begin();

  // Crear las colas
  tempQueue = xQueueCreate(10, sizeof(float));
  humidityQueue = xQueueCreate(10, sizeof(float));
  lightQueue = xQueueCreate(10, sizeof(int));
  motorSpeedQueue = xQueueCreate(10, sizeof(int));

  // Crear el mutex
  xMutex = xSemaphoreCreateMutex();

  // Verificar si las colas y el mutex se crearon correctamente
  if (tempQueue != NULL && humidityQueue != NULL && lightQueue != NULL && motorSpeedQueue != NULL && xMutex != NULL) {
    // Crear las tareas
    xTaskCreate(TaskReadTemperature, "ReadTemp", 128, NULL, 1, NULL);
    xTaskCreate(TaskReadHumidity, "ReadHum", 128, NULL, 1, NULL);
    xTaskCreate(TaskReadLight, "ReadLight", 128, NULL, 1, NULL);
    xTaskCreate(TaskControlMotor, "ControlMotor", 256, NULL, 2, NULL);
  } else {
    Serial.println("Error al crear las colas o el mutex");
  }
}

void loop() {
  
}

void TaskReadTemperature(void *pvParameters) {
  (void) pvParameters;
  
  for (;;) {
    float temperature = dht.readTemperature();

    if (!isnan(temperature)) {
      xQueueSend(tempQueue, &temperature, portMAX_DELAY);
    } else {
      Serial.println("Error al leer la temperatura del sensor DHT!");
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Leer cada 1 segundo
  }
}

void TaskReadHumidity(void *pvParameters) {
  (void) pvParameters;
  
  for (;;) {
    float humidity = dht.readHumidity();

    if (!isnan(humidity)) {
      xQueueSend(humidityQueue, &humidity, portMAX_DELAY);
    } else {
      Serial.println("Error al leer la humedad del sensor DHT!");
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Leer cada 1 segundo
  }
}

void TaskReadLight(void *pvParameters) {
  (void) pvParameters;
  
  for (;;) {
    int lightValue = analogRead(lightPin);
    xQueueSend(lightQueue, &lightValue, portMAX_DELAY);

    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Leer cada 1 segundo
  }
}

void TaskControlMotor(void *pvParameters) {
  (void) pvParameters;

  float temperature = 0;
  float humidity = 0;
  int lightValue = 0;
  int potValue = 0;
  int motorSpeed = 0;

  for (;;) {
    // Leer el valor del potenciómetro
    potValue = analogRead(potPin);
    motorSpeed = map(potValue, 0, 1023, 0, 255); // Convertir el valor del potenciómetro a velocidad del motor

    // Leer las colas de sensores
    if (xQueueReceive(tempQueue, &temperature, 0) == pdPASS) {
      Serial.print("Temperatura: ");
      Serial.print(temperature);
      Serial.println(" *C");
    }

    if (xQueueReceive(humidityQueue, &humidity, 0) == pdPASS) {
      Serial.print("Humedad: ");
      Serial.print(humidity);
      Serial.println(" %");
    }

    if (xQueueReceive(lightQueue, &lightValue, 0) == pdPASS) {
      Serial.print("Luminosidad: ");
      Serial.println(lightValue);
    }

    // Ajustar la velocidad en función de la temperatura, luminosidad y humedad
    if (temperature > 29) {
      motorSpeed -= 50; // Reducir velocidad si la temperatura es mayor a 29 grados
    }

    if (lightValue < 300) {
      motorSpeed -= 50; // Reducir velocidad si la luminosidad es muy baja
    }

    if (humidity > 80) {
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

    vTaskDelay(200 / portTICK_PERIOD_MS);  // Ajustar este delay según sea necesario
  }
}
