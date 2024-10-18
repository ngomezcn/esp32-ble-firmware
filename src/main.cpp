#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <WiFi.h> // Asegúrate de incluir esta biblioteca

// Define el nombre del dispositivo BLE
#define DEVICE_NAME "GUAT"

// Define el UUID del servicio y la característica
#define SERVICE_UUID "4c491e6a-38df-4d0f-b04b-8704a40071ce"
#define CHARACTERISTIC_UUID "b0726341-e52e-471b-9cd6-4061e54616cc"

const int statusLedPin = 2;
const int gateRelayPin = 5;

String commandBuffer = "";

// Variables para el parpadeo del LED de estado
unsigned long previousMillis = 0;

const long offInterval = 4000; // 4 segundos (parpadeo normal)
const long onInterval = 1000;  // 1 segundo (parpadeo normal)

const long rapidOffInterval = 200; // 0.2 segundos (parpadeo rápido)
const long rapidOnInterval = 200;  // 0.2 segundos (parpadeo rápido)

enum LedState
{
  OFF,
  ON
};
LedState currentLedState = OFF; // Estado inicial del LED

// Variables globales
BLEServer *pServer = nullptr;
BLECharacteristic *pCharacteristic = nullptr;
BLEAdvertising *pAdvertising = nullptr; // Mover la declaración aquí
bool deviceConnected = false;
bool ledState = false;

// Declarar la función sendLEDState antes de la clase
void sendLEDState();

// Clase para manejar conexiones del cliente
class MyServerCallbacks : public BLEServerCallbacks
{
public:
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
    previousMillis = 0;   // Reiniciar el tiempo
    sendLEDState();       // Enviar el estado del LED al cliente al conectar

    // Reiniciar la publicidad
    pAdvertising->start(); // Reiniciar la publicidad al conectar
  }

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    pAdvertising->start(); // Reiniciar la publicidad al desconectar
  }
};

// Función para enviar el estado del LED al cliente
void sendLEDState()
{
  if (deviceConnected)
  {
    String state = ledState ? "ENCENDIDO" : "APAGADO";
    pCharacteristic->setValue(state.c_str()); // Configura el valor de la característica
    pCharacteristic->notify();                // Envía la notificación al cliente
    Serial.print("Estado del LED enviado: ");
    Serial.println(state);
  }
}

void setup()
{
  // Inicializar el puerto serie
  Serial.begin(115200);

  // Inicializar el pin del LED
  pinMode(statusLedPin, OUTPUT);
  pinMode(gateRelayPin, OUTPUT);

  /* TEMPORALMENT DESACTIVADO 
  pinMode(32, OUTPUT);
  pinMode(26, OUTPUT);
  digitalWrite(32, HIGH);
  digitalWrite(26, HIGH);
  */

  digitalWrite(statusLedPin, LOW);
  digitalWrite(gateRelayPin, LOW);

  // Inicializar BLE
  BLEDevice::init(DEVICE_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Crear un servicio BLE
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Crear una característica BLE
  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY); // Agregar propiedad de notificación

  // Iniciar el servicio
  pService->start();

  // Inicializar la publicidad BLE
  pAdvertising = BLEDevice::getAdvertising(); // Inicializa el objeto pAdvertising
  pAdvertising->addServiceUUID(SERVICE_UUID); // Añadir el UUID del servicio
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinInterval(100);
  pAdvertising->setMaxInterval(200);
  pAdvertising->setAdvertisementType(ADV_TYPE_IND);

  // Iniciar el advertising
  pAdvertising->start();

  // Mostrar la dirección MAC en el puerto serie
  Serial.print("\n\nDirección MAC: ");
  Serial.println(WiFi.macAddress()); // Mostrar la dirección MAC

  Serial.println("Advertising started...");
}

void blinkStatusLedPeriodically()
{
  unsigned long currentMillis = millis();

  // Cambiar el estado del LED dependiendo de la conexión
  if (deviceConnected)
  {
    // Parpadeo rápido
    if (currentLedState == OFF && currentMillis - previousMillis >= rapidOffInterval)
    {
      previousMillis = currentMillis;
      currentLedState = ON;
      digitalWrite(statusLedPin, HIGH); // Encender LED
    }
    else if (currentLedState == ON && currentMillis - previousMillis >= rapidOnInterval)
    {
      previousMillis = currentMillis;
      currentLedState = OFF;
      digitalWrite(statusLedPin, LOW); // Apagar LED
    }
  }
  else
  {
    // Si no está conectado, asegúrate de que el LED esté apagado
    // Parpadeo normal
    if (currentLedState == OFF && currentMillis - previousMillis >= offInterval)
    {
      previousMillis = currentMillis;
      currentLedState = ON;
      digitalWrite(statusLedPin, HIGH); // Encender LED
    }
    else if (currentLedState == ON && currentMillis - previousMillis >= onInterval)
    {
      previousMillis = currentMillis;
      currentLedState = OFF;
      digitalWrite(statusLedPin, LOW); // Apagar LED
    }
  }
}

void loop()
{
  if (deviceConnected)
  {
    if (pCharacteristic->getValue().length() > 0)
    {
      String value = pCharacteristic->getValue().c_str();
      Serial.print("Valor recibido: ");
      Serial.println(value);

      // Comprobar el código recibido
      if (value == "ENCENDER" && !ledState)
      {                                   // Cambia solo si el LED está apagado
        digitalWrite(gateRelayPin, HIGH); // Encender LED
        ledState = true;
        Serial.println("LED encendido.");
        sendLEDState(); // Enviar el estado del LED después de encenderlo
      }
      else if (value == "APAGAR" && ledState)
      {                                  // Cambia solo si el LED está encendido
        digitalWrite(gateRelayPin, LOW); // Apagar LED
        ledState = false;
        Serial.println("LED apagado.");
        sendLEDState(); // Enviar el estado del LED después de apagarlo
      }

      // Limpiar el valor después de procesarlo
      pCharacteristic->setValue(""); // Reiniciar valor
    }
  }

  // Llamar a la función de parpadeo del LED
  blinkStatusLedPeriodically();

  // Evitar que el loop consuma mucha CPU
  delay(10); // Un pequeño delay para evitar un uso excesivo de la CPU
}
