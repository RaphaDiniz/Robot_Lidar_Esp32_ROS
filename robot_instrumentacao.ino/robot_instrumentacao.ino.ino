#include <SoftwareSerial.h>

// Classe para controlar os motores
class MotorController {
public:
  MotorController(int pin1, int pin2, int pin3, int pin4) : motorPins{pin1, pin2, pin3, pin4} {
    // Configura os pinos dos motores como saídas
    for (int i = 0; i < 4; ++i) {
      pinMode(motorPins[i], OUTPUT);
    }
  }

  // Função para mover os motores para frente
  void moveForward() {
    Serial.println("frente");
    digitalWrite(motorPins[0], HIGH);
    digitalWrite(motorPins[1], LOW);
    digitalWrite(motorPins[2], HIGH);
    digitalWrite(motorPins[3], LOW);
  }

  // Função para mover os motores para trás
  void moveBackward() {
    digitalWrite(motorPins[0], LOW);
    digitalWrite(motorPins[1], HIGH);
    digitalWrite(motorPins[2], LOW);
    digitalWrite(motorPins[3], HIGH);
  }

  // Função para girar os motores para a esquerda
  void turnLeft() {
    digitalWrite(motorPins[0], LOW);
    digitalWrite(motorPins[1], HIGH);
    digitalWrite(motorPins[2], HIGH);
    digitalWrite(motorPins[3], LOW);
  }

  // Função para girar os motores para a direita
  void turnRight() {
    digitalWrite(motorPins[0], HIGH);
    digitalWrite(motorPins[1], LOW);
    digitalWrite(motorPins[2], LOW);
    digitalWrite(motorPins[3], HIGH);
  }

  // Função para parar os motores
  void stop() {
    for (int i = 0; i < 4; ++i) {
      digitalWrite(motorPins[i], LOW);
    }
  }

private:
  int motorPins[4]; // Pinos dos motores
};

// Classe para controlar a comunicação Bluetooth
class BluetoothController {
public:
  BluetoothController(int rxPin, int txPin) : bluetooth(rxPin, txPin) {
    bluetooth.begin(9600); // Inicia a comunicação serial com o módulo Bluetooth
  }

  // Função para ler os dados recebidos via Bluetooth
  char readBluetooth() {
    if (bluetooth.available()) {
      char data = bluetooth.read();
      Serial.print("Dado recebido via Bluetooth: ");
      Serial.println(data);
      return data;
    }
    return '\0'; // Retorna caractere nulo se não houver dados disponíveis
  } 

  // Função para escrever dados via Bluetooth
  void writeSerial(char data) {
    bluetooth.write(data);
  }

private:
  SoftwareSerial bluetooth; // Objeto SoftwareSerial para comunicação Bluetooth
};

// Pinos dos motores
const int motorPino1 = 5;
const int motorPino2 = 6;
const int motorPino3 = 9;
const int motorPino4 = 10;

// Pinos RX e TX para o módulo Bluetooth
const int bluetoothRx = 2;
const int bluetoothTx = 3;

MotorController motor(motorPino1, motorPino2, motorPino3, motorPino4);
BluetoothController bluetooth(bluetoothRx, bluetoothTx);

void setup() {
  // Configura os motores e o Bluetooth
  Serial.begin(9600);
}

void loop() {
  // Lê os dados recebidos via Bluetooth
  char comando = bluetooth.readBluetooth();
  Serial.println(comando);
  // Executa a ação correspondente ao comando recebido
  switch (comando) {
    case 'F':
      motor.moveForward();
      break;
    case 'B':
      motor.moveBackward();
      break;
    case 'L':
      motor.turnLeft();
      break;
    case 'R':
      motor.turnRight();
      break;
    case 'S':
      motor.stop();
      break;
    default:
      break;
  }
}
