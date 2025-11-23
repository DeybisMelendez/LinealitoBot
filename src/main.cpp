#include <Arduino.h>
#include <IRremote.h>

#define IR_PIN 7

// Pines del L298N
#define IN1 11
#define IN2 10

#define IN3 9
#define IN4 8

// Pines de los sensores
#define S1 4 // izquierda
#define S2 3 // centro
#define S3 2 // derecha

int lastState = 0; // -1: izquierda, 0: centro, 1: derecha
bool running = false;

void setup()
{
  Serial.begin(9600);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);

  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);

  Serial.println("Iniciando robot...");
}

void seguir()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  // Serial.println("seguir");
}

void parar()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  // Serial.println("parar");
}

void giroIzq()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  // Serial.println("giro izquierda");
  lastState = -1;
}

void giroDer()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println("giro derecha");
  lastState = 1;
}

void loop()
{
  int L = digitalRead(S1);
  int C = digitalRead(S2);
  int R = digitalRead(S3);

  // DEPURACIÓN SERIAL
  /*Serial.print("L=");
  Serial.print(L);
  Serial.print("  C=");
  Serial.print(C);
  Serial.print("  R=");
  Serial.print(R);
  Serial.print("  ---> ");*/

  if (running)
  {
    if (C == LOW && L == LOW && R == LOW)
    {
      if (lastState == -1)
      {
        giroIzq();
      }
      else if (lastState == 1)
      {
        giroDer();
      }
      else
      {
        seguir();
      }
    }
    else if (C == LOW && L == LOW && R == HIGH)
    {

      giroDer();
    }
    else if (C == LOW && L == HIGH && R == LOW)
    {
      giroIzq();
    }
    else if (C == LOW && L == HIGH && R == HIGH)
    {
      seguir();
    }
    else if (C == HIGH && L == LOW && R == LOW)
    {
      seguir();
    }
    else if (C == HIGH && L == LOW && R == HIGH)
    {
      giroDer();
    }
    else if (C == HIGH && L == HIGH && R == LOW)
    {
      giroIzq();
    }
    else if (C == HIGH && L == HIGH && R == HIGH)
    {
      parar();
    }
  }

  if (IrReceiver.decode())
  {

    uint32_t code = IrReceiver.decodedIRData.decodedRawData;

    Serial.println(code, HEX);

    // ======== arrancar =========
    if (code == 0xFC03EF00)
    {
      running = true;
    }

    // ======== detener =========
    if (code == 0xFD02EF00)
    {
      running = false;
      parar();
    }

    IrReceiver.resume();
  }

  delay(20);
}
