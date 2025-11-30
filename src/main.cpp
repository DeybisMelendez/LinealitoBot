#include <Arduino.h>
#include <IRremote.h>

#define IR_PIN 9

#define MOTOR_ENB_PIN 11
#define MOTOR_IN3_PIN 12
#define MOTOR_IN2_PIN 8
#define MOTOR_ENA_PIN 10

#define SENSOR_L2_PIN 7
#define SENSOR_L1_PIN 5
#define SENSOR_C_PIN 4
#define SENSOR_R1_PIN 3
#define SENSOR_R2_PIN 2

#define MAX_SPEED 180
#define CORRECTION_SLIGHT 45
#define CORRECTION_MODERATE 90
#define CORRECTION_HARD 180

const uint32_t LOOP_INTERVAL_US = 16666; // 16.666 ms = 60 Hz
uint32_t lastLoopTime = 0;

uint8_t sensorL2 = LOW;
uint8_t sensorL1 = LOW;
uint8_t sensorC = LOW;
uint8_t sensorR1 = LOW;
uint8_t sensorR2 = LOW;

int speedLeft = 0;
int correctionLeft = 0;
int lastCorrectionLeft = 0;

int speedRight = 0;
int correctionRight = 0;
int lastCorrectionRight = 0;

uint8_t positionError = 0;

// Estados del robot
enum State
{
  STATE_IDLE,
  STATE_RUNNING,
  STATE_CALIBRATION,
  STATE_DEBUG
};
State state = STATE_IDLE;

enum
{
  DIR_CENTER,
  DIR_SLIGHT_LEFT,
  DIR_MODERATE_LEFT,
  DIR_SLIGHT_RIGHT,
  DIR_MODERATE_RIGHT,
  DIR_LOST,
  DIR_CROSS,
  DIR_OTHER
};

const uint8_t LUT[32] = {
    /* 0b00000 */ DIR_LOST,
    /* 0b00001 */ DIR_MODERATE_RIGHT,
    /* 0b00010 */ DIR_SLIGHT_RIGHT,
    /* 0b00011 */ DIR_SLIGHT_RIGHT,
    /* 0b00100 */ DIR_CENTER,
    /* 0b00101 */ DIR_OTHER,
    /* 0b00110 */ DIR_OTHER,
    /* 0b00111 */ DIR_OTHER,

    /* 0b01000 */ DIR_SLIGHT_LEFT,
    /* 0b01001 */ DIR_OTHER,
    /* 0b01010 */ DIR_OTHER,
    /* 0b01011 */ DIR_OTHER,
    /* 0b01100 */ DIR_CENTER,
    /* 0b01101 */ DIR_CENTER,
    /* 0b01110 */ DIR_CENTER,
    /* 0b01111 */ DIR_CENTER,

    /* 0b10000 */ DIR_MODERATE_LEFT,
    /* 0b10001 */ DIR_OTHER,
    /* 0b10010 */ DIR_OTHER,
    /* 0b10011 */ DIR_OTHER,
    /* 0b10100 */ DIR_CENTER,
    /* 0b10101 */ DIR_CENTER,
    /* 0b10110 */ DIR_CENTER,
    /* 0b10111 */ DIR_CENTER,

    /* 0b11000 */ DIR_SLIGHT_LEFT,
    /* 0b11001 */ DIR_OTHER,
    /* 0b11010 */ DIR_OTHER,
    /* 0b11011 */ DIR_OTHER,
    /* 0b11100 */ DIR_CENTER,
    /* 0b11101 */ DIR_CENTER,
    /* 0b11110 */ DIR_CENTER,
    /* 0b11111 */ DIR_CROSS};

void stopMotors()
{
  analogWrite(MOTOR_ENA_PIN, 0);
  analogWrite(MOTOR_ENB_PIN, 0);
  digitalWrite(MOTOR_IN2_PIN, LOW);
  digitalWrite(MOTOR_IN3_PIN, LOW);
}

void forward()
{
  digitalWrite(MOTOR_IN2_PIN, HIGH);
  digitalWrite(MOTOR_IN3_PIN, HIGH);
  analogWrite(MOTOR_ENA_PIN, speedRight);
  analogWrite(MOTOR_ENB_PIN, speedLeft);
}

void checkIRCommands()
{
  if (IrReceiver.decode())
  {
    uint32_t code = IrReceiver.decodedIRData.decodedRawData;

    if (code == 0xFC03EF00 && state != STATE_RUNNING)
    {
      state = STATE_RUNNING;
      Serial.println("New state: STATE_RUNNING");
    }

    if (code == 0xFD02EF00 && state != STATE_IDLE)
    {
      state = STATE_IDLE;
      stopMotors();
      Serial.println("New state: STATE_IDLE");
    }

    if (code == 0xFB04EF00 && state != STATE_DEBUG)
    {
      state = STATE_DEBUG;
      stopMotors();
      Serial.println("New state: STATE_DEBUG");
    }
    IrReceiver.resume();
  }
}

uint8_t read_sensors()
{
  uint8_t s = 0;
  uint8_t l2 = digitalRead(SENSOR_L2_PIN);
  uint8_t l1 = digitalRead(SENSOR_L1_PIN);
  uint8_t c = digitalRead(SENSOR_C_PIN);
  uint8_t r1 = digitalRead(SENSOR_R1_PIN);
  uint8_t r2 = digitalRead(SENSOR_R2_PIN);
  positionError = r1 + r2 + r2 - l1 - l2 - l2;

  s |= (l2 << 4);
  s |= (l1 << 3);
  s |= (c << 2);
  s |= (r1 << 1);
  s |= (r2 << 0);

  return s;
}

void run()
{

  uint32_t now = micros();
  if (now - lastLoopTime < LOOP_INTERVAL_US)
  {
    return;
  }
  lastLoopTime = now;

  switch (LUT[read_sensors()])
  {
  case DIR_CENTER: // Sin corrección
    break;
  case DIR_SLIGHT_LEFT:
    correctionLeft = CORRECTION_SLIGHT;
    break;
  case DIR_MODERATE_LEFT:
    correctionLeft = CORRECTION_MODERATE;
    break;
  case DIR_SLIGHT_RIGHT:
    correctionRight = CORRECTION_SLIGHT;
    break;
  case DIR_MODERATE_RIGHT:
    correctionRight = CORRECTION_MODERATE;
    break;
  case DIR_LOST:
    if (positionError < 0)
    {
      correctionLeft = CORRECTION_SLIGHT;
    }
    else if (positionError > 0)
    {
      correctionRight = CORRECTION_SLIGHT;
    }
    break;
  case DIR_CROSS:
    stopMotors();
    delay(500);
    break;
  case DIR_OTHER:
    if (positionError < 0)
    {
      correctionLeft = CORRECTION_SLIGHT;
    }
    else if (positionError > 0)
    {
      correctionRight = CORRECTION_SLIGHT;
    }
    break;
  }
  correctionLeft = constrain(correctionLeft * 0.7 + lastCorrectionLeft * 0.3, 0, 255);
  correctionRight = constrain(correctionRight * 0.7 + lastCorrectionRight * 0.3, 0, 255);
  lastCorrectionLeft = correctionLeft;
  lastCorrectionRight = correctionRight;
  speedLeft = MAX_SPEED - correctionLeft;
  speedRight = MAX_SPEED - correctionRight;
}

void setup()
{
  Serial.begin(9600);

  pinMode(MOTOR_ENA_PIN, OUTPUT);
  pinMode(MOTOR_ENB_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  pinMode(MOTOR_IN3_PIN, OUTPUT);

  pinMode(SENSOR_L2_PIN, INPUT);
  pinMode(SENSOR_L1_PIN, INPUT);
  pinMode(SENSOR_C_PIN, INPUT);
  pinMode(SENSOR_R1_PIN, INPUT);
  pinMode(SENSOR_R2_PIN, INPUT);

  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);
}

void loop()
{
  checkIRCommands();

  switch (state)
  {
  case STATE_IDLE:
    stopMotors();
    break;
  case STATE_RUNNING:
    run();
    forward();
    break;
  case STATE_DEBUG:
    digitalWrite(MOTOR_IN2_PIN, HIGH);
    digitalWrite(MOTOR_IN3_PIN, HIGH);
    analogWrite(MOTOR_ENA_PIN, 255);
    analogWrite(MOTOR_ENB_PIN, 255);
    break;
  case STATE_CALIBRATION:
    Serial.print(digitalRead(SENSOR_L2_PIN));
    Serial.print(digitalRead(SENSOR_L1_PIN));
    Serial.print(digitalRead(SENSOR_R1_PIN));
    Serial.println(digitalRead(SENSOR_R2_PIN));
    delay(500);
    break;
  }
}
