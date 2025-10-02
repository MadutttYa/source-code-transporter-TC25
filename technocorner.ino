#include <ESP32Servo.h>
#include <PS4Controller.h>
#include <Arduino.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_err.h"

/*------------------------- SERVO STUFF ------------------------------- */

const uint8_t PIN_SERVO_CAPIT = 12;
const uint8_t PIN_SERVO_LENGAN = 14;

Servo servoCapit;
uint8_t posisiCapit = 0;
uint8_t targetPosisiCapit = 0;

Servo servoLengan;
uint8_t posisiLengan = 0;

uint8_t defaultServoPos = 0;

// Timing biar bisa bersamaan
// Tambahan, selama lomba ternyata gak perlu harus bersamaan sih... cuman mencegah pake delay saja...
unsigned long lastServoCapitUpdate = 0;
unsigned long lastServoLenganUpdate = 0;
const uint8_t servoDelayCapit = 50;
const uint8_t servoDelayLengan = 20;

void setupServo()
{
    servoCapit.attach(PIN_SERVO_CAPIT);
    servoCapit.write(defaultServoPos);
    servoLengan.attach(PIN_SERVO_LENGAN);
    servoLengan.write(20);
}

void handleServoCapit()
{
    unsigned long now = millis();

    bool buttonPressed = PS4.R1() || PS4.L1();

    if (buttonPressed)
    {
        uint8_t newTarget = targetPosisiCapit;
        if (PS4.R1())
        {
            newTarget = constrain(targetPosisiCapit + 15, 0, 55);
        }
        else if (PS4.L1())
        {
            newTarget = constrain(targetPosisiCapit - 15, 0, 55);
        }

        if (newTarget != targetPosisiCapit)
        {
            targetPosisiCapit = newTarget;
        }

        if (now - lastServoCapitUpdate >= servoDelayCapit)
        {
            if (posisiCapit < targetPosisiCapit)
            {
                posisiCapit = constrain(posisiCapit + 15, 0, 55);
            }
            else if (posisiCapit > targetPosisiCapit)
            {
                posisiCapit = constrain(posisiCapit - 15, 0, 55);
            }

            Serial.println("Posisi Capit: " + String(posisiCapit) + ", Target: " + String(targetPosisiCapit));

            servoCapit.write(posisiCapit);
            lastServoCapitUpdate = now;
        }
    }
}

void handleLengan()
{
    unsigned long now = millis();
    if (now - lastServoLenganUpdate >= servoDelayLengan)
    {
        uint8_t oldPos = posisiLengan;

        // R2/L2 for arm control
        if (PS4.Triangle())
        {
            posisiLengan = constrain(posisiLengan + 25, 0, 90);
        }
        else if (PS4.Cross())
        {
            posisiLengan = constrain(posisiLengan - 25, 20, 90);
        }

        if (oldPos != posisiLengan)
        {
            servoLengan.write(posisiLengan);
            lastServoLenganUpdate = now;
        }
    }
}

/*------------------------- MOTOR STUFF ------------------------------- */

const uint8_t PIN_ENA = 23;
const uint8_t PIN_ENB = 27;

const uint8_t PIN_IN1 = 18;
const uint8_t PIN_IN2 = 19;

const uint8_t PIN_IN3 = 33;
const uint8_t PIN_IN4 = 32;

const uint16_t frequency = 10000;
const uint8_t resolution = 8;
const uint8_t ledChannel1 = 3;
const uint8_t ledChannel2 = 4;

const uint16_t SPEED_TRANSPORT = 240;
const uint16_t SPEED_TURN_SHARP = 240; // Aggressive turning
const uint16_t SPEED_PIVOT = 200;      // Balanced pivot speed

void setupPinMotor()
{
    pinMode(PIN_ENA, OUTPUT);
    pinMode(PIN_ENB, OUTPUT);

    ledcAttach(PIN_ENA, frequency, resolution);
    ledcAttach(PIN_ENB, frequency, resolution);

    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_IN3, OUTPUT);
    pinMode(PIN_IN4, OUTPUT);
}

void setMotorSpeed(int leftSpeed, int rightSpeed)
{
    // Motor Kiri
    if (leftSpeed > 0)
    {
        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, HIGH);
    }
    else if (leftSpeed < 0)
    {
        digitalWrite(PIN_IN1, HIGH);
        digitalWrite(PIN_IN2, LOW);
    }
    else
    {
        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, LOW);
    }

    // Motor Kanan
    if (rightSpeed > 0)
    {
        digitalWrite(PIN_IN3, LOW);
        digitalWrite(PIN_IN4, HIGH);
    }
    else if (rightSpeed < 0)
    {
        digitalWrite(PIN_IN3, HIGH);
        digitalWrite(PIN_IN4, LOW);
    }
    else
    {
        digitalWrite(PIN_IN3, LOW);
        digitalWrite(PIN_IN4, LOW);
    }

    analogWrite(PIN_ENA, abs(leftSpeed));
    analogWrite(PIN_ENB, abs(rightSpeed));
}

void stopMotors()
{
    setMotorSpeed(0, 0);
}

void handleControlMotor()
{
    int leftSpeed = 0;
    int rightSpeed = 0;
    bool isMoving = false;

    uint16_t normalSpeed = SPEED_TRANSPORT;
    uint16_t turnSpeed = SPEED_TURN_SHARP;
    uint16_t pivotSpeed = SPEED_PIVOT;

    // berputar di tempat
    if (PS4.Square())
    {
        Serial.println("Berputar di tempat ke kiri");
        leftSpeed = -pivotSpeed;
        rightSpeed = pivotSpeed;
        isMoving = true;
    }
    else if (PS4.Circle())
    {
        Serial.println("Berputar di tempat ke kanan");
        leftSpeed = pivotSpeed;
        rightSpeed = -pivotSpeed;
        isMoving = true;
    }

    // SINGLE DIRECTION MOVEMENTS
    else if (PS4.Up())
    {
        Serial.println("Bergerak maju");
        leftSpeed = normalSpeed;
        rightSpeed = normalSpeed;
        isMoving = true;
    }
    else if (PS4.Down())
    {
        Serial.println("Bergerak mundur");
        leftSpeed = -normalSpeed;
        rightSpeed = -normalSpeed;
        isMoving = true;
    }
    else if (PS4.Left())
    {
        Serial.println("Bergerak kiri");
        leftSpeed = turnSpeed * 0.25; // More aggressive
        rightSpeed = turnSpeed;
        isMoving = true;
    }
    else if (PS4.Right())
    {
        Serial.println("Bergerak kanan");
        leftSpeed = turnSpeed;
        rightSpeed = turnSpeed * 0.25; // More aggressive
        isMoving = true;
    }

    if (isMoving)
    {
        setMotorSpeed(leftSpeed, rightSpeed);
    }
    else
    {
        stopMotors();
    }
}

void removePairedDevices() {
  uint8_t pairedDeviceBtAddr[20][6];
  int count = esp_bt_gap_get_bond_device_num();
  esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
  for (int i = 0; i < count; i++) {
    esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
  }
}

void printDeviceAddress() {
  const uint8_t* point = esp_bt_dev_get_address();
  for (int i = 0; i < 6; i++) {
    char str[3];
    sprintf(str, "%02x", (int)point[i]);
    Serial.print(str);
    if (i < 5) {
      Serial.print(":");
    }
  }
}

void setup() {
  Serial.begin(115200);
  
  setupServo();
  setupPinMotor();
  
  // Inisialisasi PS4 Controller
  // Ganti dengan MAC address PS4 controller Anda
  PS4.begin();
  removePairedDevices();
  Serial.print("This device MAC is: ");
  printDeviceAddress();
  Serial.println("");
  
  Serial.println("ESP32 siap, hubungkan PS4 controller...");
}

void loop() {
  // Cek jika PS4 controller terhubung
  if (PS4.isConnected()) {
    handleServoCapit();
    handleLengan();
    handleControlMotor();
  }
}