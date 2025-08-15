#include <ESP32Servo.h>
#include <Bluepad32.h>

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

// Fungsi untuk mengecek apakah servo sudah mencapai posisi yang diinginkan
bool servoReachedTarget()
{
    // Toleransi 2-3 derajat untuk menghindari "hunting"
    return abs(posisiCapit - targetPosisiCapit) <= 2;
}

// Fungsi utama yang diperbaiki
void handleServoCapit(ControllerPtr ctl)
{
    unsigned long now = millis();

    bool buttonPressed = ctl->r1() || ctl->l1();

    if (buttonPressed)
    {
        uint8_t newTarget = targetPosisiCapit;
        if (ctl->r1())
        {
            newTarget = constrain(targetPosisiCapit + 20, 0, 120);
        }
        else if (ctl->l1())
        {
            newTarget = constrain(targetPosisiCapit - 20, 0, 120);
        }

        if (newTarget != targetPosisiCapit)
        {
            targetPosisiCapit = newTarget;
        }

        if (now - lastServoCapitUpdate >= servoDelayCapit)
        {
            if (posisiCapit < targetPosisiCapit)
            {
                posisiCapit = constrain(posisiCapit + 20, 0, 120);
            }
            else if (posisiCapit > targetPosisiCapit)
            {
                posisiCapit = constrain(posisiCapit - 20, 0, 120);
            }

            Serial.println("Posisi Capit: " + String(posisiCapit) + ", Target: " + String(targetPosisiCapit));

            servoCapit.write(posisiCapit);
            lastServoCapitUpdate = now;
        }
    }
}

void handleLengan(ControllerPtr ctl)
{
    unsigned long now = millis();
    if (now - lastServoLenganUpdate >= servoDelayLengan)
    {
        uint8_t oldPos = posisiLengan;

        // R2/L2 for arm control
        if (ctl->buttons() & BUTTON_TRIGGER_R)
        {
            posisiLengan = constrain(posisiLengan + 25, 0, 90);
        }
        else if (ctl->buttons() & BUTTON_TRIGGER_L)
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

const uint16_t SPEED_TRANSPORT = 200;
const uint16_t SPEED_TURN_SHARP = 240; // Aggressive turning
const uint16_t SPEED_PIVOT = 200;      // Balanced pivot speed

void setupPinMotor()
{
    pinMode(PIN_ENA, OUTPUT);
    pinMode(PIN_ENB, OUTPUT);

    ledcSetup(ledChannel1, frequency, resolution);
    ledcSetup(ledChannel2, frequency, resolution);
    ledcAttachPin(PIN_ENA, ledChannel1);
    ledcAttachPin(PIN_ENB, ledChannel2);

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

    ledcWrite(ledChannel1, abs(leftSpeed));
    ledcWrite(ledChannel2, abs(rightSpeed));
}

void stopMotors()
{
    setMotorSpeed(0, 0);
}

void handleControlMotor(ControllerPtr ctl)
{
    int leftSpeed = 0;
    int rightSpeed = 0;
    bool isMoving = false;

    uint8_t dpadState = ctl->dpad();

    uint16_t normalSpeed = SPEED_TRANSPORT;
    uint16_t turnSpeed = SPEED_TURN_SHARP;
    uint16_t pivotSpeed = SPEED_PIVOT;

    // berputar di tempat
    if (ctl->buttons() & BUTTON_X)
    {
        Serial.println("Berputar di tempat ke kiri");
        leftSpeed = -pivotSpeed;
        rightSpeed = pivotSpeed;
        isMoving = true;
    }
    else if (ctl->buttons() & BUTTON_B)
    {
        Serial.println("Berputar di tempat ke kanan");
        leftSpeed = pivotSpeed;
        rightSpeed = -pivotSpeed;
        isMoving = true;
    }

    // SINGLE DIRECTION MOVEMENTS
    else if (dpadState & DPAD_UP)
    {
        Serial.println("Bergerak maju");
        leftSpeed = normalSpeed;
        rightSpeed = normalSpeed;
        isMoving = true;
    }
    else if (dpadState & DPAD_DOWN)
    {
        Serial.println("Bergerak mundur");
        leftSpeed = -normalSpeed;
        rightSpeed = -normalSpeed;
        isMoving = true;
    }
    else if (dpadState & DPAD_LEFT)
    {
        Serial.println("Bergerak kiri");
        leftSpeed = turnSpeed * 0.25; // More aggressive
        rightSpeed = turnSpeed;
        isMoving = true;
    }
    else if (dpadState & DPAD_RIGHT)
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

/*------------------------- CONTROLLER HANDLER (NGAMBIL DARI CONTOH) ------------------------------- */

ControllerPtr myControllers[BP32_MAX_GAMEPADS];
bool controllerConnected = false;

void onConnectController(ControllerPtr ctl)
{
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++)
    {
        if (myControllers[i] == nullptr)
        {
            Serial.printf("CALLBACK: controller is connected, index=%d\n", i);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot)
    {
        Serial.println('CALLBACK: Controller connected, but could not found empty slot.');
    }

    controllerConnected = true;
}

void onDisconnectedController(ControllerPtr ctl)
{
    bool foundController = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++)
    {
        if (myControllers[i] == ctl)
        {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }
    if (!foundController)
    {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers.");
    }
    stopMotors();
    controllerConnected = false;
}

void processGamepad(ControllerPtr ctl)
{
    handleControlMotor(ctl);
    handleServoCapit(ctl);
    handleLengan(ctl);
}

void processControllers()
{
    for (auto myController : myControllers)
    {
        if (myController && myController->isConnected() && myController->hasData())
        {
            if (myController->isGamepad())
            {
                processGamepad(myController);
            }
            else
            {
                Serial.println("Unsupported controller");
            }
        }
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());

    BP32.setup(&onConnectController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    BP32.enableVirtualDevice(false);

    setupServo();
    setupPinMotor();

    Serial.println("Kode baru sudah masuk.25");
    Serial.println("Setup completed. Waiting for PS4 controller to connect...");
    Serial.println("To pair: Hold PS button + Share button on PS4 controller until it blinks rapidly");
}

void loop()
{
    bool updateData = BP32.update();
    if (updateData)
    {
        processControllers();
    }
    else if (!controllerConnected)
    {
        stopMotors();
    }

    vTaskDelay(1);
}