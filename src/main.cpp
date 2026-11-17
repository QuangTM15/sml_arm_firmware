/**
 * @file    main.cpp
 * @brief   Single joint hardware test application.
 * @project SML Arm Firmware
 * @version 0.1.0
 */

#include <Arduino.h>
#include <math.h>
#include <stdlib.h>

#include "board_config.h"
#include "robot_config.h"
#include "StepperDriver.h"

using namespace SML;

// ================= STEPPER ENGINE =================

FastAccelStepperEngine engine;

// ================= MOTOR INSTANCE =================

StepperDriver motor1(
    engine,
    BoardConfig::JOINT_1_STEP_PIN,
    BoardConfig::JOINT_1_DIR_PIN,
    BoardConfig::DIR_HIGH_COUNTS_UP,
    BoardConfig::DIR_DELAY_US);

// ================= TEST CONFIGURATION =================

constexpr float STEPS_PER_REVOLUTION =
    RobotConfig::JOINT_1.pulsesPerRevolution *
    RobotConfig::JOINT_1.gearRatio;

constexpr float STEPS_PER_DEGREE =
    STEPS_PER_REVOLUTION / 360.0f;

constexpr float TEST_ANGLE = 30.0f;

constexpr uint32_t TEST_SPEED = 4000U;
constexpr uint32_t TEST_ACCELERATION = 3000U;
constexpr uint32_t TEST_LINEAR_ACCELERATION = 200U;

// Temporary bench test limits.
constexpr float MIN_ANGLE = -90.0f;
constexpr float MAX_ANGLE = 90.0f;

constexpr uint32_t MAX_TEST_SPEED = 8000U;
constexpr uint32_t MAX_TEST_ACCELERATION = 6000U;

// ================= SYSTEM VARIABLES =================

bool driverReady = false;
bool wasRunning = false;

char commandBuffer[64];
uint8_t commandLength = 0;
bool commandOverflow = false;

// ================= UNIT CONVERSION =================

int32_t angleToSteps(float angle)
{
  return static_cast<int32_t>(
      lroundf(angle * STEPS_PER_DEGREE));
}

float stepsToAngle(int32_t steps)
{
  return static_cast<float>(steps) /
         STEPS_PER_DEGREE;
}

// ================= PRINT FUNCTIONS =================

void printResult(ErrorCode result)
{
  if (result == ErrorCode::OK)
  {
    Serial.println("[OK]");
  }
  else
  {
    Serial.printf(
        "[ERROR] Code: %u\n",
        static_cast<unsigned int>(result));
  }
}

void printHelp()
{
  Serial.println();
  Serial.println("===== SML ARM JOINT TEST =====");

  Serial.println("f     : Forward 30 degrees");
  Serial.println("b     : Backward 30 degrees");
  Serial.println("a90   : Absolute angle 90");
  Serial.println("r30   : Relative angle +30");
  Serial.println("r-30  : Relative angle -30");

  Serial.println("v4000 : Set speed (Hz)");
  Serial.println("c3000 : Set acceleration");
  Serial.println("l200  : Set linear acceleration");

  Serial.println("o     : Return to zero");
  Serial.println("z     : Set zero reference");
  Serial.println("s     : Soft stop");
  Serial.println("x     : Hard stop");
  Serial.println("p     : Print status");
  Serial.println("h     : Print help");

  Serial.println("==============================");
}

void printStatus()
{
  Serial.println();
  Serial.println("===== JOINT 1 STATUS =====");

  Serial.printf(
      "Position        : %ld steps\n",
      static_cast<long>(
          motor1.getCurrentPosition()));

  Serial.printf(
      "Estimated angle : %.3f deg\n",
      stepsToAngle(motor1.getCurrentPosition()));

  Serial.printf(
      "Speed limit     : %lu Hz\n",
      static_cast<unsigned long>(
          motor1.getSpeedHz()));

  Serial.printf(
      "Angular speed   : %.3f deg/s (limit)\n",
      motor1.getSpeedHz() / STEPS_PER_DEGREE);

  Serial.printf(
      "Acceleration    : %lu steps/s^2\n",
      static_cast<unsigned long>(
          motor1.getAcceleration()));

  Serial.printf(
      "Linear Accel    : %lu steps\n",
      static_cast<unsigned long>(
          motor1.getLinearAcceleration()));

  Serial.printf(
      "Running         : %s\n",
      motor1.isRunning() ? "YES" : "NO");

  Serial.printf(
      "Position valid  : %s\n",
      motor1.isPositionValid() ? "YES" : "NO");

  Serial.println("==========================");
}

// ================= MOTION TEST =================

ErrorCode moveToAngle(float angle)
{
  if (!isfinite(angle))
  {
    return ErrorCode::INVALID_PARAMETER;
  }

  if ((angle < MIN_ANGLE) || (angle > MAX_ANGLE))
  {
    return ErrorCode::OUT_OF_RANGE;
  }

  return motor1.moveToSteps(angleToSteps(angle));
}

ErrorCode moveRelativeAngle(float angle)
{
  if (!isfinite(angle))
  {
    return ErrorCode::INVALID_PARAMETER;
  }

  float currentAngle =
      stepsToAngle(motor1.getCurrentPosition());

  return moveToAngle(currentAngle + angle);
}

// ================= PARAMETER PARSING =================

bool parseValue(const char *text, float &value)
{
  char *end = nullptr;

  value = strtof(text, &end);

  return (end != text) &&
         (*end == '\0') &&
         isfinite(value);
}

// ================= COMMAND PROCESSING =================

void processCommand(const char *command)
{
  char type = command[0];
  float value = 0.0f;

  if (type == 'a' ||
      type == 'r' ||
      type == 'v' ||
      type == 'c' ||
      type == 'l')
  {
    if (!parseValue(command + 1, value))
    {
      printResult(ErrorCode::INVALID_PARAMETER);
      return;
    }
  }
  else if (command[1] != '\0')
  {
    printResult(ErrorCode::INVALID_PARAMETER);
    return;
  }

  switch (type)
  {
  case 'f':
    printResult(moveRelativeAngle(TEST_ANGLE));
    break;

  case 'b':
    printResult(moveRelativeAngle(-TEST_ANGLE));
    break;

  case 'a':
    printResult(moveToAngle(value));
    break;

  case 'r':
    printResult(moveRelativeAngle(value));
    break;

  case 'v':
    if (motor1.isRunning())
    {
      printResult(ErrorCode::BUSY);
    }
    else if (value < 1.0f ||
             value > MAX_TEST_SPEED ||
             floorf(value) != value)
    {
      printResult(ErrorCode::OUT_OF_RANGE);
    }
    else
    {
      printResult(motor1.setSpeedHz(
          static_cast<uint32_t>(value)));
    }
    break;

  case 'c':
    if (motor1.isRunning())
    {
      printResult(ErrorCode::BUSY);
    }
    else if (value < 1.0f ||
             value > MAX_TEST_ACCELERATION ||
             floorf(value) != value)
    {
      printResult(ErrorCode::OUT_OF_RANGE);
    }
    else
    {
      printResult(motor1.setAcceleration(
          static_cast<uint32_t>(value)));
    }
    break;

  case 'l':
    if (motor1.isRunning())
    {
      printResult(ErrorCode::BUSY);
    }
    else if (value < 0.0f || value > 5000.0f ||
             floorf(value) != value)
    {
      printResult(ErrorCode::OUT_OF_RANGE);
    }
    else
    {
      printResult(motor1.setLinearAcceleration(
          static_cast<uint32_t>(value)));
    }
    break;

  case 'o':
    printResult(moveToAngle(0.0f));
    break;

  case 'z':
    printResult(motor1.setCurrentPosition(0));
    break;

  case 's':
    printResult(motor1.stop(StopMode::SOFT));
    break;

  case 'x':
    printResult(motor1.stop(StopMode::HARD));
    break;

  case 'p':
    printStatus();
    break;

  case 'h':
    printHelp();
    break;

  default:
    Serial.println("[ERROR] Unknown command");
    break;
  }
}

// ================= SERIAL HANDLER =================

void updateSerial()
{
  // Bound processing time per loop.
  for (uint8_t i = 0;
       i < 64U && Serial.available() > 0;
       i++)
  {
    char data = static_cast<char>(Serial.read());

    if (data == '\r' || data == '\n')
    {
      if (commandOverflow)
      {
        Serial.println("[ERROR] Command overflow");
      }
      else if (commandLength > 0)
      {
        commandBuffer[commandLength] = '\0';

        processCommand(commandBuffer);
      }

      commandLength = 0;
      commandOverflow = false;
    }
    else if (!commandOverflow)
    {
      if (commandLength <
          sizeof(commandBuffer) - 1)
      {
        commandBuffer[commandLength++] = data;
      }
      else
      {
        commandOverflow = true;
      }
    }
  }
}

// ================= SETUP =================

void setup()
{
  Serial.begin(BoardConfig::SERIAL_BAUDRATE);

  delay(1000);

  Serial.println();
  Serial.println("SML ARM FIRMWARE v0.1.0");
  Serial.println("Single Joint Hardware Test");

  engine.init();

  if (motor1.begin() != ErrorCode::OK)
  {
    Serial.println("[FAIL] Driver initialization");
    return;
  }

  if (motor1.setSpeedHz(TEST_SPEED) != ErrorCode::OK ||
      motor1.setAcceleration(TEST_ACCELERATION) != ErrorCode::OK ||
      motor1.setLinearAcceleration(
          TEST_LINEAR_ACCELERATION) != ErrorCode::OK)
  {
    Serial.println("[FAIL] Motion configuration");
    return;
  }

  driverReady = true;

  Serial.println("[PASS] Driver initialized");

  Serial.printf(
      "Steps per degree: %.4f\n",
      STEPS_PER_DEGREE);

  Serial.println("Send z to establish zero reference.");

  printHelp();
  printStatus();
}

// ================= MAIN LOOP =================

void loop()
{
  if (!driverReady)
  {
    return;
  }

  updateSerial();

  bool isRunningNow = motor1.isRunning();

  if (wasRunning && !isRunningNow)
  {
    Serial.println("[EVENT] Motor stopped");

    printStatus();
  }

  wasRunning = isRunningNow;
}
