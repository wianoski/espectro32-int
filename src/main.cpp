/**
 ******************************************************************************
 * @file    DISCO_IOT_WakeUp.ino
 * @author  WI6LABS from AST
 * @version V1.0.0
 * @date    7 September 2017
 * @brief   Arduino test application for the STMicrolectronics STM32 IOT Discovery Kit.
 *          MEMS Inertial and Environmental sensor expansion board.
 *          This application detects wake up event through the LSM6DSL sensor.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


// Includes.#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include <LSM6DSLSensor.h>

#define SerialPort Serial
// #define I2C2_SCL    PB10
// #define I2C2_SDA    PB11
#define INT1 34


// Components.
// TwoWire Wire(I2C2_SDA, I2C2_SCL);
LSM6DSLSensor AccGyr(&Wire, LSM6DSL_ACC_GYRO_I2C_ADDRESS_HIGH);

//Interrupts.
volatile int mems_event = 0;

char report[256];

void INT1Event_cb();
void sendOrientation();

void setup() {
  // Led.
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize serial for output.
  SerialPort.begin(115200);

  // Initialize I2C bus.
  Wire.begin();

  //Interrupts.
  pinMode(INT1, INPUT_PULLDOWN);
  // pinMode(INT1, INPUT_PULLUP);
  attachInterrupt(INT1, INT1Event_cb, RISING);

  // Initlialize Components.
  AccGyr.begin();
  AccGyr.Enable_X();

  uint8_t id;
  AccGyr.ReadID(&id);

  Serial.printf("LSM6DSL ID: 0x%x\r\n", id);

  // Enable 6D Orientation.
  AccGyr.Enable_6D_Orientation();
  AccGyr.Enable_Wake_Up_Detection(LSM6DSL_INT1_PIN);
  AccGyr.Enable_Tilt_Detection(LSM6DSL_INT1_PIN);

  Serial.println("Beres setup");
  Serial.println(digitalRead(INT1));

  LSM6DSL_ACC_GYRO_INT_ACT_LEVEL_t interruptActLevel;
  if (LSM6DSL_ACC_GYRO_R_INT_ACT_LEVEL((void*)&AccGyr, &interruptActLevel) == MEMS_SUCCESS)
  {
    Serial.printf("Interrupt act level: %d\r\n", interruptActLevel);
  }
}

void loop() {
  if (mems_event)
  {
    // Serial.println("MEMS_EVENT!");
    mems_event = 0;
    LSM6DSL_Event_Status_t status;
    AccGyr.Get_Event_Status(&status);
    if (status.D6DOrientationStatus)
    {
      // Send 6D Orientation
	    sendOrientation();

      // Led blinking.
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
    }

    if (status.WakeUpStatus)
    {
      Serial.println("WAKEUP!");
    }

    if (status.FreeFallStatus)
    {
      Serial.println("FREE FALL!\r\n");
    }

    if (status.TiltStatus)
    {
      Serial.println("TILT!\r\n");
    }
  }
}

void INT1Event_cb()
{
  mems_event = 1;
}

void sendOrientation()
{
  uint8_t xl = 0;
  uint8_t xh = 0;
  uint8_t yl = 0;
  uint8_t yh = 0;
  uint8_t zl = 0;
  uint8_t zh = 0;

  AccGyr.Get_6D_Orientation_XL(&xl);
  AccGyr.Get_6D_Orientation_XH(&xh);
  AccGyr.Get_6D_Orientation_YL(&yl);
  AccGyr.Get_6D_Orientation_YH(&yh);
  AccGyr.Get_6D_Orientation_ZL(&zl);
  AccGyr.Get_6D_Orientation_ZH(&zh);

  if ( xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 1 && zh == 0 )
  {
    sprintf( report, "\r\n  ________________  " \
                      "\r\n |                | " \
                      "\r\n |  *             | " \
                      "\r\n |                | " \
                      "\r\n |                | " \
                      "\r\n |                | " \
                      "\r\n |                | " \
                      "\r\n |________________| \r\n" );
  }

  else if ( xl == 1 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 0 )
  {
    sprintf( report, "\r\n  ________________  " \
                      "\r\n |                | " \
                      "\r\n |             *  | " \
                      "\r\n |                | " \
                      "\r\n |                | " \
                      "\r\n |                | " \
                      "\r\n |                | " \
                      "\r\n |________________| \r\n" );
  }

  else if ( xl == 0 && yl == 0 && zl == 0 && xh == 1 && yh == 0 && zh == 0 )
  {
    sprintf( report, "\r\n  ________________  " \
                      "\r\n |                | " \
                      "\r\n |                | " \
                      "\r\n |                | " \
                      "\r\n |                | " \
                      "\r\n |                | " \
                      "\r\n |  *             | " \
                      "\r\n |________________| \r\n" );
  }

  else if ( xl == 0 && yl == 1 && zl == 0 && xh == 0 && yh == 0 && zh == 0 )
  {
    sprintf( report, "\r\n  ________________  " \
                      "\r\n |                | " \
                      "\r\n |                | " \
                      "\r\n |                | " \
                      "\r\n |                | " \
                      "\r\n |                | " \
                      "\r\n |             *  | " \
                      "\r\n |________________| \r\n" );
  }

  else if ( xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 1 )
  {
    sprintf( report, "\r\n  __*_____________  " \
                      "\r\n |________________| \r\n" );
  }

  else if ( xl == 0 && yl == 0 && zl == 1 && xh == 0 && yh == 0 && zh == 0 )
  {
    sprintf( report, "\r\n  ________________  " \
                      "\r\n |________________| " \
                      "\r\n    *               \r\n" );
  }

  else
  {
    sprintf( report, "None of the 6D orientation axes is set in LSM6DSL - accelerometer.\r\n" );
  }

  SerialPort.print(report);
}