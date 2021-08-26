// /**
//  ******************************************************************************
// //  * @file    DISCO_IOT_DoubleTap.ino
// //  * @author  WI6LABS FROM AST
// //  * @version V1.0.0
// //  * @date    7 September 2017
// //  * @brief   Arduino test application for the STMicrolectronics STM32 IOT Discovery Kit.
//  *          MEMS Inertial and Environmental sensor expansion board.
//  *          This application detects double tap event through the LSM6DSL sensor.
//  *          This application makes use of C++ classes obtained from the C
//  *          components' drivers.
//  ******************************************************************************
// //  * @attention
//  *
//  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
//  *
//  * Redistribution and use in source and binary forms, with or without modification,
//  * are permitted provided that the following conditions are met:
//  *   1. Redistributions of source code must retain the above copyright notice,
//  *      this list of conditions and the following disclaimer.
//  *   2. Redistributions in binary form must reproduce the above copyright notice,
//  *      this list of conditions and the following disclaimer in the documentation
//  *      and/or other materials provided with the distribution.
//  *   3. Neither the name of STMicroelectronics nor the names of its contributors
//  *      may be used to endorse or promote products derived from this software
//  *      without specific prior written permission.
//  *
//  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
//  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//  *
//  ******************************************************************************
//  */


// // Includes.
// #include <Arduino.h>
// #include<Wire.h>
// #include <LSM6DSLSensor.h>

// #define SerialPort Serial
// // #define I2C2_SCL    PB10
// // #define I2C2_SDA    PB11
// #define INT1        34

// // Components.
// // TwoWire Wire;
// LSM6DSLSensor AccGyr(&Wire, LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW);

// //Interrupts.
// volatile int mems_event = 0;

// void INT1Event_cb();

// void setup() {
//   // Led.
//   pinMode(LED_BUILTIN, OUTPUT);

//   // Initialize serial for output.
//   SerialPort.begin(9600);

//   // Initialize I2C bus.
//   Wire.begin();

//   //Interrupts.
//   attachInterrupt(INT1, INT1Event_cb, RISING);

//   // Initlialize Components.
//   AccGyr.begin();
//   AccGyr.Enable_X();

//   // Enable Double Tap Detection.
//   AccGyr.Enable_Double_Tap_Detection();
// }

// void loop() {
//   if (mems_event) {
//     mems_event = 0;
//     LSM6DSL_Event_Status_t status;
//     AccGyr.Get_Event_Status(&status);
//     if (status.DoubleTapStatus)
//     {
//       // Led blinking.
//       digitalWrite(LED_BUILTIN, HIGH);
//       delay(100);
//       digitalWrite(LED_BUILTIN, LOW);
//       delay(100);
//       digitalWrite(LED_BUILTIN, HIGH);
//       delay(100);
//       digitalWrite(LED_BUILTIN, LOW);

//       // Output data.
//       SerialPort.println("Double Tap Detected!");
//     }
//   }
// }

// void INT1Event_cb()
// {
//   mems_event = 1;
// }

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LSM6DSL.h>

// Uncomment this line for SPI mode. Assign the correct chip select pin to the constructor.
// LSM6DSLCore imu(LSM6DSL_MODE_SPI, 33);

// Using I2C mode by default.
LSM6DSLCore imu(LSM6DSL_MODE_I2C, 0x6B);

#define INT1_PIN 34
uint8_t int1Status;

void int1Handler() {
    int1Status++;
}

void setup() {
    Serial.begin(9600);
	delay(2000);

	Serial.println("Start!");

	if (imu.beginCore() != IMU_SUCCESS) {
		Serial.println("Failed initializing IMU sensor");
	}

	uint8_t errorAccumulator = 0;
	uint8_t dataToWrite = 0;

	// Setup accelerometer
	dataToWrite = 0;
	dataToWrite |= LSM6DSL_ACC_GYRO_FS_XL_2g;
	dataToWrite |= LSM6DSL_ACC_GYRO_ODR_G_416Hz;
    errorAccumulator += imu.writeRegister(LSM6DSL_ACC_GYRO_CTRL1_XL_REG, dataToWrite);

    // Enable tap detection on X, Y, and Z axis and disable latch interrupt.
    errorAccumulator += imu.writeRegister(LSM6DSL_ACC_GYRO_TAP_CFG, 0x0E);

    errorAccumulator += imu.writeRegister(LSM6DSL_ACC_GYRO_TAP_THS_6D, 0x03);

    errorAccumulator += imu.writeRegister(LSM6DSL_ACC_GYRO_INT_DUR2, 0x7F);

    errorAccumulator += imu.writeRegister(LSM6DSL_ACC_GYRO_WAKE_UP_THS, 0x80);

    errorAccumulator += imu.writeRegister(LSM6DSL_ACC_GYRO_MD1_CFG, 0x48);

    pinMode(INT1_PIN, INPUT);
    attachInterrupt(INT1_PIN, int1Handler, RISING);
}

void loop() {
    if (int1Status > 0) {
        delay(300);

        if (int1Status == 1) {
            Serial.println("Single-tap event");
        }

        if (int1Status > 1) {
            Serial.println("Double-tap event");
        }

        int1Status = 0;
    }
}
