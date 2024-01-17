#include <AK09918.h>
#include <Arduino.h>
#include <ICM20600.h>
#include <Wire.h>

int32_t x, y, z;

int32_t offset_x, offset_y, offset_z;
double roll, pitch;
AK09918 ak09918;
ICM20600 icm20600(true);
AK09918_err_type_t err;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Serial.begin(115200);
    while (!Serial) {
        delay(100);
    }
    Wire1.begin();
    Serial.println("waiting for serial");

    Serial.println("Starting setup");

    // err = ak09918.initialize();
    // Serial.println("AK09918 ID: " + String(ak09918.getDeviceID()));
    // Serial.println("ICM20600 ID: " + String(icm20600.getDeviceID()));
    // Serial.println("err: " + String(err));
    // icm20600.initialize();
    // ak09918.switchMode(AK09918_POWER_DOWN);
    // ak09918.switchMode(AK09918_CONTINUOUS_100HZ);
    //  err = ak09918.isDataReady();
    //  while (err != AK09918_ERR_OK) {
    //      Serial.println("Waiting Sensor");
    //      delay(100);
    //      err = ak09918.isDataReady();
    //      Serial.println("err: " + String(err));
    //  }

    Serial.println("Start figure-8 calibration after 2 seconds.");
    delay(2000);
    // calibrate(10000, &offset_x, &offset_y, &offset_z);
    Serial.println("");
}

void loop() {
    // get acceleration
    Serial.println("LOOPING");
    delay(500);
}
