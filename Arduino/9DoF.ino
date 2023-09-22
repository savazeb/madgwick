#include <Adafruit_ICM20948.h>
#include <Adafruit_ICM20X.h>

Adafruit_ICM20948 icm;
Adafruit_Sensor *icm_temp, *icm_accel, *icm_gyro, *icm_mag;

#define ICM_CS 10
// For software-SPI mode we need SCK/MOSI/MISO pins
#define ICM_SCK 13
#define ICM_MISO 12
#define ICM_MOSI 11

unsigned long timer = 0;
long loopTime = 10000; // microseconds

void setup(void)
{
    Serial.begin(115200);
    while (!Serial)
        delay(10); // will pause Zero, Leonardo, etc until serial console opens

    if (!icm.begin_I2C())
    {
        // if (!icm.begin_SPI(ICM_CS)) {
        // if (!icm.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI)) {
        while (1)
        {
            delay(10);
        }
    }

    icm_temp = icm.getTemperatureSensor();
    icm_accel = icm.getAccelerometerSensor();
    icm_gyro = icm.getGyroSensor();
    icm_mag = icm.getMagnetometerSensor();

    timer = micros();
}

void loop()
{
    //  /* Get a new normalized sensor event */
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    sensors_event_t mag;
    icm_temp->getEvent(&temp);
    icm_accel->getEvent(&accel);
    icm_gyro->getEvent(&gyro);
    icm_mag->getEvent(&mag);

    double gx = double(gyro.gyro.x), gy = double(gyro.gyro.y), gz = double(gyro.gyro.z);
    double ax = double(accel.acceleration.x), ay = double(accel.acceleration.y), az = double(accel.acceleration.z);
    double mx = double(mag.magnetic.x), my = double(mag.magnetic.y), mz = double(mag.magnetic.z);

    //  Serial.print(gx);
    //  Serial.print(",");
    //  Serial.print(gy);
    //  Serial.print(",");
    //  Serial.print(gz);
    //  Serial.print(",");
    Serial.print(ax);
    Serial.print(",");
    Serial.print(ay);
    Serial.print(",");
    Serial.print(az);
    Serial.print(",");
    //  Serial.print(mx);
    //  Serial.print(",");
    //  Serial.print(my);
    //  Serial.print(",");
    //  Serial.print(mz);

    Serial.println();

    //  sendToPC(&gx);
    //  sendToPC(&gx, &gy, &gz,
    //           &ax, &ay, &az,
    //           &mx, &my, &mz);

    delay(100);
}

void sendToPC(double *data1)
{
    byte *byteData1 = (byte *)(data1);
    byte buf[4] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3]};
    Serial.write(buf, 4);
}

void sendToPC(double *data1, double *data2, double *data3,
              double *data4, double *data5, double *data6,
              double *data7, double *data8, double *data9)
{
    byte *byteData1 = (byte *)(data1);
    byte *byteData2 = (byte *)(data2);
    byte *byteData3 = (byte *)(data3);
    byte *byteData4 = (byte *)(data4);
    byte *byteData5 = (byte *)(data5);
    byte *byteData6 = (byte *)(data6);
    byte *byteData7 = (byte *)(data7);
    byte *byteData8 = (byte *)(data8);
    byte *byteData9 = (byte *)(data9);
    byte buf[36] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                    byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                    byteData3[0], byteData3[1], byteData3[2], byteData3[3],
                    byteData4[0], byteData4[1], byteData4[2], byteData4[3],
                    byteData5[0], byteData5[1], byteData5[2], byteData5[3],
                    byteData6[0], byteData6[1], byteData6[2], byteData6[3],
                    byteData7[0], byteData7[1], byteData7[2], byteData7[3],
                    byteData8[0], byteData8[1], byteData8[2], byteData8[3],
                    byteData9[0], byteData9[1], byteData9[2], byteData9[3]};
    Serial.write(buf, 36);
}
