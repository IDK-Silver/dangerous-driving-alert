#include <WiFi.h>
#include <Wire.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ArtronShop_LineNotify.h>
#include <ToneESP32.h>

// sensor
Adafruit_MPU6050 mpu;
ToneESP32 buzzer(25, 0);

// config
#define LINE_TOKEN              "nqmKbehEWb4ClLh5KvsZoiN7bCB7RHuoRlRSVbiKbIB"
#define WIFI_SSID               "DANG_DRIVING"
#define WIFI_PASSWORD           "DANG_DRIVING"
#define SAMPLING_TIME           100
#define SHAKE_TRIGGER_VALUE     0.06
#define WIFI_CONNECT_TIME_LIMIT ((30) * (1000))
#define DEBUG_LOG

// calculates the difference in acceleration
double accel_diff(const double & prev, const double & future, const double & time);


void setup() {

    // init serial
    Serial.begin(115200);

    // try to init mpu
    while (!mpu.begin())
    {
        Serial.println("failed to find MPU6050 chip");
        delay(1000);
    }
    Serial.println("MPU6050 Found!");

    // set up mpu
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);


    // try to connect Wi-Fi
    auto start_time = millis();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi");
    while (WiFiClass::status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");

        if ((millis() - start_time) >= WIFI_CONNECT_TIME_LIMIT)
        {
            Serial.println("\nWiFi connect time out");

            // failed sound
            for (int i = 0; i < 3; i++)
            {
                buzzer.tone(NOTE_C4, 100);
                delay(100);
            }
            buzzer.noTone();

            break;
        }
    }

    // print the Wi-Fi info
    if (WiFiClass::status() == WL_CONNECTED)
    {
        Serial.println("\nConnected to WiFi");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
    }

    // init Line notify robot
    LINE.begin(LINE_TOKEN);
}

void loop() {

    // get init value
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);


    // current acceleration value
    double current = accel.acceleration.y;

    // previous acceleration value
    double prev = current;

    while (true)
    {

        // update value
        mpu.getEvent(&accel, &gyro, &temp);
        current = accel.acceleration.y;

        // get shark value by differentiation
        double shake_value = accel_diff(
                prev, current,(SAMPLING_TIME / 2.0)
        );

        // update previous acceleration value
        prev = current;


        if (shake_value >= SHAKE_TRIGGER_VALUE)
        {
            buzzer.tone(NOTE_C4, 300);
            buzzer.noTone();
            Serial.println("dangerous driving");
            if (WiFiClass::status() == WL_CONNECTED)
            {
                LINE.send("偵測到危險駕駛行為");
            }

        }

    #ifdef DEBUG_LOG
        Serial.print("diff value : ");
        Serial.println(shake_value);
    #endif

        // wait next sampling time
        delay(SAMPLING_TIME);
    }

}


/**
 * Calculates the difference in acceleration between two positions over a given time interval.
 *
 * @param prev The previous position.
 * @param future The future position.
 * @param time The time interval between the two positions.
 * @return The difference in acceleration.
 */
double accel_diff(const double & prev, const double & future, const double & time)
{
    const double epsilon = 1e-6;

    if (fabs(time) < epsilon) {
        return 0.0;
    }

    return (prev - future) / (2 * time);
}