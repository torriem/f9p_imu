/*
  Perform terrain compensation on GPS position information from a
  GGA nmea serial source using a CMPS14 or BNO08x IMU.  Intended
  to run on an ESP32, and utilizes bluetooth to transmit RTCM
  data (if available) to the F9P over the same serial link.

  Based on Panda by Brian Tischler.  Brian's code is MIT licensed,
  but the modified Sparkfun BNO08x library is GPL.

  This code is released under the GPLv3.
*/

#include <BluetoothSerial.h>
#include <Wire.h>
#include "gganmea.h"
#include "BNO08x_AOG.h"
#include "haversine.h"

bool use_cmps = false;
bool use_bno08x = false;

//BNO08x addresses to look for:
const uint8_t bno08x_address[] = { 0x4A, 0x4B };
const int8_t num_BNO08x_adresses = 2;
uint8_t bno08xAddress;
BNO080 bno08x;
float bno08x_yawrate=0;
float bno08x_heading=0;
float bno08x_roll=0;
float bno08x_pitch=0;
const uint16_t GYRO_LOOP_TIME=10;
uint32_t last_gyro_time = 10;
uint32_t bno08x_last_time = 10;

//CMPS14
#define CMPS14_ADDRESS 0x60

//gps coordinate variables

double last_lat = -300;
double last_lon = -300;

//Serial speeds
const int32_t gps_speed = 57600;
const int32_t serial_speed = 115200;

//F9P serial port pins
#define GPSRX 13 //??
#define GPSTX 12 //??
#define SerialGPS Serial2

BluetoothSerial SerialBT;

char nmea_buffer[160];
GGANMEA ggaparser(nmea_buffer,sizeof(nmea_buffer));

//data we will use for compensation calculations
float imu_heading;
float imu_roll;
float imu_pitch;
float imu_yaw_rate;

void process_bno08x (uint32_t delta) {
	float gyro;

	if (bno08x.dataAvailable()) {
		gyro = bno08x.getGyroZ() * RAD_TO_DEG; //yaw rate deg/s
		//invert sign?

		bno08x_heading = bno08x.getYaw() * RAD_TO_DEG;
		//invert sign?

		if (bno08x_heading < 0)
			bno08x_heading += 360;

		if (swap_roll_pitch) {
			bno08x_roll = bno08x.getPitch() * RAD_TO_DEG;
			bno08x_pitch = bno08x.getRoll() * RAD_TO_DEG;
		} else {
			bno08x_roll = bno08x.getRoll() * RAD_TO_DEG;
			bno08x_pitch = bno08x.getPitch() * RAD_TO_DEG;
			//invert pitch sign?
		}

		//complementary filter on the gyro yaw rate
		bno08x_yawrate = 0.96 * bno08x_yawrate + 0.04 * gyro;
	}
	last_gyro_time = millis();

}

void process_imu() {
	if (use_cmps) {
		Wire.beginTransmission(CMPS14_ADDRESS);
		Wire.write(0x1C);
		Wire.endTransmission();

		Wire.requestFrom(CMPS14_ADDRESS, 2);
		while (Wire.available() < 2);

		imu_roll = (int16_t(Wire.read()) << 8 | Wire.read()) / 10.0;
		//perhaps filter

		Wire.beginTransmission(CMPS14_ADDRESS);
		Wire.write(0x02);
		Wire.endTransmission();

		Wire.requestFrom(CMPS14_ADDRESS, 3);
		while (Wire.available() < 3);

		imu_heading = (int16_t)(Wire.read() << 8 | Wire.read()) / 10.0;

		imu_pitch = Wire.read() / 10.0;

		imu_yaw_rate = 0; //not read on cmps14

	} else if (use_bno08x) {
		//copy the latest IMU data to our holding variables
		imu_heading = bno08x_heading;
		imu_pitch = bno08x_pitch;
		imu_roll = bno08x_roll;
		imu_yaw_rate = bno08x_yaw_rate;
	}
}

void output_gga() {
	float heading;
	double latitude;
	double longitude;
	double altitude;

	bool new_gga = false;

	latitude = ggaparser.getLatitude() / 10000000.0;
	longitude = ggaparser.getLongitude() / 10000000.0;
	altitude = ggaparser.getAltitude / 1000.0;

	if (use_bno08x) {
		;
	//if GGA speed is > threshold, calculate gyro offset using it, and
	//use GGA heading for terrain compensation below, perhaps filtering it

	//if we know our gyro offset, if we're slow or turning very fast, use the
	//gyro + offset as the heading for doing terrain compensation.

	//if we don't know our gyro offset, wait until we've moved a certain
	//distance, and use that fix heading to calculate the gyro_offset.
	} else {
		heading = gga.
		;
		//heading = gga heading since we have no gyro;
	}

	if (imu_roll && (ggaparser.getFixType() == 4 ||
	                 ggaparser.getFixType() == 5)) {
		//use the imu_roll to do terrain comp, adjust lat, lon, and altitude
		//build a new GGA sentence, transmit it via bluetooth, and over serial

	} else {
		//pass GGA sentence through unaltered.
		Serial.write(nmea_buffer,nmea_buffer.sentence_length);
		Serial.write('\n');
		SerialBT.write(nmea_buffer,nmea_buffer.sentence_length);
		SerialBT.write('\n');

	}

	if (use_bno08x) {
		bno08x_last_time = millis();
		is_triggered = true;
	}




void setup()
{
	Serial.begin(serial_speed);
	SerialGPS.begin(gps_speed, SERIAL_8N1, GPSRX, GPSTX);
	SerialBT.begin("esp32_f9p");

	pinMode(13, OUTPUT); //for an LED
	//another LED to indicate IMU presence
	pinMode(IMULED, OUTPUT);

	Wire.begin();

	uint8_t error;

	Wire.beginTransmission(CPMS14_ADDRESS);

	if(!Wire.endTransmission()) {
		use_cmps = true;
	} else {
		//try the BNO08x
		for (int8_t i=0 ; i < num_BNO08x_addresses ; i++) {
			Wire.beginTransmission(bno08x_address[i])
			if (! Wire.endTransmission() ) {
				//we found the BNO08x
				if (bno08x.begin(bno08x_address[i]) {
					//increase I2C data rate to 400 kHz
					Wire.setClock(400000);

					bno08x.enableGyro(GYRO_LOOP_TIME);
					bno08x.enableGyroIntegratedRotationVector(GYRO_LOOP_TIME-1);

					//check to see if Rotation Vector report is enabled
					if (bno08x.getFeatureResponseAvailable()) {
						if (! bno08x.checkReportEnable(SENSOR_REPORTID_GAME_ROTATION_VECTOR, (GYRO_LOOP_TIME - 1))) {
							use_bno08x = true;
							break;
						}
					}
				}
			}
		}
	}

	if(use_cmps || use_bno08x) {
		digitalWrite(IMULED, HIGH);
	} else {
		digitalWrite(IMULED, LOW);
	}
}

void loop()
{	
	uint32_t current_time;

	//Read RTCM data from bluetooth
	for (int8_t i=0; i < SerialBT.available(); i++) {
		SerialGPS.write(SerialBT.read());
	}

	//Read NMEA data from GPS
	//for (uint8_t i=0; i < SerialGPS.available(); i++) {
	if (SerialGPS.available()) { //just one byte at a time?
		if(ggaparser.process(SerialGPS.read()) {
			//we've got a sentence
			if (use_bno08x) {
				output_gga(); //this will set is_triggered
			} else {
				gps_ready_time = millis();
				is_triggered = true;
			}
			//break;
		}
	}

	current_time = millis();
	
	if (use_bno08x) {
		if (is_triggered && current_time - bno08x_last_time >= IMU_DELAY_TIME) {
			//fetch the imu parameters that will be used for the
			//subsequent GPS fix.  We get them ahead of time so that
			//they might line up better due to the latency in the GPS
			//fix calculations

			process_imu();
			is_triggered = false;
			current_time = millis();
		}
		if (current_time - last_gyro_time >= GYRO_LOOP_TIME) {
			process_bno08x(current_time - last_gyro_time);
		}
	} else if (use_cmps) {
		if (is_triggered && current_time - gps_ready_time >= CMPS_DELAY_TIME) {
			process_imu();
			output_gga();

			is_triggered = false;
			current_time = millis();


}


