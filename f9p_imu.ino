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
#include "simplenmea.h"
#include "BNO08x_AOG.h"
#include "haversine.h"

bool use_cmps = false;
bool use_bno08x = false;

//BNO08x addresses to look for:
const uint8_t bno08x_address[] = { 0x4A, 0x4B };
const int8_t num_BNO08x_addresses = 2;
uint8_t bno08xAddress;
BNO080 bno08x;
float bno08x_yawrate=0;
float bno08x_heading=0;
float bno08x_roll=0;
float bno08x_pitch=0;
const uint16_t GYRO_LOOP_TIME=10;
uint32_t last_gyro_time = 10;
uint32_t bno08x_last_time = 10;
bool swap_roll_pitch = false;
const uint16_t IMU_DELAY_TIME = 190; //10 ms before the next message
float gyro_sum;

float rollK=0, Pc=0, G=0, Xp=0, Zp=0, XeRoll=0, P = 1.0f;
float varRoll = 0.1f, varProcess = 0.0003f;

//CMPS14
#define CMPS14_ADDRESS 0x60
const uint16_t CMPS_DELAY_TIME = 4;
uint32_t gps_ready_time = CMPS_DELAY_TIME;

bool is_triggered = false;

//gps coordinate variables

double last_lat = -301;
double last_lon = -301;

//Serial speeds
const int32_t gps_speed = 57600;
const int32_t serial_speed = 115200;

//F9P serial port pins
#define GPSRX 17 
#define GPSTX 16

#define SerialGPS Serial2

BluetoothSerial SerialBT;

char nmea_buffer[2][160]; //two buffers
char *gga_buffer;
char *vtg_buffer;
uint8_t which_nmea_buffer=0;
SimpleNMEAParser nmeaparser(nmea_buffer[0],sizeof(nmea_buffer[0]));
uint32_t last_gga = 0;
uint32_t last_vtg = 0;

char gga_output[100];
char vtg_output[80];

//data we will use for compensation calculations
float imu_heading = 0;
float imu_roll = 0;
float imu_pitch = 0;
float imu_yaw_rate = 0;
float imu_gyro_offset = -401;

#define SPEED_THRESHOLD 3 //3 kph
#define FIX_HEADING_MIN 1 //1 metre
#define YAW_RATE_THRESHOLD 6 //deg/sec

#define INCHES 

float antenna_height = 120 * 0.0254 /*metres/inch*/;
float antenna_forward = 0;
float antenna_left = 0;

void add_checksum(char *buffer, uint8_t buf_size) {
	uint8_t sum = 0;
	char tmp;
	char checksum[4];

	//Serial.println(buf_size);
	
	for (int8_t i=1; i < buf_size ; i++)
		
		sum ^= buffer[i];

	//Serial.println(sum,16);
	sprintf(checksum,"*%02X",sum);
	strcat(buffer,checksum);
}

void process_bno08x (uint32_t delta) {
	float gyro;

	if (bno08x.dataAvailable()) {
		gyro = -bno08x.getGyroZ() * haversine::toDegrees; //yaw rate deg/s
		//invert sign?

		bno08x_heading = -bno08x.getYaw() * haversine::toDegrees;
		//invert sign? Yes in my case.

		if (isnan(bno08x_heading))
			bno08x_heading = 0;

		if (bno08x_heading < 0)
			bno08x_heading += 360;

		if (swap_roll_pitch) {
			rollK = bno08x.getPitch() * haversine::toDegrees;
			bno08x_pitch = bno08x.getRoll() * haversine::toDegrees;
		} else {
			rollK = bno08x.getRoll() * haversine::toDegrees;
			bno08x_pitch = bno08x.getPitch() * haversine::toDegrees;
			//invert pitch sign?
		}

		if (isnan(rollK))
			rollK = 0.0;

		//TODO: adjust roll for offset.

		//Kalman filter on roll
		Pc = P + varProcess;
		G = Pc / (Pc + varRoll);
		P = (1 - G) * Pc;
		Xp = XeRoll;
		Zp = Xp;
		XeRoll = (G * (rollK - Zp)) + Xp;
		bno08x_roll = XeRoll;

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
		imu_yaw_rate = bno08x_yawrate;
	}
}

void output_gga() {
	float heading;
	float gyro_heading;
	double latitude;
	double longitude;
	double altitude;
	long tempalt;

	bool new_gga = false;

	heading = -301;

	latitude = nmeaparser.getLatitude() / 10000000.0;
	longitude = nmeaparser.getLongitude() / 10000000.0;

	if(nmeaparser.getAltitude(tempalt)) {
		altitude = tempalt / 1000.0;
	} else
		altitude = 0;

	//TESTING:
	imu_gyro_offset = 0; //assume started pointing straight north

	if (use_bno08x) {
		if (imu_gyro_offset < -400) {
			//no offset is known.

			if (nmeaparser.getSpeed() > SPEED_THRESHOLD) {
				//assume we're going forward
				heading = nmeaparser.getCourse();
			} else {
				//assume going forward, but it's still slow, so we need to go at least
				//some distance
				if (last_lat < -300) {
					last_lat = nmeaparser.getLatitude() / 10000000.0;
					last_lon = nmeaparser.getLongitude() / 10000000.0;
				} else if (haversine::distance(last_lat, last_lon,
					       nmeaparser.getLatitude() / 10000000.0,
						   nmeaparser.getLongitude() / 10000000.0) > FIX_HEADING_MIN) {
					heading = haversine::bearing_degrees(
					              last_lat, last_lon,
					              nmeaparser.getLatitude() / 10000000.0,
								  nmeaparser.getLongitude() / 10000000.0);
					
				}

				if(heading > -300) {
					//we have a heading from GPS
					imu_gyro_offset = heading - imu_heading;
				}
			}
		} else {
			//we have a gyro offset we can use if needed.
			gyro_heading = imu_heading + imu_gyro_offset;

			//Serial.print(gyro_heading,2);
			//Serial.print(", fix head: ");
			//Serial.println(nmeaparser.getCourse(),2);

			//wrap around if needed
			if (gyro_heading < 0)         gyro_heading += 360;
			else if (gyro_heading >= 360) gyro_heading -= 360;

			if (nmeaparser.getSpeed() > SPEED_THRESHOLD &&
			    imu_yaw_rate < YAW_RATE_THRESHOLD) {
				//assume we're going forward, at a decent speed, and
				//not turning very fast.  We'll use mostly gps heading
				//with some gyro heading (adjust?)
				
				heading = nmeaparser.getCourse() * 0.6 +
				          gyro_heading * 0.4;

				//recompute offset
				imu_gyro_offset = heading - imu_heading;

			} else {
				//we're either going really slow, or turning very fast
				//rely on gyro with offset
				heading = gyro_heading;
			}


		}
	} else {
		heading = nmeaparser.getCourse();
		//we may have roll information from cmps14, but heading comes from gps
		//Serial.println(heading);
	}
	
	//TESTING
	//imu_roll = 1;  //1 degree to left?
	//heading = 0;

	Serial.print(heading);
	Serial.print(" hr ");
	Serial.println(imu_roll);

	if (heading > -300 && imu_roll &&
	    (nmeaparser.getFixType() == 4 ||
	    (nmeaparser.getFixType() == 5))) {

		//double lat, lon, alt;

		float heading90;
		float tilt_offset;
		float center_offset = 0;
		float alt_offset1;
		float alt_offset2 = 0;

		long latdegmin;
		long londegmin;
		long latminfrac;
		long lonminfrac;
		char latdir, londir;

		//use the imu_roll to do terrain comp, adjust lat, lon, and altitude
		//build a new GGA sentence, transmit it via bluetooth, and over serial

		//lat = latitude / 10000000.0;
		//lon = longitude / 10000000.0;
		//alt = altitude / 1000.0;

		Serial.print(latitude,7);
		Serial.print(", ");
		Serial.print(longitude,7);
		Serial.print(", ");
		Serial.print(altitude,7);
		Serial.print(" =>");

		heading90 = heading + 90;
		if (heading90 >= 360) heading90 -= 360;

		if (antenna_left) {
			center_offset = cos(imu_roll * haversine::toRadians) * antenna_left;
			alt_offset2 = sin(imu_roll * haversine::toRadians) * center_offset;
		}

		tilt_offset = sin(imu_roll * haversine::toRadians) * antenna_height;
		alt_offset1 = cos(imu_roll * haversine::toRadians) * antenna_height;

		haversine::move_distance_bearing(latitude, longitude, heading90, tilt_offset + center_offset);
		altitude -= (alt_offset1 - alt_offset2);

		if (antenna_forward) {
			//translate from GPS back to axle
			haversine::move_distance_bearing(latitude, longitude, heading, -antenna_forward);
		}

		//make a new GGA message. Copy unchanged parts from the original message
		//convert lat and lon to DDMM.xxxxxx

		Serial.print(latitude,7);
		Serial.print(", ");
		Serial.print(longitude,7);
		Serial.print(", ");
		Serial.print(altitude,7);
		Serial.print(" (");
		Serial.print(heading);
		Serial.print(", ");
		Serial.print(imu_roll);
		Serial.println(")");



		if (latitude < 0) {
			latdir = 'S';
			latitude = -latitude;
		} else {
			latdir = 'N';
		}

		if (longitude < 0) {
			londir = 'W';
			longitude = -longitude;
		} else {
			londir = 'E';
		}

		latdegmin = latitude; //get degrees part
		londegmin = longitude;

		latitude = (latitude - latdegmin) * 60.0; //isolate the minutes
		longitude = (longitude - londegmin) * 60.0;


		latdegmin = latdegmin * 100 + latitude; //add on minutes digits
		//Serial.println(latdegmin);
		londegmin = londegmin * 100 + longitude;

		latitude = latitude - (int)latitude; //isolate fractional part
		//Serial.println(latitude,7);
		longitude = longitude - (int)longitude;

		latminfrac = latitude * 1000000.0;
		//Serial.println(latminfrac);
		lonminfrac = longitude * 1000000.0;


		//Serial.write((uint8_t *)gga_buffer,strnlen((const char*)gga_buffer,160));
		//Serial.write('\n');

		//grab the original timestamp and stuff before the lat and lon
		memcpy(gga_output,gga_buffer,nmeaparser.before_latlon);
		sprintf(gga_output+nmeaparser.before_latlon,"%04d.%06d,%c,%05d.%06d,%c,",
		        latdegmin, latminfrac, latdir,
				londegmin, lonminfrac, londir);

		//copy the stuff after lat and lon, up to the altitude
		gga_buffer[nmeaparser.before_altitude] = '\0';
		strcat(gga_output,gga_buffer+nmeaparser.after_latlon);

		//write the modified altitude
		sprintf(gga_output+strlen(gga_output),"%.2f,", altitude);

		//now grab the rest of the original sentence
		strcat(gga_output,gga_buffer+nmeaparser.after_altitude);

		uint8_t buf_size = strlen(gga_output)-3;
		gga_output[buf_size] = 0;

		//redo the checksum
		add_checksum(gga_output, buf_size);

		Serial.println(gga_output);
		SerialBT.println(gga_output);

	} else {
		//pass GGA sentence through unaltered.
		Serial.println(gga_buffer);
		Serial.println(vtg_buffer);

		SerialBT.println(gga_buffer);
		SerialBT.println(vtg_buffer);
		/*
		Serial.write((uint8_t *)gga_buffer,strnlen((const char*)gga_buffer,160));
		Serial.write('\n');
		Serial.write((uint8_t *)vtg_buffer,strnlen((const char*)vtg_buffer,160));
		Serial.write('\n');

		SerialBT.write((uint8_t *)gga_buffer,strnlen((const char*)gga_buffer,160));
		SerialBT.write('\n');
		SerialBT.write((uint8_t *)vtg_buffer,strnlen((const char*)vtg_buffer,160));
		SerialBT.write('\n');
		*/
	}

	if (use_bno08x) {
		bno08x_last_time = millis();
		is_triggered = true;
	}

	//zero out buffer for next message
	nmeaparser.setBuffer((void *)&(nmea_buffer[which_nmea_buffer][0]), sizeof(nmea_buffer[0]));
}

void setup()
{
	Serial.begin(serial_speed);
	delay(5000);
	Serial.println("F9P IMU integration.");

	SerialBT.begin("esp32_f9p");


	//pinMode(13, OUTPUT); //for an LED
	//another LED to indicate IMU presence
	//pinMode(IMULED, OUTPUT);

	Wire.begin();

	uint8_t error;

	Wire.beginTransmission(CMPS14_ADDRESS);

	if(!Wire.endTransmission()) {
		use_cmps = true;
	} else {
		//try the BNO08x
		for (int8_t i=0 ; i < num_BNO08x_addresses ; i++) {
			Wire.beginTransmission(bno08x_address[i]);
			if (! Wire.endTransmission() ) {
				//we found the BNO08x
				if (bno08x.begin(bno08x_address[i])) {
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
				} else {
					Serial.println("Found BNO08x but could not communicate.");
				}
			} else {
				Serial.println("BNO08x not at address.");

			}
		}
	}

	if(use_cmps || use_bno08x) {
		Serial.println("Got BNO08x.");
		//digitalWrite(IMULED, HIGH);
	} else {
		Serial.println("No IMU present.");
		//digitalWrite(IMULED, LOW);
	}

	//now start talking to f9
	SerialGPS.begin(gps_speed, SERIAL_8N1, GPSRX, GPSTX);
}

void loop()
{	
	uint32_t current_time;
	uint32_t avail;

	//Read RTCM data from bluetooth
	//Is this a valid way of reading in a chunk?
	avail = SerialBT.available();
	for (uint32_t i=0; i < avail; i++) {
		SerialGPS.write(SerialBT.read());
	}

	//Read NMEA data from GP
	avail = SerialGPS.available();
	while(SerialGPS.available()) {
	//if (SerialGPS.available()) { //just one byte at a time?
		if(nmeaparser.process(SerialGPS.read())) {
			//we've got a sentence

			if (!strcmp(nmeaparser.getMessageID(), "GGA")) {
				last_gga = millis();
				//Serial.print(last_gga);
				//Serial.print(" ");
				//Serial.print(last_vtg);
				//Serial.println(" Got gga.");

				//save a pointer to the gga buffer
				gga_buffer = nmea_buffer[which_nmea_buffer];
				
				//did we have a vtg message very recently?
				if (last_gga - last_vtg < 40) {
					//Serial.println(last_gga - last_vtg);
					//Serial.println("VTG was recent, so we're good.");
					if (! use_cmps) {
						output_gga(); //this will set is_triggered
					} else {
						gps_ready_time = millis();
						is_triggered = true;
					}
				} else {
					//swap buffers so we can get a more recent vtg
					//select the other buffer
					which_nmea_buffer = (which_nmea_buffer + 1 ) % 2;
					nmeaparser.setBuffer((void *)&(nmea_buffer[which_nmea_buffer][0]), sizeof(nmea_buffer[0]));
				}
				break;

			} else if (!strcmp(nmeaparser.getMessageID(), "VTG")) {
				last_vtg = millis();
				//Serial.print(last_vtg);
				//Serial.print(" ");
				//Serial.print(last_gga);
				//Serial.println(" Got vtg.");

				//save a pointer to the vtg buffer
				vtg_buffer = nmea_buffer[which_nmea_buffer];

				//did we have a gga message very recently?
				if (last_vtg - last_gga < 40) {
					Serial.println(last_vtg - last_gga);
					//Serial.println("GGA was recent, so we're good.");
					if (! use_cmps) {
						output_gga(); //this will set is_triggered
					} else {
						gps_ready_time = millis();
						is_triggered = true;
					}
				} else {
					//select the other buffer
					which_nmea_buffer = (which_nmea_buffer + 1 ) % 2;
					nmeaparser.setBuffer((void *)&(nmea_buffer[which_nmea_buffer][0]), sizeof(nmea_buffer[0]));
				}
				break;
			}
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
	}

}


