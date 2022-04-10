f9p_imu
==========

 This code has not yet been tested on a tractor!  It may not even compile. This code is for the Arduino IDE with ESP32 support enabled.
 
 f9_imu  is a sketch for the ESP32 that combines GPS position (from NMEA GGA and VTG) with information from a BNO08x IMU to calculate a virtual GPS position on the ground when the antenna is on the roof of a tractor.  This is also called "terrain compensation."  The idea is to calculate the actual position on the ground right at the tractor's axle, even though the antenna might be on the roof and moving back and forth as the tires drive over uneven ground.

 The algorithms and techniques are based on Brian Tischler's Panda sketch.  From Panda I borrowed the idea of wating a certain amount of time after a GPS fix before reading the IMU's roll which will be used for the subsequent GPS fix. This allows better synchronization of the roll angle reading and the GPS fix, owing to latency in the GPS receiver as it calculates the RTK fix.

 f9p_imu reads in NMEA data from serial port 1 and parses out the latitude, longitude, altitude, heading, and speed.  As well it monitors the bluetooth serial port for RTCM data which it passes on to serial port 2 connected to a GPS receiver (ZED F9P in my case).

 Unlike Panda, f9p_imu actually calculates the effect the roll has on the GPS position, based on the current heading.  It computes the lateral movement of the fix depending on the tip of the tractor.  If the antenna is offset from the center of the tractor, the virtual GPS position is translated to account for that.  Finally if the antenna is forward of the axle, the virtual GPS posistion is translated back.  As well the altitude is adjusted to reflect the position on the ground.

 The BNO08x is used in combined gyro mode.  There is some drift, and an offset between actual heading and gyro heading must be computed.  This is done by either driving faster than a certain speed, or by driving a certain distance and computing the heading.  Then the difference between this and the gyro reading is set as the gyro offset.  As the tractor drives above a certain speed, the GPS heading is mixed with the gyro heading, filtered, and then the gyro offset is recomputeed.  Below a certain speed, or if the turn rate is above a threshold, the heading is read from the gyro exclusively.  This heading angle is used in the calculation described above, and it is also passed on in the output NMEA.

 So far there is no provision for detecting reverse, so when backing up terrain compensation may not work properly at all, depending on the speed.

 After computing the virtual GPS position, revised GGA and VTG NMEA sentences are sent out serial port 1.


