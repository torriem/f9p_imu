#ifndef __HAVERSINE_H__
#define __HAVERSINE_H__


namespace haversine
{
	//const double EARTH_RADIUS = 6371000.00; //radis of earth in metres
	const double EARTH_RADIUS = 6378137.00; //radis of earth in metres
	const double toDegrees = 57.295779; //180/PI
	const double toRadians = 3.1415926535897932384626433832795 / 180.0; 

	double distance ( double from_lat1, double from_lon1, double to_lat2, double to_lon2 );
	double bearing ( double from_lat1, double from_lon1, double to_lat2, double to_lon2 );
	double bearing_degrees ( double from_lat1, double from_lon1, double to_lat2, double to_lon2 );
	void move_distance_bearing( double &lat, double &lon,
                                    double heading, double distance) ;
}

#endif
