#include "haversine.h"
#include <math.h>
#include <Arduino.h>

#define RADIANS(deg) deg * M_PI / 180.0
#define DEGREES(rad) rad * 180.0 / M_PI


namespace haversine {
	/*-----------------------------Distance & Bearing Calculator---------------------------------*/
	double distance (double lat1, double lon1, double lat2, double lon2 )
	{
		//This is a haversine based distance calculation formula

		//This portion converts the current and destination GPS coords from decDegrees to Radians
		double lonR1 = lon1*toRadians;
		double lonR2 = lon2*toRadians;
		double latR1 = lat1*toRadians;
		double latR2 = lat2*toRadians;

		//This portion calculates the differences for the Radian latitudes and longitudes and saves them to variables
		double dlon = lonR2 - lonR1;
		double dlat = latR2 - latR1;

		//This portion is the Haversine Formula for distance between two points. Returned value is in metres
		double a = (sq(sin(dlat * 0.5))) + cos(latR1) * cos(latR2) * (sq(sin(dlon * 0.5)));
		double e = 2 * atan2(sqrt(a), sqrt(1-a)) ;

		return EARTH_RADIUS * e;
	}

	double bearing (double lat1, double lon1, double lat2, double lon2 )
	{
		double latR1 = lat1*toRadians;
		double latR2 = lat2*toRadians;
		double lonR1 = lon1*toRadians;
		double lonR2 = lon2*toRadians;

		//This portion is the Haversine Formula for required bearing between current 
		//location and destination. Returned value is in radians
		double y = cos(latR2)*sin(lonR2-lonR1); //calculate x
		double x = cos(latR1)*sin(latR2)-sin(latR1)*cos(latR2)*cos(lonR2-lonR1); //calculate y

		return atan2(y, x); //return atan2 result for bearing. Result at this point is in Radians

	}
	double bearing_degrees (double lat1, double lon1, double lat2, double lon2 )
	{
		return bearing(lon1, lat1, lon2, lat2) *  toDegrees;
	}

	void move_distance_bearing( double &lat, double &lon,
                                    double heading, double distance)
	{
		double offset = distance / EARTH_RADIUS;	
		double latr = RADIANS(lat);
		double lonr = RADIANS(lon);

		double lat1sin = sin(latr);	

		double lat1cos = cos(latr);
		double distcos = cos(offset);
		double distsin = sin(offset);

		heading = RADIANS(heading);

		lat = asin( lat1sin * distcos +
			    lat1cos * distsin * cos(heading) );
		lon = lonr + atan2( sin(heading) * distsin * lat1cos,
				   distcos - lat1sin * sin(lat) );

		lat = DEGREES(lat);
		lon = DEGREES(lon);
	}

}

