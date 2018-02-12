#ifndef ROUTING_KIT_GEO_DIST_H
#define ROUTING_KIT_GEO_DIST_H

#include <math.h>

namespace RoutingKit{

//! Returns the distance in meters. Latitudes must be in range [-90,+90] and longitudes in range [-180,+180]
inline double geo_dist(double lat_a, double lon_a, double lat_b, double lon_b){

// The formula used in this function was derived as following:
//
// lat_a = input
// lon_a = input
// lat_b = input
// lon_b = input
//
// ----- Step 1: Convert from degree to radian
//
// lat_a /= 180
// lat_a *= pi
// lon_a /= 180
// lon_a *= pi
// lat_b /= 180
// lat_b *= pi
// lon_b /= 180
// lon_b *= pi
//
// ----- Step 2: Convert to 3D unit vectors
//
// x_a = cos(lon_a) * cos(lat_a)
// y_a = sin(lon_a) * cos(lat_a)
// z_a = sin(lat_a)
//
// x_b = cos(lon_b) * cos(lat_b)
// y_b = sin(lon_b) * cos(lat_b)
// z_b = sin(lat_b)
//
// ----- Step 3: Compute scalar product
//
// c = x_a * x_b + y_a * y_b + z_a * z_b
//
// ----- Step 4: acos(c) is length in radian, scaling up by earth radius gives distance
//
// length = earth_radius*acos(c)
// Output length
//
// ----- Steps 2 to 4 merged in one formula is
//
// length = R*acos(
//	cos(lon_a) * cos(lat_a) * cos(lon_b) * cos(lat_b) +
//	sin(lon_a) * cos(lat_a) * sin(lon_b) * cos(lat_b) +
//	sin(lat_a) * sin(lat_b)
// )
//
// ----- It can be simplified as follows using standard trigonometric identities and basic math operations
// See https://en.wikipedia.org/wiki/List_of_trigonometric_identities
//
// = R*acos(
//	cos(lat_a) * cos(lat_b) * (
//		cos(lon_a) * cos(lon_b) +
//		sin(lon_a) * sin(lon_b)
//	)+
//	sin(lat_a) * sin(lat_b)
// ) = R*acos(
//	cos(lat_a) * cos(lat_b) * (
//		1/2 * (cos(lon_a - lon_b) + cos(lon_a + lon_b)) +
//		1/2 * (cos(lon_a - lon_b) - cos(lon_a + lon_b))
//	)+
//	sin(lat_a) * sin(lat_b)
// ) = R*acos(
//	1/2 * cos(lat_a) * cos(lat_b) * (
//		cos(lon_a - lon_b) + cos(lon_a + lon_b) +
//		cos(lon_a - lon_b) - cos(lon_a + lon_b)
//	)+
//	sin(lat_a) * sin(lat_b)
// ) = R*acos(
//	1/2 * cos(lat_a) * cos(lat_b) * (
//		cos(lon_a - lon_b) +
//		cos(lon_a - lon_b)
//	)+
//	sin(lat_a) * sin(lat_b)
// ) = R*acos(
//	cos(lat_a) * cos(lat_b) * cos(lon_a - lon_b) +
//	sin(lat_a) * sin(lat_b)
// ) = R*acos(
//	1/2 * (cos(lat_a - lat_b) + cos(lat_a + lat_b)) * cos(lon_a - lon_b) +
//	1/2 * (cos(lat_a - lat_b) - cos(lat_a + lat_b))
// ) = R*acos(
//	1/2 * (
//		(cos(lat_a - lat_b) + cos(lat_a + lat_b)) * cos(lon_a - lon_b) +
//		(cos(lat_a - lat_b) - cos(lat_a + lat_b))
//	)
// )

	const double pi_div_180 = 3.14159265359/180.0;
	const double earth_radius = 6371000.785; // in meter

	// To help the auto-vectorizer figure out that using SIMD is a good idea here, we use a local array with a constant size instead of variables.
	// GCC 5's auto vectorizer requires that the array has a power of two size.
	// We thus add a fourth dummy value.
	//
	// If the code was vectorized then the compiler would have to insert this dummy value implicitly. We are helping it by doing it explicitly.
	// If SIMD is disabled, the array should be converted by the optimizer to 4 variables. The dummy variable should subsequently be eliminated using dead code elimination.
	double vec[4] = {
		lat_a - lat_b,
		lat_a + lat_b,
		lon_a - lon_b,
		0
	};

	// It is important that this loop has four iterations, as otherwise the auto-vectorizer does not do its job.
	for(unsigned i=0; i<4; ++i)
		vec[i] = cos(vec[i]*pi_div_180);

	double len = (vec[0] + vec[1]) * vec[2] + vec[0] - vec[1];
	len *= 0.5;
	len  = acos(len);
	len *= earth_radius;

	// len is in meter

	return len;
}

/* Inspired by:
 * http://www.sunshine2k.de/coding/java/PointOnLine/PointOnLine.html
 */

inline void geo_dist_projected_to_arc(double a_lat, double a_lon, double b_lat, double b_lon, double p_lat, double p_lon, double& a_dist, double& b_dist, double& pp_dist, double& pp_lat, double& pp_lon) {
	/* Get the dot product of e1, e2 */
	double e1_lat = b_lat - a_lat;
	double e1_lon = b_lon - a_lon;

	double e2_lat = p_lat - a_lat;
	double e2_lon = p_lon - a_lon;

	double recArea = (e1_lat * e1_lat) + (e1_lon * e1_lon);
	double valDp = (e1_lat * e2_lat) + (e1_lon * e2_lon);

	if (valDp > 0 && valDp < recArea) {
		/* Get the position of the projected point */
		pp_lat = a_lat + (valDp * e1_lat) / recArea;
		pp_lon = a_lon + (valDp * e1_lon) / recArea;

		pp_dist = geo_dist(p_lat, p_lon, pp_lat, pp_lon);
		a_dist = geo_dist(pp_lat, pp_lon, a_lat, a_lon) + pp_dist;
		b_dist = geo_dist(pp_lat, pp_lon, b_lat, b_lon) + pp_dist;

	} else {
		a_dist = geo_dist(p_lat, p_lon, a_lat, a_lon);
		b_dist = geo_dist(p_lat, p_lon, b_lat, b_lon);
		double e_dist = geo_dist(a_lat, a_lon, b_lat, b_lon);

		if (a_dist < b_dist) {
			pp_lat  = a_lat;
			pp_lon  = a_lon;
			pp_dist = a_dist;
			b_dist  = a_dist + e_dist;
		} else {
			pp_lat  = b_lat;
			pp_lon  = b_lon;
			pp_dist = b_dist;
			a_dist  = b_dist + e_dist;
		}
	}
}

} // RoutingKit

#endif
