#ifndef GPS_TO_BODY_HPP
#define GPS_TO_BODY_HPP

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <geodesy/utm.h>
#include <cmath>

using Point = geometry_msgs::msg::Point;
using NavSatFix = sensor_msgs::msg::NavSatFix;
using GeoPoint = geographic_msgs::msg::GeoPoint;
using UTMPoint = geodesy::UTMPoint;

// Convert GPS to FLU coordinates
inline Point gps_to_utm(
    const NavSatFix &current_gps,
    const NavSatFix &origin_gps)
{
    GeoPoint current_geopoint;
    GeoPoint origin_geopoint;

    current_geopoint.latitude = current_gps.latitude;
    current_geopoint.longitude = current_gps.longitude;
    current_geopoint.altitude = current_gps.altitude;

    origin_geopoint.latitude = origin_gps.latitude;
    origin_geopoint.longitude = origin_gps.longitude;
    origin_geopoint.altitude = origin_gps.altitude;

    UTMPoint current_utm(current_geopoint);
    UTMPoint origin_utm(origin_geopoint);

    Point utm;
    utm.x = current_utm.easting - origin_utm.easting;
    utm.y = current_utm.northing - origin_utm.northing;
    utm.z = current_geopoint.altitude - origin_geopoint.altitude;

    return utm;
};

#endif // GPS_TO_BODY_HPP