
#include <sensor_msgs/NavSatFix.h>

#ifndef UTMCONVERTER_H
#define UTMCONVERTER_H

typedef enum
{
    kUTMHemisphereNorthern,
    kUTMHemisphereSouthern

} UTMHemisphere;

typedef struct
{
    double northing;
    double easting;
    unsigned int grid_zone;
    UTMHemisphere hemisphere;

} UTMCoordinates;

typedef struct
{
    double equitorial_radius;
    double polar_radius;

} UTMDatum;


class UTMConverter
{
public:
    static void latitudeAndLongitudeToUTMCoordinates(sensor_msgs::NavSatFix &fix, UTMCoordinates &utm);
    static void UTMCoordinatesToLatitudeAndLongitude(UTMCoordinates &utm, sensor_msgs::NavSatFix &fix);

    static void setDatum(double equitorial_radius, double polar_radius);

private:
    UTMConverter()
    {

    };

    static double arcLengthOfMeridian(double latitude_in_radians);
    static double UTMCentralMeridian(unsigned int zone);
    static double footpointLatitude(double northing_in_meters);
    static void latitudeAndLongitudeToTMCoordinates(double latitude, double longitude, UTMCoordinates &tm, double lambda0);
    static void TMCoordinatesToLatitudeAndLongitude(double easting, double northing, sensor_msgs::NavSatFix &fix, double lambda0);

    static UTMDatum utm_datum_;
    static double utm_scale_factor_;
};


static inline UTMCoordinates UTMCoordinatesMake(double northing, double easting, unsigned int grid_zone, UTMHemisphere hemisphere)
{
    UTMCoordinates coordinates;
    coordinates.northing = northing;
    coordinates.easting = easting;
    coordinates.grid_zone = grid_zone;
    coordinates.hemisphere = hemisphere;

    return coordinates;
}

static inline UTMDatum UTMDatumMake(double equitorial_radius, double polar_radius)
{
    UTMDatum datum;
    datum.equitorial_radius = equitorial_radius;
    datum.polar_radius = polar_radius;

    return datum;
}

#endif
