//
//  GeodeticUTMConverter.m
//  GeodeticUTMConverter
//
//  Created by Cameron Lowell Palmer & Mariia Ruchko on 19.06.12.
//  Copyright (c) 2012 Cameron Lowell Palmer & Mariia Ruchko. All rights reserved.
//
//  Code converted from Javascript as written by Chuck Taylor http://home.hiwaay.net/~taylorc/toolbox/geography/geoutm.html
//  Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J., GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
//

#include <angles/angles.h>
#include <rtk_ros/UTMConverter.h>

UTMDatum UTMConverter::utm_datum_ = UTMDatumMake(6378137, 6356752.3142);
double UTMConverter::utm_scale_factor_ = 0.9996;

void UTMConverter::latitudeAndLongitudeToUTMCoordinates(sensor_msgs::NavSatFix &fix, UTMCoordinates &utm)
{
    unsigned int zone = floor((fix.longitude + 180.0) / 6) + 1;
    UTMHemisphere hemisphere;
    if(fix.latitude < 0)
    {
        hemisphere = kUTMHemisphereSouthern;
    }
    else
    {
        hemisphere = kUTMHemisphereNorthern;
    }
    double c_meridian = UTMCentralMeridian(zone);

    double longitude = angles::from_degrees(fix.longitude);
    double latitude = angles::from_degrees(fix.latitude);
    latitudeAndLongitudeToTMCoordinates(latitude, longitude, utm, c_meridian);

    /* Adjust easting and northing for UTM system. */
    double x = utm.easting * utm_scale_factor_ + 500000.0;
    double y = utm.northing * utm_scale_factor_;
    if(y < 0.0)
    {
        y = y + 10000000.0;
    }

    utm.easting = x;
    utm.northing = y;
    utm.grid_zone = zone;
    utm.hemisphere = hemisphere;
}

void UTMConverter::UTMCoordinatesToLatitudeAndLongitude(UTMCoordinates &utm, sensor_msgs::NavSatFix &fix)
{
    double c_meridian;

    double x = utm.easting;
    double y = utm.northing;
    double zone = utm.grid_zone;
    UTMHemisphere hemisphere = utm.hemisphere;

    x -= 500000.0;
    x /= utm_scale_factor_;

    /* If in southern hemisphere, adjust y accordingly. */
    if(hemisphere == kUTMHemisphereSouthern)
    {
        y -= 10000000.0;
    }
    y /= utm_scale_factor_;

    double easting = x;
    double northing = y;

    c_meridian = UTMCentralMeridian(zone);
    TMCoordinatesToLatitudeAndLongitude(easting, northing, fix, c_meridian);
    fix.latitude = angles::to_degrees(fix.latitude);
    fix.longitude = angles::to_degrees(fix.longitude);
}

// Computes the ellipsoidal distance from the equator to a point at a given latitude in meters
double UTMConverter::arcLengthOfMeridian(double latitude_in_radians)
{
    double alpha;
    double beta;
    double gamma;
    double delta;
    double epsilon;
    double n;

    double result;

    double equitorial_radius = utm_datum_.equitorial_radius;
    double polar_radius = utm_datum_.polar_radius;

    /* Precalculate n */
    n = (equitorial_radius - polar_radius) / (equitorial_radius + polar_radius);

    /* Precalculate alpha */
    alpha = ((equitorial_radius + polar_radius) / 2.0) * (1.0 + (pow(n, 2.0) / 4.0) + (pow(n, 4.0) / 64.0));

    /* Precalculate beta */
    beta = (-3.0 * n / 2.0) + (9.0 * pow(n, 3.0) / 16.0) + (-3.0 * pow(n, 5.0) / 32.0);

    /* Precalculate gamma */
    gamma = (15.0 * pow(n, 2.0) / 16.0) + (-15.0 * pow(n, 4.0) / 32.0);

    /* Precalculate delta */
    delta = (-35.0 * pow(n, 3.0) / 48.0) + (105.0 * pow(n, 5.0) / 256.0);

    /* Precalculate epsilon */
    epsilon = (315.0 * pow(n, 4.0) / 512.0);

    /* Now calculate the sum of the series and return */
    result = alpha * (latitude_in_radians + (beta * sin(2.0 * latitude_in_radians)) + (gamma * sin(4.0 * latitude_in_radians)) + (delta * sin(6.0 * latitude_in_radians)) + (epsilon * sin(8.0 * latitude_in_radians)));

    return result;
}

// Determines the central meridian for the given UTM zone.
double UTMConverter::UTMCentralMeridian(unsigned int zone)
{
    double c_meridian;

    c_meridian = angles::from_degrees(-183.0 + (zone * 6.0));

    return c_meridian;
}

// Computes the footpoint latitude for use in converting transverse Mercator coordinates to ellipsoidal coordinates.
double UTMConverter::footpointLatitude(double northing_in_meters)
{
    double y;
    double alpha;
    double beta;
    double gamma;
    double delta;
    double epsilon;
    double n;

    double result;

    double equitorial_radius = utm_datum_.equitorial_radius;
    double polar_radius = utm_datum_.polar_radius;

    /* Precalculate n (Eq. 10.18) */
    n = (equitorial_radius - polar_radius) / (equitorial_radius + polar_radius);

    /* Precalculate alpha_ (Eq. 10.22) */
    /* (Same as alpha in Eq. 10.17) */
    alpha = ((equitorial_radius + polar_radius) / 2.0) * (1 + (pow(n, 2.0) / 4) + (pow(n, 4.0) / 64));

    /* Precalculate y (Eq. 10.23) */
    y = northing_in_meters / alpha;

    /* Precalculate beta (Eq. 10.22) */
    beta = (3.0 * n / 2.0) + (-27.0 * pow(n, 3.0) / 32.0) + (269.0 * pow(n, 5.0) / 512.0);

    /* Precalculate gamma (Eq. 10.22) */
    gamma = (21.0 * pow(n, 2.0) / 16.0) + (-55.0 * pow(n, 4.0) / 32.0);

    /* Precalculate delta (Eq. 10.22) */
    delta = (151.0 * pow(n, 3.0) / 96.0) + (-417.0 * pow(n, 5.0) / 128.0);

    /* Precalculate epsilon (Eq. 10.22) */
    epsilon = (1097.0 * pow(n, 4.0) / 512.0);

    /* Now calculate the sum of the series (Eq. 10.21) */
    result = y + (beta * sin(2.0 * y)) + (gamma * sin(4.0 * y)) + (delta * sin(6.0 * y)) + (epsilon * sin(8.0 * y));

    return result;
}

// Converts a latitude/longitude pair to x and y coordinates in the Transverse Mercator projection.  Note that Transverse Mercator is not the same as UTM; a scale factor is required to convert between them.
void UTMConverter::latitudeAndLongitudeToTMCoordinates(double latitude, double longitude, UTMCoordinates &tm, double lambda0)
{
    double N, nu2, ep2, t, t2, l;
    double l3coef, l4coef, l5coef, l6coef, l7coef, l8coef;
    double tmp;

    double phi = latitude; // Latitude in radians
    double lambda = longitude; // Longitude in radians

    double equitorial_radius = utm_datum_.equitorial_radius;
    double polar_radius = utm_datum_.polar_radius;

    /* Precalculate ep2 */
    ep2 = (pow(equitorial_radius, 2.0) - pow(polar_radius, 2.0)) / pow(polar_radius, 2.0);

    /* Precalculate nu2 */
    nu2 = ep2 * pow(cos(phi), 2.0);

    /* Precalculate N */
    N = pow(equitorial_radius, 2.0) / (polar_radius * sqrt(1 + nu2));

    /* Precalculate t */
    t = tan(phi);
    t2 = t * t;
    tmp = (t2 * t2 * t2) - pow(t, 6.0);

    /* Precalculate l */
    l = lambda - lambda0;

    /* Precalculate coefficients for l**n in the equations below
     so a normal human being can read the expressions for easting
     and northing
     -- l**1 and l**2 have coefficients of 1.0 */
    l3coef = 1.0 - t2 + nu2;
    l4coef = 5.0 - t2 + 9 * nu2 + 4.0 * (nu2 * nu2);
    l5coef = 5.0 - 18.0 * t2 + (t2 * t2) + 14.0 * nu2 - 58.0 * t2 * nu2;
    l6coef = 61.0 - 58.0 * t2 + (t2 * t2) + 270.0 * nu2 - 330.0 * t2 * nu2;
    l7coef = 61.0 - 479.0 * t2 + 179.0 * (t2 * t2) - (t2 * t2 * t2);
    l8coef = 1385.0 - 3111.0 * t2 + 543.0 * (t2 * t2) - (t2 * t2 * t2);

    /* Calculate easting (x) */
    tm.easting = N * cos(phi) * l + (N / 6.0 * pow(cos(phi), 3.0) * l3coef * pow(l, 3.0)) + (N / 120.0 * pow(cos(phi), 5.0) * l5coef * pow(l, 5.0)) + (N / 5040.0 * pow(cos(phi), 7.0) * l7coef * pow(l, 7.0));

    /* Calculate northing (y) */
    tm.northing = arcLengthOfMeridian(phi) + (t / 2.0 * N * pow(cos(phi), 2.0) * pow(l, 2.0)) + (t / 24.0 * N * pow(cos(phi), 4.0) * l4coef * pow(l, 4.0)) + (t / 720.0 * N * pow(cos(phi), 6.0) * l6coef * pow(l, 6.0)) + (t / 40320.0 * N * pow(cos(phi), 8.0) * l8coef * pow(l, 8.0));
}

// Converts x and y coordinates in the Transverse Mercator projection to a latitude/longitude pair.  Note that Transverse Mercator is not the same as UTM; a scale factor is required to convert between them.
// Remarks:
// The local variables Nf, nuf2, tf, and tf2 serve the same purpose as N, nu2, t, and t2 in MapLatLonToXY, but they are computed with respect to the footpoint latitude phif.
// x1frac, x2frac, x2poly, x3poly, etc. are to enhance readability and to optimize computations.
void UTMConverter::TMCoordinatesToLatitudeAndLongitude(double easting, double northing, sensor_msgs::NavSatFix &fix, double lambda0)
{
    double x = easting;
    double y = northing;

    double equitorial_radius = utm_datum_.equitorial_radius;
    double polar_radius = utm_datum_.polar_radius;

    double phif, Nf, Nfpow, nuf2, ep2, tf, tf2, tf4, cf;
    double x1frac, x2frac, x3frac, x4frac, x5frac, x6frac, x7frac, x8frac;
    double x2poly, x3poly, x4poly, x5poly, x6poly, x7poly, x8poly;

    /* Get the value of phif, the footpoint latitude. */
    phif = footpointLatitude(y);

    /* Precalculate ep2 */
    ep2 = (pow(equitorial_radius, 2.0) - pow(polar_radius, 2.0)) / pow(polar_radius, 2.0);

    /* Precalculate cos (phif) */
    cf = cos(phif);

    /* Precalculate nuf2 */
    nuf2 = ep2 * pow(cf, 2.0);

    /* Precalculate Nf and initialize Nfpow */
    Nf = pow(equitorial_radius, 2.0) / (polar_radius * sqrt(1 + nuf2));
    Nfpow = Nf;

    /* Precalculate tf */
    tf = tan(phif);
    tf2 = tf * tf;
    tf4 = tf2 * tf2;

    /* Precalculate fractional coefficients for x**n in the equations
     below to simplify the expressions for latitude and longitude. */
    x1frac = 1.0 / (Nfpow * cf);

    Nfpow *= Nf;   /* now equals Nf**2) */
    x2frac = tf / (2.0 * Nfpow);

    Nfpow *= Nf;   /* now equals Nf**3) */
    x3frac = 1.0 / (6.0 * Nfpow * cf);

    Nfpow *= Nf;   /* now equals Nf**4) */
    x4frac = tf / (24.0 * Nfpow);

    Nfpow *= Nf;   /* now equals Nf**5) */
    x5frac = 1.0 / (120.0 * Nfpow * cf);

    Nfpow *= Nf;   /* now equals Nf**6) */
    x6frac = tf / (720.0 * Nfpow);

    Nfpow *= Nf;   /* now equals Nf**7) */
    x7frac = 1.0 / (5040.0 * Nfpow * cf);

    Nfpow *= Nf;   /* now equals Nf**8) */
    x8frac = tf / (40320.0 * Nfpow);

    /* Precalculate polynomial coefficients for x**n.
     -- x**1 does not have a polynomial coefficient. */
    x2poly = -1.0 - nuf2;
    x3poly = -1.0 - 2 * tf2 - nuf2;
    x4poly = 5.0 + 3.0 * tf2 + 6.0 * nuf2 - 6.0 * tf2 * nuf2 - 3.0 * (nuf2 *nuf2) - 9.0 * tf2 * (nuf2 * nuf2);
    x5poly = 5.0 + 28.0 * tf2 + 24.0 * tf4 + 6.0 * nuf2 + 8.0 * tf2 * nuf2;
    x6poly = -61.0 - 90.0 * tf2 - 45.0 * tf4 - 107.0 * nuf2 + 162.0 * tf2 * nuf2;
    x7poly = -61.0 - 662.0 * tf2 - 1320.0 * tf4 - 720.0 * (tf4 * tf2);
    x8poly = 1385.0 + 3633.0 * tf2 + 4095.0 * tf4 + 1575 * (tf4 * tf2);

    /* Calculate latitude */
    fix.latitude = phif + x2frac * x2poly * (x * x) + x4frac * x4poly * pow(x, 4.0) + x6frac * x6poly * pow(x, 6.0) + x8frac * x8poly * pow(x, 8.0);

    /* Calculate longitude */
    fix.longitude = lambda0 + x1frac * x + x3frac * x3poly * pow(x, 3.0) + x5frac * x5poly * pow(x, 5.0) + x7frac * x7poly * pow(x, 7.0);
}

void UTMConverter::setDatum(double equitorial_radius, double polar_radius)
{
    utm_datum_ = UTMDatumMake(equitorial_radius, polar_radius);
}
