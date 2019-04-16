#pragma once

// Third-party includes
#include "third_party/units.h"

//----------------------------------------------------------------------------
// BoBRobotics::MapCoordinate::Transform
//----------------------------------------------------------------------------
namespace BoBRobotics
{
namespace MapCoordinate
{
//! Transform from one datum to another
struct Transform
{
    // Translation
    const units::length::meter_t tx;
    const units::length::meter_t ty;
    const units::length::meter_t tz;

    // Scale
    const double s;

    // Rotation
    const units::angle::radian_t rx;
    const units::angle::radian_t ry;
    const units::angle::radian_t rz;
};

//----------------------------------------------------------------------------
// BoBRobotics::MapCoordinate::Ellipsoid
//----------------------------------------------------------------------------
//! Elipsoid associated with a particular datum
struct Ellipsoid
{
    // Major and minor axis
    const units::length::meter_t a;
    const units::length::meter_t b;
};

//----------------------------------------------------------------------------
// BoBRobotics::MapCoordinate::WGS84
//----------------------------------------------------------------------------
//! WGS84 datum - used for GPS data
struct WGS84
{
    static const Ellipsoid ellipsoid;
};
const Ellipsoid WGS84::ellipsoid{units::length::meter_t(6378137), units::length::meter_t(6356752.314245)};

//----------------------------------------------------------------------------
// BoBRobotics::MapCoordinate::OSGB36
//----------------------------------------------------------------------------
//! OSGB-36 datum - used for UK OS map coordinates
struct OSGB36
{
    static const Transform transformFromWGS84;
    static const Ellipsoid ellipsoid;
};
const Transform OSGB36::transformFromWGS84{units::length::meter_t(-446.448), units::length::meter_t(125.157), units::length::meter_t(-542.060),
                                           20.4894, units::angle::arcsecond_t(-0.1502), units::angle::arcsecond_t(-0.2470), units::angle::arcsecond_t(-0.8421)};
const Ellipsoid OSGB36::ellipsoid{units::length::meter_t(6377563.396), units::length::meter_t(6356256.909)};

//----------------------------------------------------------------------------
// BoBRobotics::MapCoordinate::OSCoordinate
//----------------------------------------------------------------------------
//! An OS coordinate consisting of an 'easting' and 'northing' distance
struct OSCoordinate
{
    const units::length::meter_t easting;
    const units::length::meter_t northing;
};

//----------------------------------------------------------------------------
// BoBRobotics::MapCoordinate::Ellipsoid
//----------------------------------------------------------------------------
//! A latitude and longitude relative to a particular datum
template<typename D>
struct LatLon
{
    typedef D Datum;

    const units::angle::degree_t lat;
    const units::angle::degree_t lon;
};

//----------------------------------------------------------------------------
// BoBRobotics::MapCoordinate::Cartesian
//----------------------------------------------------------------------------
//! A cartesian coordinate relative to a particular datum
template<typename D>
struct Cartesian
{
    typedef D Datum;

    const units::length::meter_t x;
    const units::length::meter_t y;
    const units::length::meter_t z;
};


//! Convert latitude and longitude to cartesian
/*! Ported from Javascript Ordnance Survey Grid Reference functions (c) Chris Veness 2005-2017 (MIT Licence) */
template<typename Datum>
Cartesian<Datum> latLonToCartesian(const LatLon<Datum> &latLon)
{
    using namespace units::math;
    using namespace units::length;
    using namespace units::literals;

    const meter_t h = 0.0_m; // height above ellipsoid - not currently used
    const double sinPhi = sin(latLon.lat);
    const double cosPhi = cos(latLon.lat);
    const double sinLambda = sin(latLon.lon);
    const double cosLambda = cos(latLon.lon);

    const double eSq = 1.0 - (Datum::ellipsoid.b * Datum::ellipsoid.b) / (Datum::ellipsoid.a * Datum::ellipsoid.a); // 1st eccentricity squared
    const meter_t nu = Datum::ellipsoid.a / std::sqrt(1.0 - eSq * sinPhi * sinPhi);                                 // radius of curvature in prime vertical

    return Cartesian<Datum>{(nu + h) * cosPhi * cosLambda,
                            (nu + h) * cosPhi * sinLambda,
                            (nu * (1.0 - eSq) + h) * sinPhi};
}

//! Transform cartesian coordinates from WGS84 into desired space
/*! Ported from Javascript Ordnance Survey Grid Reference functions (c) Chris Veness 2005-2017 (MIT Licence) */
template<typename Datum>
Cartesian<Datum> transformCartesian(const Cartesian<WGS84> &c)
{
    const double s1 = Datum::transformFromWGS84.s / 1E6 + 1.0;            // scale: normalise parts-per-million to (s+1)

    const double rx = Datum::transformFromWGS84.rx.value();
    const double ry = Datum::transformFromWGS84.ry.value();
    const double rz = Datum::transformFromWGS84.rz.value();

    // apply transform
    return Cartesian<Datum>{Datum::transformFromWGS84.tx + c.x * s1 - c.y * rz + c.z * ry,
                            Datum::transformFromWGS84.ty + c.x * rz + c.y * s1 - c.z * rx,
                            Datum::transformFromWGS84.tz - c.x * ry + c.y * rx + c.z * s1};
}

//! Convert cartesian coordinates back to latitude and longitude
/*! Ported from Javascript Ordnance Survey Grid Reference functions (c) Chris Veness 2005-2017 (MIT Licence) */
template<typename Datum>
LatLon<Datum> cartesianToLatLon(const Cartesian<Datum> &c)
{
    using namespace units::angle;
    using namespace units::length;
    using namespace units::math;
    using namespace units::literals;

    const double eSq = 1.0 - (Datum::ellipsoid.b * Datum::ellipsoid.b) / (Datum::ellipsoid.a * Datum::ellipsoid.a); // 1st eccentricity squared
    const double epsilonSq = eSq / (1.0 - eSq);                                                                     // 2nd eccentricity squared
    const meter_t p = sqrt(c.x * c.x + c.y * c.y);                                                                  // distance from minor axis
    const meter_t r = sqrt(p * p + c.z * c.z);                                                                      // polar radius

    // parametric latitude (Bowring eqn 17, replacing tanβ = z·a / p·b)
    const double tanBeta = (Datum::ellipsoid.b * c.z) / (Datum::ellipsoid.a * p) * (1.0 + epsilonSq * Datum::ellipsoid.b / r);
    const double sinBeta = tanBeta / std::sqrt(1.0 + tanBeta * tanBeta);
    const double cosBeta = sinBeta / tanBeta;

    // geodetic latitude (Bowring eqn 18: tanφ = z+ε²bsin³β / p−e²cos³β)
    const degree_t phi = std::isnan(cosBeta) ? 0.0_rad : atan2(c.z + epsilonSq * Datum::ellipsoid.b * sinBeta * sinBeta * sinBeta,
                                                               p - eSq * Datum::ellipsoid.a * cosBeta * cosBeta * cosBeta);

    // longitude
    const degree_t lambda = atan2(c.y, c.x);

    // height above ellipsoid (Bowring eqn 7) [not currently used]
    /*const double sinPhi = sin(phi), cosPhi = cos(phi);
    const meter_t nu = std::get<0>(ellipsoid) / std::sqrt(1.0 - eSq * sinPhi * sinPhi); // length of the normal terminated by the minor axis
    var h = p*cosφ + cartesian[2]*sinφ - (osgb36EllipseA*osgb36EllipseA/ν);*/

    return LatLon<Datum>{phi, lambda};
}

//! Convert latitude and longitude in the OSGB36 space to OS grid coordinates
/*! Ported from Javascript Ordnance Survey Grid Reference functions (c) Chris Veness 2005-2017 (MIT Licence) */
OSCoordinate latLonToOS(const LatLon<OSGB36> &latLon)
{
    using namespace units::angle;
    using namespace units::length;
    using namespace units::math;
    using namespace units::literals;

    const meter_t a = 6377563.396_m, b = 6356256.909_m;                 // Airy 1830 major & minor semi-axes
    constexpr double f0 = 0.9996012717;                                 // NatGrid scale factor on central meridian;
    const radian_t phi0 = 49.0_deg, lambda0 = -2.0_deg;                 // NatGrid true origin is 49°N,2°W
    const meter_t n0 = -100000.0_m, e0 = 400000.0_m;                    // northing & easting of true origin, metres
    const double eSq = 1.0 - (b * b) / (a * a);                         // eccentricity squared
    const double n = (a - b) / (a + b), n2 = n * n, n3 = n * n * n;     // n, n², n³

    const radian_t phi = latLon.lat;
    const radian_t lambda = latLon.lon;

    const double sinPhi = sin(phi);
    const double cosPhi = cos(phi);
    const double tanPhi = tan(phi);
    const double cos3Phi = cosPhi * cosPhi * cosPhi;
    const double cos5Phi = cos3Phi * cosPhi * cosPhi;
    const double tan2Phi = tanPhi * tanPhi;
    const double tan4Phi = tan2Phi * tan2Phi;


    const meter_t nu = a * f0 / std::sqrt(1.0 - eSq * sinPhi * sinPhi);                     // nu = transverse radius of curvature
    const meter_t rho = a * f0 * (1.0 - eSq) / std::pow(1.0 - eSq * sinPhi * sinPhi, 1.5);  // rho = meridional radius of curvature
    const double eta2 = nu / rho - 1.0;                                                     // eta = ?

    const double mA = (1.0 + n + (5.0 / 4.0) * n2 + (5.0 / 4.0) * n3) * (phi - phi0).value();
    const double mB = (3.0 * n + 3.0 * n * n + (21.0 / 8.0) * n3) * sin(phi - phi0) * cos(phi + phi0);
    const double mC = ((15.0 / 8.0) * n2 + (15.0 / 8.0) * n3) * sin(2.0 * (phi - phi0)) * cos(2.0 * (phi + phi0));
    const double mD = (35.0 / 24.0) * n3 * sin(3.0 * (phi - phi0)) * cos(3.0 * (phi + phi0));
    const meter_t m = b * f0 * (mA - mB + mC - mD);                                                                  // meridional arc

    const meter_t i = m + n0;
    const meter_t ii = (nu / 2.0) * sinPhi * cosPhi;
    const meter_t iii = (nu / 24.0) * sinPhi * cos3Phi * (5.0 - tan2Phi + 9.0 * eta2);
    const meter_t iiia = (nu / 720.0) * sinPhi * cos5Phi * (61.0 - 58.0 * tan2Phi + tan4Phi);
    const meter_t iv = nu * cosPhi;
    const meter_t v = (nu / 6.0) * cos3Phi * (nu / rho - tan2Phi);
    const meter_t vi = (nu / 120.0) * cos5Phi * (5.0 - 18.0 * tan2Phi + tan4Phi+ 14.0 * eta2 - 58.0 * tan2Phi * eta2);


    const double deltaLambda = (lambda - lambda0).value();
    const double deltaLambda2 = deltaLambda * deltaLambda;
    const double deltaLambda3 = deltaLambda2 * deltaLambda;
    const double deltaLambda4 = deltaLambda3 * deltaLambda;
    const double deltaLambda5 = deltaLambda4 * deltaLambda;
    const double deltaLambda6 = deltaLambda5 * deltaLambda;

    return OSCoordinate{e0 + (iv * deltaLambda) + (v * deltaLambda3) + (vi * deltaLambda5),
                        i + (ii * deltaLambda2) + (iii * deltaLambda4) + (iiia * deltaLambda6)};
}
}   // namespace MapCoordinate
}   // namespace BoBRobotics