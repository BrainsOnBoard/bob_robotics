/* -*- mode: C++ -*-
 *
 *  Conversions between Latitude/Longitude and UTM
 *              (Universal Transverse Mercator) coordinates.
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef _UTM_H
#define _UTM_H

/**  @file

 @brief Universal Transverse Mercator transforms.

 Functions to convert (spherical) latitude and longitude to and
 from (Euclidean) UTM coordinates.

 @author Chuck Gantz- chuck.gantz@globalstar.com
 */

#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include "units.h"

using units::length::meter_t;
using units::angle::degree_t;
using units::angle::radian_t;



namespace UTM
{
    // Grid granularity for rounding UTM coordinates to generate MapXY.
    const radian_t grid_size = radian_t(100000.0);    ///< 100 km grid

// WGS84 Parameters
#define WGS84_A		meter_t(6378137.0)		///< major axis
#define WGS84_B		meter_t(6356752.31424518)	///< minor axis
#define WGS84_F		0.0033528107		///< ellipsoid flattening
#define WGS84_E		0.0818191908		///< first eccentricity
#define WGS84_EP	0.0820944379		///< second eccentricity

    // UTM Parameters
#define UTM_K0		0.9996			///< scale factor
#define UTM_FE		meter_t(500000.0)		///< false easting
#define UTM_FN_N	meter_t(0.0)           ///< false northing, northern hemisphere
#define UTM_FN_S	meter_t(10000000.0)    ///< false northing, southern hemisphere
#define UTM_E2		WGS84_E*WGS84_E	///< e^2
#define UTM_E4		UTM_E2*UTM_E2		///< e^4
#define UTM_E6		UTM_E4*UTM_E2		///< e^6
#define UTM_EP2		UTM_E2/(1-UTM_E2)	///< e'^2

#define RAD_TO_DEG      180/M_PI
#define DEG_TO_RAD      M_PI/180

    /**
     * Determine the correct UTM letter designator for the
     * given latitude
     *
     * @returns 'Z' if latitude is outside the UTM limits of 84N to 80S
     *
     * Written by Chuck Gantz- chuck.gantz@globalstar.com
     */
    static inline char UTMLetterDesignator(degree_t Lat)
    {
        char LetterDesignator;
        
        if     ((degree_t(84) >= Lat) && (Lat >= degree_t(72)))  LetterDesignator = 'X';
        else if ((degree_t(72) > Lat) && (Lat >= degree_t(64)))  LetterDesignator = 'W';
        else if ((degree_t(64) > Lat) && (Lat >= degree_t(56)))  LetterDesignator = 'V';
        else if ((degree_t(56) > Lat) && (Lat >= degree_t(48)))  LetterDesignator = 'U';
        else if ((degree_t(48) > Lat) && (Lat >= degree_t(40)))  LetterDesignator = 'T';
        else if ((degree_t(40) > Lat) && (Lat >= degree_t(32)))  LetterDesignator = 'S';
        else if ((degree_t(32) > Lat) && (Lat >= degree_t(24)))  LetterDesignator = 'R';
        else if ((degree_t(24) > Lat) && (Lat >= degree_t(16)))  LetterDesignator = 'Q';
        else if ((degree_t(16) > Lat) && (Lat >= degree_t(8)))   LetterDesignator = 'P';
        else if (( degree_t(8) > Lat) && (Lat >= degree_t(0)))   LetterDesignator = 'N';
        else if (( degree_t(0) > Lat) && (Lat >= degree_t(-8)))  LetterDesignator = 'M';
        else if ((degree_t(-8) > Lat) && (Lat >= degree_t(-16))) LetterDesignator = 'L';
        else if((degree_t(-16) > Lat) && (Lat >= degree_t(-24))) LetterDesignator = 'K';
        else if((degree_t(-24) > Lat) && (Lat >= degree_t(-32))) LetterDesignator = 'J';
        else if((degree_t(-32) > Lat) && (Lat >= degree_t(-40))) LetterDesignator = 'H';
        else if((degree_t(-40) > Lat) && (Lat >= degree_t(-48))) LetterDesignator = 'G';
        else if((degree_t(-48) > Lat) && (Lat >= degree_t(-56))) LetterDesignator = 'F';
        else if((degree_t(-56) > Lat) && (Lat >= degree_t(-64))) LetterDesignator = 'E';
        else if((degree_t(-64) > Lat) && (Lat >= degree_t(-72))) LetterDesignator = 'D';
        else if((degree_t(-72) > Lat) && (Lat >= degree_t(-80))) LetterDesignator = 'C';
        // 'Z' is an error flag, the Latitude is outside the UTM limits
        else LetterDesignator = 'Z';
        return LetterDesignator;
    }

    

    /**
     * Convert lat/long to UTM coords.  Equations from USGS Bulletin 1532
     *
     * East Longitudes are positive, West longitudes are negative.
     * North latitudes are positive, South latitudes are negative
     * Lat and Long are in fractional degrees
     *
     * Written by Chuck Gantz- chuck.gantz@globalstar.com
     */
    static inline void LLtoUTM(const degree_t Lat, const degree_t Long,
                               meter_t &UTMNorthing, meter_t &UTMEasting,
                               char* UTMZone)
    {
        meter_t a = WGS84_A;
        double eccSquared = UTM_E2;
        double k0 = UTM_K0;

        degree_t LongOrigin;
        double eccPrimeSquared;
        meter_t N,M;
        double C, T;
        radian_t A;
        //Make sure the longitude is between -180.00 .. 179.9
        degree_t LongTemp = (Long+degree_t(180))-degree_t(int((Long+degree_t(180))/degree_t(360))*degree_t(360)-degree_t(180));

        radian_t LatRad = Lat;
        radian_t LongRad = LongTemp;
        radian_t LongOriginRad;
        int    ZoneNumber;

        ZoneNumber = int((LongTemp + degree_t(180))/degree_t(6)) + int(1);

        if( Lat >= degree_t(56.0) && Lat < degree_t(64.0) && LongTemp >= degree_t(3.0) && LongTemp < degree_t(12.0) )
            ZoneNumber = 32;

        // Special zones for Svalbard
        if( Lat >= degree_t(72.0) && Lat < degree_t(84.0) )
        {
            if(      LongTemp >= degree_t(0.0)  && LongTemp <  degree_t(9.0) ) ZoneNumber = 31;
            else if( LongTemp >= degree_t(9.0)  && LongTemp < degree_t(21.0) ) ZoneNumber = 33;
            else if( LongTemp >= degree_t(21.0) && LongTemp < degree_t(33.0) ) ZoneNumber = 35;
            else if( LongTemp >= degree_t(33.0) && LongTemp < degree_t(42.0) ) ZoneNumber = 37;
        }
        // +3 puts origin in middle of zone
        LongOrigin = degree_t((ZoneNumber - 1)*6 - 180 + 3);
        LongOriginRad = LongOrigin;

        //compute the UTM Zone from the latitude and longitude
        sprintf(UTMZone, "%d%c", ZoneNumber, UTMLetterDesignator(Lat));

        eccPrimeSquared = (eccSquared)/(1-eccSquared);

        using namespace units::math;

        N = a/sqrt(1-eccSquared*sin(LatRad)*sin(LatRad));
        T = (double)(tan(LatRad)*tan(LatRad));
        C = eccPrimeSquared*cos(LatRad)*cos(LatRad);
        A = cos(LatRad)*(LongRad-LongOriginRad).value();

        M = a*((1 - eccSquared/4 - 3*eccSquared*eccSquared/64
                - 5*eccSquared*eccSquared*eccSquared/256)
                 * (double)LatRad
               - (3*eccSquared/8 + 3*eccSquared*eccSquared/32
                  + 45*eccSquared*eccSquared*eccSquared/1024)
                  *sin(2*LatRad)
               + (15*eccSquared*eccSquared/256
                  + 45*eccSquared*eccSquared*eccSquared/1024)*sin(4*LatRad)
               - (35*eccSquared*eccSquared*eccSquared/3072)*sin(6*LatRad));

        UTMEasting = 
        (k0*N*(A+(1-T+C)*A*A*A/6
               + (5-18*T+T*T+72*C-58*eccPrimeSquared)*A*A*A*A*A/120)
         + (meter_t)500000.0);

        UTMNorthing = 
        (k0*(M+N*tan(LatRad)
             *(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
               + (61-58*T+T*T+600*C-330*eccPrimeSquared)*A*A*A*A*A*A/720)));

        if(Lat < degree_t(0))
        {
            //10000000 meter offset for southern hemisphere
            UTMNorthing += meter_t(10000000.0);
        }
    }

    /**
     * Converts UTM coords to lat/long.  Equations from USGS Bulletin 1532
     *
     * East Longitudes are positive, West longitudes are negative.
     * North latitudes are positive, South latitudes are negative
     * Lat and Long are in fractional degrees.
     *
     * Written by Chuck Gantz- chuck.gantz@globalstar.com
     */
    template<typename T>
    static inline void UTMtoLL(const T UTMNorthing, const T UTMEasting,
                               const char* UTMZone, double& Lat,  double& Long )
    {
        double k0 = UTM_K0;
        double a = WGS84_A;
        double eccSquared = UTM_E2;
        double eccPrimeSquared;
        double e1 = (1-sqrt(1-eccSquared))/(1+sqrt(1-eccSquared));
        double N1, T1, C1, R1, D, M;
        double LongOrigin;
        double mu, phi1Rad;
        double x, y;
        int ZoneNumber;
        char* ZoneLetter;

        x = UTMEasting - 500000.0; //remove 500,000 meter offset for longitude
        y = UTMNorthing;

        ZoneNumber = strtoul(UTMZone, &ZoneLetter, 10);
        if((*ZoneLetter - 'N') < 0)
        {
            //remove 10,000,000 meter offset used for southern hemisphere
            y -= 10000000.0;
        }

        //+3 puts origin in middle of zone
        LongOrigin = (ZoneNumber - 1)*6 - 180 + 3;
        eccPrimeSquared = (eccSquared)/(1-eccSquared);
        
        M = y / k0;
        mu = M/(a*(1-eccSquared/4-3*eccSquared*eccSquared/64
                   -5*eccSquared*eccSquared*eccSquared/256));
        
        phi1Rad = mu + ((3*e1/2-27*e1*e1*e1/32)*sin(2*mu) 
                        + (21*e1*e1/16-55*e1*e1*e1*e1/32)*sin(4*mu)
                        + (151*e1*e1*e1/96)*sin(6*mu));
        
        N1 = a/sqrt(1-eccSquared*sin(phi1Rad)*sin(phi1Rad));
        T1 = tan(phi1Rad)*tan(phi1Rad);
        C1 = eccPrimeSquared*cos(phi1Rad)*cos(phi1Rad);
        R1 = a*(1-eccSquared)/pow(1-eccSquared*sin(phi1Rad)*sin(phi1Rad), 1.5);
        D = x/(N1*k0);
        
        Lat = phi1Rad - ((N1*tan(phi1Rad)/R1)
                         *(D*D/2
                           -(5+3*T1+10*C1-4*C1*C1-9*eccPrimeSquared)*D*D*D*D/24
                           +(61+90*T1+298*C1+45*T1*T1-252*eccPrimeSquared
                             -3*C1*C1)*D*D*D*D*D*D/720));
        
        Lat = Lat * RAD_TO_DEG;
        
        Long = ((D-(1+2*T1+C1)*D*D*D/6
                 +(5-2*C1+28*T1-3*C1*C1+8*eccPrimeSquared+24*T1*T1)
                 *D*D*D*D*D/120)
                / cos(phi1Rad));
        Long = LongOrigin + Long * RAD_TO_DEG;
        
    }
} // end namespace UTM

#endif // _UTM_H

