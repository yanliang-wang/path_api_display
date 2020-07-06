#ifndef GPSCONVERSION_H
#define GPSCONVERSION_H
#include <cmath>



/**
 * \brief Converts lat,lon coordinates into ENU frame
 *
 * The implementation here is according to the paper:
 * - "Conversion of Geodetic coordinates to the Local Tangent Plane" Version 2.01.
 * - "The basic reference for this paper is J.Farrell & M.Barth 'The Global Positioning System & Inertial Navigation'"
 * - Also helpful is Wikipedia: http://en.wikipedia.org/wiki/Geodetic_datum
 * - Taken from https://gist.github.com/govert/1b373696c9a27ff4c72a
 */
class GPSConversion {

public:

    // WGS-84 geodetic constants
    static constexpr double a = 6378137;              // WGS-84 Earth semimajor axis (m)
    static constexpr double b = 6356752.3142;         // WGS-84 Earth semiminor axis (m)
    static constexpr double x_pi = 3.14159265358979324 * 3000.0 / 180.0;
    static constexpr double  pi = 3.1415926535897932384626;  // π
    //static constexpr double a = 6378245.0;  // 长半轴
    static constexpr double ee = 0.00669342162296594323;  // 偏心率平方
    /**
     * Converts WGS-84 Geodetic point (lat, lon, h) to the
     * Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z).
     */
     //将WGS-84大地测量点（lat，lon，h）转换为以地球为中心的地球固定（ECEF）坐标（x，y，z）。
    static void GeodeticToEcef(double lat, double lon, double h, double& x, double& y, double& z) {
        //经度，纬度，高度
        double f = (a - b) / a;      // Ellipsoid Flatness
        double e_sq = f * (2 - f);   // Square of Eccentricity

        // Convert to radians in notation consistent with the paper:
        double lambda = DegreeToRadian(lat);
        double phi = DegreeToRadian(lon);
        double s = sin(lambda);
        double N = a / sqrt(1 - e_sq * s * s);

        double sin_lambda = sin(lambda);
        double cos_lambda = cos(lambda);
        double cos_phi = cos(phi);
        double sin_phi = sin(phi);

        x = (h + N) * cos_lambda * cos_phi;
        y = (h + N) * cos_lambda * sin_phi;
        z = (h + (1 - e_sq) * N) * sin_lambda;
    }

    /**
     * Converts the Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z) to
     * East-North-Up coordinates in a Local Tangent Plane that is centered at the
     * (WGS-84) Geodetic point (lat0, lon0, h0).
     */
     //将以地球为中心的地球固定（ECEF）坐标（x，y，z）转换为以大地点为中心的局部切线平面中的东北向上坐标（east，north，up）ENU。
     //得到的enu坐标是以（lat0, lon0, h0）为远点计算得到的
    static void EcefToEnu(double x, double y, double z, double lat0, double lon0, double h0,
                          double& xEast, double& yNorth, double& zUp) {

        double f = (a - b) / a;      // Ellipsoid Flatness
        double e_sq = f * (2 - f);   // Square of Eccentricity

        // Convert to radians in notation consistent with the paper:
        double lambda = DegreeToRadian(lat0);//度换位弧度
        double phi = DegreeToRadian(lon0);
        double s = sin(lambda);
        double N = a / sqrt(1 - e_sq * s * s);

        double sin_lambda = sin(lambda);
        double cos_lambda = cos(lambda);
        double cos_phi = cos(phi);
        double sin_phi = sin(phi);

        //先讲（（lat0, lon0, h0））转换为ecef表示的坐标（x0,y0,z0）
        double x0 = (h0 + N) * cos_lambda * cos_phi;
        double y0 = (h0 + N) * cos_lambda * sin_phi;
        double z0 = (h0 + (1 - e_sq) * N) * sin_lambda;

        double xd, yd, zd;
        xd = x - x0;
        yd = y - y0;
        zd = z - z0;

        // This is the matrix multiplication
        xEast = -sin_phi * xd + cos_phi * yd;
        yNorth = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd;
        zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;
    }

    /**
     * Converts the geodetic WGS-84 coordinated (lat, lon, h) to
     * East-North-Up coordinates in a Local Tangent Plane that is centered at the
     * (WGS-84) Geodetic point (lat0, lon0, h0).
     */
     //将大地测量WGS-84坐标系（lat，lon，h）转换为以（WGS-84）大地点（lat0，lon0，h0）为中心的局部切线平面中的东北向上坐标。
    static void GeodeticToEnu(double lat, double lon, double h, double lat0, double lon0, double h0,
                              double& xEast, double& yNorth, double& zUp) {
        double x, y, z;
        GeodeticToEcef(lat, lon, h, x, y, z);
        EcefToEnu(x, y, z, lat0, lon0, h0, xEast, yNorth, zUp);
    }
    static void wgs84_to_gcj02(double& lng, double& lat){
        //默认坐标在国内，不做国外情况的讨论
        double dlat = _transformlat(lng - 105.0, lat - 35.0);
        double dlng = _transformlng(lng - 105.0, lat - 35.0);
        double radlat = lat / 180.0 * pi;
        double magic = sin(radlat);
        magic = 1 - ee * magic * magic;
        double sqrtmagic = sqrt(magic);
        dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * pi);
        dlng = (dlng * 180.0) / (a / sqrtmagic * cos(radlat) * pi);

        lat = lat + dlat;
        lng = lng + dlng;
        //return [mglng, mglat]
        }

private:

    /**
     * Converts degrees to radians
     * \param angle The angle in degrees
     * \return Angle converted into radians
     */
    static double DegreeToRadian(double angle) {
        return M_PI * angle / 180.0;
    }

    static double _transformlat(double lng, double lat){
        double ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * sqrt(fabs(lng));
        ret += (20.0 * sin(6.0 * lng * pi) + 20.0 * sin(2.0 * lng * pi)) * 2.0 / 3.0;
        ret += (20.0 * sin(lat * pi) + 40.0 * sin(lat / 3.0 * pi)) * 2.0 / 3.0;
        ret += (160.0 * sin(lat / 12.0 * pi) + 320 * sin(lat * pi / 30.0)) * 2.0 / 3.0;
        return ret;
    }
    static double _transformlng(double lng, double lat){
        double ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * sqrt(fabs(lng));
        ret += (20.0 * sin(6.0 * lng * pi) + 20.0 *sin(2.0 * lng * pi)) * 2.0 / 3.0;
        ret += (20.0 * sin(lng * pi) + 40.0 *sin(lng / 3.0 * pi)) * 2.0 / 3.0;
        ret += (150.0 * sin(lng / 12.0 * pi) + 300.0 *sin(lng / 30.0 * pi)) * 2.0 / 3.0;
        return ret;
    }





};

#endif //GPSCONVERSION_H