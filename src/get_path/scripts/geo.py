# -*- coding: utf-8 -*-
# @Author: wuzida
# @Date:   2018-12-28 15:20:54
# @Last Modified by:   wuzida
# @Last Modified time: 2018-12-29 16:49:16
import math

a = 6378137
b = 6356752.3142
f = (a - b) / a
e_sq = f * (2-f)
pi = 3.14159265359

def geodetic_to_ecef(lat, lon, h):
    # (lat, lon) in degrees
    # h in meters
    lamb = math.radians(lat)
    phi = math.radians(lon)
    s = math.sin(lamb)
    N = a / math.sqrt(1 - e_sq * s * s)

    sin_lambda = math.sin(lamb)
    cos_lambda = math.cos(lamb)
    sin_phi = math.sin(phi)
    cos_phi = math.cos(phi)

    x = (h + N) * cos_lambda * cos_phi
    y = (h + N) * cos_lambda * sin_phi
    z = (h + (1 - e_sq) * N) * sin_lambda

    return x, y, z

def ecef_to_enu(x, y, z, lat0, lon0, h0):
    lamb = math.radians(lat0)
    phi = math.radians(lon0)
    s = math.sin(lamb)
    N = a / math.sqrt(1 - e_sq * s * s)

    sin_lambda = math.sin(lamb)
    cos_lambda = math.cos(lamb)
    sin_phi = math.sin(phi)
    cos_phi = math.cos(phi)

    x0 = (h0 + N) * cos_lambda * cos_phi
    y0 = (h0 + N) * cos_lambda * sin_phi
    z0 = (h0 + (1 - e_sq) * N) * sin_lambda

    xd = x - x0
    yd = y - y0
    zd = z - z0

    t = -cos_phi * xd -  sin_phi * yd

    xEast = -sin_phi * xd + cos_phi * yd
    yNorth = t * sin_lambda  + cos_lambda * zd
    zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd

    return xEast, yNorth, zUp

def enu_to_ecef(xEast, yNorth, zUp, lat0, lon0, h0):
    lamb = math.radians(lat0)
    phi = math.radians(lon0)
    s = math.sin(lamb)
    N = a / math.sqrt(1 - e_sq * s * s)

    sin_lambda = math.sin(lamb)
    cos_lambda = math.cos(lamb)
    sin_phi = math.sin(phi)
    cos_phi = math.cos(phi)

    x0 = (h0 + N) * cos_lambda * cos_phi
    y0 = (h0 + N) * cos_lambda * sin_phi
    z0 = (h0 + (1 - e_sq) * N) * sin_lambda

    t = cos_lambda * zUp - sin_lambda * yNorth

    zd = sin_lambda * zUp + cos_lambda * yNorth
    xd = cos_phi * t - sin_phi * xEast 
    yd = sin_phi * t + cos_phi * xEast

    x = xd + x0 
    y = yd + y0 
    z = zd + z0 

    return x, y, z

def ecef_to_geodetic(x, y, z):
   # Convert from ECEF cartesian coordinates to 
   # latitude, longitude and height.  WGS-84
    x2 = x ** 2 
    y2 = y ** 2 
    z2 = z ** 2 

    a = 6378137.0000    # earth radius in meters
    b = 6356752.3142    # earth semiminor in meters 
    e = math.sqrt (1-(b/a)**2) 
    b2 = b*b 
    e2 = e ** 2 
    ep = e*(a/b) 
    r = math.sqrt(x2+y2) 
    r2 = r*r 
    E2 = a ** 2 - b ** 2 
    F = 54*b2*z2 
    G = r2 + (1-e2)*z2 - e2*E2 
    c = (e2*e2*F*r2)/(G*G*G) 
    s = ( 1 + c + math.sqrt(c*c + 2*c) )**(1/3) 
    P = F / (3 * (s+1/s+1)**2 * G*G) 
    Q = math.sqrt(1+2*e2*e2*P) 
    ro = -(P*e2*r)/(1+Q) + math.sqrt((a*a/2)*(1+1/Q) - (P*(1-e2)*z2)/(Q*(1+Q)) - P*r2/2) 
    tmp = (r - e2*ro) ** 2 
    U = math.sqrt( tmp + z2 ) 
    V = math.sqrt( tmp + (1-e2)*z2 ) 
    zo = (b2*z)/(a*V) 

    height = U*( 1 - b2/(a*V) ) 
    
    lat = math.atan( (z + ep*ep*zo)/r ) 

    temp = math.atan(y/x) 
    if x >=0 :    
        long = temp 
    elif (x < 0) & (y >= 0):
        long = pi + temp 
    else :
        long = temp - pi 

    lat0 = lat/(pi/180) 
    lon0 = long/(pi/180) 
    h0 = height 

    return lat0, lon0, h0


def geodetic_to_enu(lat, lon, h, lat_ref, lon_ref, h_ref):

    x, y, z = geodetic_to_ecef(lat, lon, h)
    
    return ecef_to_enu(x, y, z, lat_ref, lon_ref, h_ref)

def enu_to_geodetic(xEast, yNorth, zUp, lat_ref, lon_ref, h_ref):

    x,y,z = enu_to_ecef(xEast, yNorth, zUp, lat_ref, lon_ref, h_ref)

    return ecef_to_geodetic(x,y,z)
