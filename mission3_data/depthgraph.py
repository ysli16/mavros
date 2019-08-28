#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Aug 19 11:40:20 2019

@author: ysli
"""
import pandas as pd
import numpy as np
import pyproj
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
def gridZone(lat, lon):
    """Find UTM zone and MGRS band for latitude and longitude.

       :param lat: latitude in degrees, negative is South.
       :param lon: longitude in degrees, negative is West.
       :returns: (zone, band) tuple.
       :raises: :exc:`ValueError` if lon not in [-180..180] or if lat
                has no corresponding band letter.

       :todo: handle polar (UPS_) zones: A, B, Y, Z.
    """
    if -180.0 > lon or lon > 180.0:
        raise ValueError('invalid longitude: ' + str(lon))
    zone = int((lon + 180.0)//6.0) + 1
    band = ' '
    if    84 >= lat and lat >= 72: band = 'X'
    elif  72 > lat and lat >= 64:  band = 'W'
    elif  64 > lat and lat >= 56:  band = 'V'
    elif  56 > lat and lat >= 48:  band = 'U'
    elif  48 > lat and lat >= 40:  band = 'T'
    elif  40 > lat and lat >= 32:  band = 'S'
    elif  32 > lat and lat >= 24:  band = 'R'
    elif  24 > lat and lat >= 16:  band = 'Q'
    elif  16 > lat and lat >= 8:   band = 'P'
    elif   8 > lat and lat >= 0:   band = 'N'
    elif   0 > lat and lat >= -8:  band = 'M'
    elif  -8 > lat and lat >= -16: band = 'L'
    elif -16 > lat and lat >= -24: band = 'K'
    elif -24 > lat and lat >= -32: band = 'J'
    elif -32 > lat and lat >= -40: band = 'H'
    elif -40 > lat and lat >= -48: band = 'G'
    elif -48 > lat and lat >= -56: band = 'F'
    elif -56 > lat and lat >= -64: band = 'E'
    elif -64 > lat and lat >= -72: band = 'D'
    elif -72 > lat and lat >= -80: band = 'C'
    else: raise ValueError('latitude out of UTM range: ' + str(lat))
    return (zone, band)

def fromLatLong(latitude, longitude):
    z, b = gridZone(latitude, longitude)
    utm_proj = pyproj.Proj(proj='utm', zone=z, datum='WGS84')
    e, n = utm_proj(longitude, latitude)
    return (e, n)

dtype={'depth':np.int,'latitude':np.float,'longitude':np.float}
df=pd.read_csv('depth_gps',header=None,dtype=dtype)
depth_gps=df.values
depth_gps=np.delete(depth_gps,np.argwhere(depth_gps[:,0]==-1).flatten(),axis=0)
depth_gps=np.delete(depth_gps,np.argwhere(depth_gps[:,0]>15).flatten(),axis=0)
coords=np.zeros(shape=[0,2])
for point in depth_gps:
    coord=fromLatLong(point[2],point[1])
    coords=np.append(coords,[[coord[0],coord[1]]],axis=0)
fig = plt.figure()
ax = Axes3D(fig)
x=coords[:,0]-coords.mean(axis=0)[0]
y=coords[:,1]-coords.mean(axis=0)[1]
z=-depth_gps[:,0]

ax.plot_trisurf(x, y, z,cmap=plt.cm.coolwarm)
plt.show()


