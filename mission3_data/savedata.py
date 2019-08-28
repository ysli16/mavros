#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Aug 17 13:57:16 2019

@author: ysli
"""
import pickle
import pandas as pd
import cv2

pkfile = open('data.p', 'rb')
alldata=pickle.load(pkfile)
for data in alldata:
    depth_gps = pd.DataFrame({'depth':[data[1]], 'longitude': [data[2][0]],'latitude':[data[2][1]]})
    depth_gps.to_csv('depth_gps', index=False, mode='a+', header=False)
    cv2.imwrite('./image/'+str(data[2][0])+"_"+str(data[2][1])+".jpg", data[0])
