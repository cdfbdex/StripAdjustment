#!/usr/bin/env python
# coding: utf-8

# ## readIMUGPSQuanergyM8:
# 
# - <b>Description</b>:  This module implement 1 class and 1 method for reading IMU and GPS information from Quanergy M8 Lidar.
# - <b>Developed by</b>: Juan Camilo Bonilla, Andrés Suarez, Carlos Ferrin.
# - <b>Company</b>:       LiDARiT          
# - <b>Input</b>:        File name with .out extension.
# - <b>Output</b>:       Pandas (Python) Dataframe with the following 20 fields: GPSTime(SoW), Latitude, Longitude, Altitude, X, Y, Z, Roll, Pitch, Heading, VelX, VelY,  VelZ, AccX, AccY, AccZ, GyroX, GyroY, GyroZ and WanderAngle

# #### 1. Import Libs

# In[1]:


import math, struct, sys
import os, mmap
import utm
import time
from numpy import array
import numpy as np
from datetime import datetime
from datetime import timezone
from multiprocessing import Pipe, Process
from tkinter import *
import pandas as pd


# #### 2. Class readSbet for handling data.

# In[2]:


class readSbet(object):

	def __init__(self, file_sbet, Data_offset=0):
		self.index = 0

		self.field_names_sbet = ('tiempo', 'latitud', 'longitud', 'altitud', 			'x_vel', 'y_vel', 'z_vel', 			'roll', 'pitch', 'heading', 'wander_angle', 			'x_acceleration', 'y_acceleration', 'z_acceleration', 			'x_angular_rate', 'y_angular_rate', 'z_angular_rate')

		self.offset = Data_offset * 136
		self.Data_offset = Data_offset
		self.mapa_sbet = file_sbet
		self.longi = len(self.mapa_sbet)
		self.data_blocks = self.num_datagrams()
		self.unpack_binary()

		

	def __iter__(self):
		return self

	def __next__(self):
		if self.Data_offset >= self.data_blocks:
			raise StopIteration
		self.offset = self.Data_offset * 136
		self.unpack_binary()
		self.Data_offset += 1
		return self.sbet_values
	
	def num_datagrams(self):
		if(self.longi%136) != 0:#------reemplaza (assert(longi%datagram_size== 0))
			print("Error in file length")
			print("longi%datagram_size =",self.longi%136)
			sys.exit()
		return int(self.longi / 136)

	@property
	def geo(self):
		return self.sbet_values

	def unpack_binary(self):
		self.values = struct.unpack('17d', self.mapa_sbet[ self.offset : self.offset + 136 ])
		self.sbet_values = dict(zip (self.field_names_sbet, self.values))

		#self.sbet_values['latitud'] = np.rad2deg(self.sbet_values['latitud'])
		#self.sbet_values['longitud'] = np.rad2deg(self.sbet_values['longitud'])       
		#self.cordeutm = utm.from_latlon(self.sbet_values['latitud'],self.sbet_values['longitud'])
		self.cordeutm = utm.from_latlon(np.rad2deg(self.sbet_values['latitud']),np.rad2deg(self.sbet_values['longitud']))
		self.sbet_values['X'] = self.cordeutm[0]
		self.sbet_values['Y'] = self.cordeutm[1]
		self.sbet_values['Z'] = self.sbet_values['altitud']
###############################################################################################################################################################


# #### 3. Main method for getting IMU and GPS information.

# In[3]:


def readIMUGPS(fileNameIMUGPS):
    file_sbet = fileNameIMUGPS
    sbet_file = open(file_sbet,'rb')
    sbet_size = os.path.getsize(file_sbet)
    mapa_sbet = mmap.mmap(sbet_file.fileno(),sbet_size, access=mmap.ACCESS_READ)
    ops = readSbet(mapa_sbet,0)
    
    colNames = ["GPSTime(SoW)","Latitude","Longitude","Altitude","X","Y","Z","Roll","Pitch","Heading",                "VelX","VelY","VelZ","AccX","AccY","AccZ","GyroX","GyroY","GyroZ","WanderAngle"]
    data = []

    for char in ops:
        time = float(char['tiempo'])            # Time (s)
        latitude = float(char['latitud'])       # Degrees (°)
        longitude = float(char['longitud'])     # Degrees (°)
        altitude = float(char['altitud'])       # Meters (m)
        x = float(char['X'])                    # Meters (m)
        y = float(char['Y'])                    # Meters (m)
        z = float(char['Z'])                    # Meters (m)
        roll = float(char['roll'])              # Degrees (°)
        pitch = float(char['pitch'])            # Degrees (°)
        heading = float(char['heading'])        # Degrees (°)
        velx = float(char['x_vel'])             # Speed (m/s)
        vely = float(char['y_vel'])             # Speed (m/s)
        velz = float(char['z_vel'])             # Speed (m/s)
        accx = float(char['x_acceleration'])    # Acceleration (m/s^2)
        accy = float(char['y_acceleration'])    # Acceleration (m/s^2)
        accz = float(char['z_acceleration'])    # Acceleration (m/s^2)
        gyrox = float(char['x_angular_rate'])   # Angular Speed (rad/s)
        gyroy = float(char['y_angular_rate'])   # Angular Speed (rad/s)
        gyroz = float(char['z_angular_rate'])   # Angular Speed (rad/s)
        wanderang = float(char['wander_angle']) # Degrees (°)
        register = [time, latitude, longitude, altitude, x, y, z, roll, pitch, heading,                     velx, vely, velz, accx, accy, accz, gyrox, gyroy, gyroz, wanderang]
        data.append(register.copy())
    df = pd.DataFrame(data, columns = colNames)
    return df


# #### 4.  Example:

# In[2]:


# Comment when exporting from IPYNB to PY
'''
fileNameInput = 'Data_samples/Data1_IMUGPS.OUT'
df = readIMUGPS(fileNameInput)
df.head()
print(fileNameInput)
'''
