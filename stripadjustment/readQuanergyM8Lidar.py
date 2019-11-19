#!/usr/bin/env python
# coding: utf-8

# ## readQuanergyM8Lidar:
# 
# - <b>Description</b>:  This module implement 3 classes and 1 method for reading Point Cloud information from Quanergy M8 Lidar.
# - <b>Developed by</b>: Juan Camilo Bonilla, Andrés Suarez, Carlos Ferrin.
# - <b>Company</b>:       LiDARiT          
# - <b>Input</b>:        File name with .lid extension.
# - <b>Output</b>:       Pandas (Python) Dataframe with the following 10 fields: PacketTime(SoW),  Block, Azimuth, Channel, Return, Distance, Intensity, X, Y and Z.

# #### 1. Import Libs

# In[11]:


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


# #### 2. Classes for handling data.

# In[12]:


class readLidBlock(object):

	def __init__(self, map_lid, packet_data, block_data = 0, offset_null = 0, final_packet = 0, filtered_range = 5):
		'''
		map_lid = mapeo de memoria
		packet_data = paquete de datos desde el cual se quieren generar los datos
		block_data = por defecto esta en "0" dado que el tiempo solo se obtiene del bloque 0 y 49
		offset_null = el numero de datos nulos que se saltan para incial a contar los paquetes; en los sistemas con el sensor M8
			este dato no es constante para el resto de los sensores siempre es 256
		final_packet = paquete hasta el cual se generaran los datos
		filtered_range = es el rango de datos que se omiten para evitar mostara los puntos generados por el drone
		'''
		#self.sin_and_cos_IDS[0]---> Senos(ID_laser_vertical)     self.sin_and_cos_IDS[1]---> Cosenos(ID_laser_vertical)
		#self.sin_and_cos_IDS = ([0.05576925045718507,0.0,-0.05576925045718507,-0.11077518341729593,-0.16444467944570706,-0.21628618000315286,-0.265960334699925,-0.3131471026627096],\
		#	[0.998443684292431,1.0,0.998443684292431,0.9938454903750705,0.9863863073877286,0.9763300099554677,0.9639839730858101,0.9497046341331342])
		self.sin_and_cos_IDS = [[-0.3131471, -0.2659603, -0.2162862, -0.1644447, -0.1107752, -0.0557693, 0.0, 0.0557693],			[0.9497046, 0.963984, 0.97633, 0.9863863, 0.9938455, 0.9984437, 1.0, 0.9984437]]
		self.filtered_range = filtered_range/0.00001
		self.block = block_data
		self.packetByte = packet_data * 6632
		self.packet = packet_data
		self.offset_null = offset_null
		self.map_lid = map_lid
		self.final_packet = final_packet
		self.time_packet = 0
		self.shots =   [0, 4, 2, 6, 1, 5, 3, 7]
		self.ID_laser_vertical = [-0.318505, -0.2692, -0.218009, -0.165195, -0.111003, -0.0557982, 0.0, 0.0557982]
		self.channel = 0
		self.retorno = 0
		self.values()
		self.xyz_channel()
		self.xyz_return()

				
	def __iter__(self):
		return self

	def __next__(self):
		if self.packet > self.final_packet:
			#for self.retorno in range(0,3):
			#	self.xyz_return()
			raise StopIteration

		
		if self.retorno >= 2:
			self.retorno  = 0						
			if(self.channel >= 7):
				self.channel = 0
				if(self.block >= 49):
					self.block = 0
					self.packet += 1
					if self.packet > self.final_packet:
						raise StopIteration
					else:
						self.packetByte = self.packet * 6632
				else:
					self.block += 1	
				self.values()
				
			else:
				self.channel += 1
			self.xyz_channel()
			
		else:
			self.retorno +=1
		self.xyz_return()

		return self.output

	def estructura_lid(self, block):
		'''
		if self.block == 0:
			file_lid = ('bandera','log_men','timeseco','timenano',\
				'apiMa','apiMi','apipa','res1','posicion','res2',\
				'dis_7_1','dis_6_1','dis_5_1','dis_4_1','dis_3_1',\
				'dis_2_1','dis_1_1','dis_0_1','dis_7_2','dis_6_2',\
				'dis_5_2','dis_4_2','dis_3_2','dis_2_2','dis_1_2',\
				'dis_0_2','dis_7_3','dis_6_3','dis_5_3','dis_4_3',\
				'dis_3_3','dis_2_3','dis_1_3','dis_0_3','ref_7_1',\
				'ref_6_1','ref_5_1','ref_4_1','ref_3_1','ref_2_1',\
				'ref_1_1','ref_0_1','ref_7_2','ref_6_2','ref_5_2',\
				'ref_4_2','ref_3_2','ref_2_2','ref_1_2','ref_0_2',\
				'ref_7_3','ref_6_3','ref_5_3','ref_4_3','ref_3_3',\
				'ref_2_3','ref_1_3','ref_0_3','ret_1','ret_2',\
				'ret_3','ret_4','ret_5','ret_6','ret_7','ret_8')
		elif ((self.block >= 1) & (self.block <= 48)):
			file_lid = ('posicion','reservado',\
				'dis_7_1','dis_6_1','dis_5_1','dis_4_1','dis_3_1',\
				'dis_2_1','dis_1_1','dis_0_1','dis_7_2','dis_6_2',\
				'dis_5_2','dis_4_2','dis_3_2','dis_2_2','dis_1_2',\
				'dis_0_2','dis_7_3','dis_6_3','dis_5_3','dis_4_3',\
				'dis_3_3','dis_2_3','dis_1_3','dis_0_3','ref_7_1',\
				'ref_6_1','ref_5_1','ref_4_1','ref_3_1','ref_2_1',\
				'ref_1_1','ref_0_1','ref_7_2','ref_6_2','ref_5_2',\
				'ref_4_2','ref_3_2','ref_2_2','ref_1_2','ref_0_2',\
				'ref_7_3','ref_6_3','ref_5_3','ref_4_3','ref_3_3',\
				'ref_2_3','ref_1_3','ref_0_3','ret_1','ret_2',\
				'ret_3','ret_4','ret_5','ret_6','ret_7','ret_8')
		elif self.block == 49:
			file_lid = ('posicion','reservado',\
				'dis_7_1','dis_6_1','dis_5_1','dis_4_1','dis_3_1',\
				'dis_2_1','dis_1_1','dis_0_1','dis_7_2','dis_6_2',\
				'dis_5_2','dis_4_2','dis_3_2','dis_2_2','dis_1_2',\
				'dis_0_2','dis_7_3','dis_6_3','dis_5_3','dis_4_3',\
				'dis_3_3','dis_2_3','dis_1_3','dis_0_3','ref_7_1',\
				'ref_6_1','ref_5_1','ref_4_1','ref_3_1','ref_2_1',\
				'ref_1_1','ref_0_1','ref_7_2','ref_6_2','ref_5_2',\
				'ref_4_2','ref_3_2','ref_2_2','ref_1_2','ref_0_2',\
				'ref_7_3','ref_6_3','ref_5_3','ref_4_3','ref_3_3',\
				'ref_2_3','ref_1_3','ref_0_3','ret_1','ret_2',\
				'ret_3','ret_4','ret_5','ret_6','ret_7','ret_8',\
				'timeseco','timenano','apiversion','estado')
		'''
		'''
		if value == 1:
			file_lid = ('bandera','log_men','timeseco','timenano',\
				'apiMa','apiMi','apipa','res1','posicion','res2',\
				'dis_0_1','dis_1_1','dis_2_1','dis_3_1','dis_4_1',\
				'dis_5_1','dis_6_1','dis_7_1','dis_0_2','dis_1_2',\
				'dis_2_2','dis_3_2','dis_4_2','dis_5_2','dis_6_2',\
				'dis_7_2','dis_0_3','dis_1_3','dis_2_3','dis_3_3',\
				'dis_4_3','dis_5_3','dis_6_3','dis_7_3','ref_0_1',\
				'ref_1_1','ref_2_1','ref_3_1','ref_4_1','ref_5_1',\
				'ref_6_1','ref_7_1','ref_0_2','ref_1_2','ref_2_2',\
				'ref_3_2','ref_4_2','ref_5_2','ref_6_2','ref_7_2',\
				'ref_0_3','ref_1_3','ref_2_3','ref_3_3','ref_4_3',\
				'ref_5_3','ref_6_3','ref_7_3','ret_1','ret_2',\
				'ret_3','ret_4','ret_5','ret_6','ret_7','ret_8')
		elif value == 2:
			file_lid = ('posicion','reservado',\
				'dis_0_1','dis_1_1','dis_2_1','dis_3_1','dis_4_1',\
				'dis_5_1','dis_6_1','dis_7_1','dis_0_2','dis_1_2',\
				'dis_2_2','dis_3_2','dis_4_2','dis_5_2','dis_6_2',\
				'dis_7_2','dis_0_3','dis_1_3','dis_2_3','dis_3_3',\
				'dis_4_3','dis_5_3','dis_6_3','dis_7_3','ref_0_1',\
				'ref_1_1','ref_2_1','ref_3_1','ref_4_1','ref_5_1',\
				'ref_6_1','ref_7_1','ref_0_2','ref_1_2','ref_2_2',\
				'ref_3_2','ref_4_2','ref_5_2','ref_6_2','ref_7_2',\
				'ref_0_3','ref_1_3','ref_2_3','ref_3_3','ref_4_3',\
				'ref_5_3','ref_6_3','ref_7_3','ret_1','ret_2',\
				'ret_3','ret_4','ret_5','ret_6','ret_7','ret_8')
		elif value == 3:
			file_lid = ('posicion','reservado',\
				'dis_0_1','dis_1_1','dis_2_1','dis_3_1','dis_4_1',\
				'dis_5_1','dis_6_1','dis_7_1','dis_0_2','dis_1_2',\
				'dis_2_2','dis_3_2','dis_4_2','dis_5_2','dis_6_2',\
				'dis_7_2','dis_0_3','dis_1_3','dis_2_3','dis_3_3',\
				'dis_4_3','dis_5_3','dis_6_3','dis_7_3','ref_0_1',\
				'ref_1_1','ref_2_1','ref_3_1','ref_4_1','ref_5_1',\
				'ref_6_1','ref_7_1','ref_0_2','ref_1_2','ref_2_2',\
				'ref_3_2','ref_4_2','ref_5_2','ref_6_2','ref_7_2',\
				'ref_0_3','ref_1_3','ref_2_3','ref_3_3','ref_4_3',\
				'ref_5_3','ref_6_3','ref_7_3','ret_1','ret_2',\
				'ret_3','ret_4','ret_5','ret_6','ret_7','ret_8',\
				'timeseco','timenano','apiversion','estado')
		
		'''
			
		
		if self.block == 0:
			file_lid = ('bandera','log_men','timeseco','timenano',				'apiMa','apiMi','apipa','res1','posicion','res2',				'dis_0_1','dis_4_1','dis_2_1','dis_6_1','dis_1_1',				'dis_5_1','dis_3_1','dis_7_1','dis_0_2','dis_4_2',				'dis_2_2','dis_6_2','dis_1_2','dis_5_2','dis_3_2',				'dis_7_2','dis_0_3','dis_4_3','dis_2_3','dis_6_3',				'dis_1_3','dis_5_3','dis_3_3','dis_7_3','ref_0_1',				'ref_4_1','ref_2_1','ref_6_1','ref_1_1','ref_5_1',				'ref_3_1','ref_7_1','ref_0_2','ref_4_2','ref_2_2',				'ref_6_2','ref_1_2','ref_5_2','ref_3_2','ref_7_2',				'ref_0_3','ref_4_3','ref_2_3','ref_6_3','ref_1_3',				'ref_5_3','ref_3_3','ref_7_3','ret_1','ret_2',				'ret_3','ret_4','ret_5','ret_6','ret_7','ret_8')
		elif ((self.block >= 1) & (self.block <= 48)):
			file_lid = ('posicion','reservado',				'dis_0_1','dis_4_1','dis_2_1','dis_6_1','dis_1_1',				'dis_5_1','dis_3_1','dis_7_1','dis_0_2','dis_4_2',				'dis_2_2','dis_6_2','dis_1_2','dis_5_2','dis_3_2',				'dis_7_2','dis_0_3','dis_4_3','dis_2_3','dis_6_3',				'dis_1_3','dis_5_3','dis_3_3','dis_7_3','ref_0_1',				'ref_4_1','ref_2_1','ref_6_1','ref_1_1','ref_5_1',				'ref_3_1','ref_7_1','ref_0_2','ref_4_2','ref_2_2',				'ref_6_2','ref_1_2','ref_5_2','ref_3_2','ref_7_2',				'ref_0_3','ref_4_3','ref_2_3','ref_6_3','ref_1_3',				'ref_5_3','ref_3_3','ref_7_3','ret_1','ret_2',				'ret_3','ret_4','ret_5','ret_6','ret_7','ret_8')
		elif self.block == 49:
			file_lid = ('posicion','reservado',				'dis_0_1','dis_4_1','dis_2_1','dis_6_1','dis_1_1',				'dis_5_1','dis_3_1','dis_7_1','dis_0_2','dis_4_2',				'dis_2_2','dis_6_2','dis_1_2','dis_5_2','dis_3_2',				'dis_7_2','dis_0_3','dis_4_3','dis_2_3','dis_6_3',				'dis_1_3','dis_5_3','dis_3_3','dis_7_3','ref_0_1',				'ref_4_1','ref_2_1','ref_6_1','ref_1_1','ref_5_1',				'ref_3_1','ref_7_1','ref_0_2','ref_4_2','ref_2_2',				'ref_6_2','ref_1_2','ref_5_2','ref_3_2','ref_7_2',				'ref_0_3','ref_4_3','ref_2_3','ref_6_3','ref_1_3',				'ref_5_3','ref_3_3','ref_7_3','ret_1','ret_2',				'ret_3','ret_4','ret_5','ret_6','ret_7','ret_8',				'timeseco','timenano','apiversion','estado')
		return file_lid
	
	def deco_lid(self, block):
		if self.block == 0:
			decobytes = '!IIIIBBBBHHIIIIIIIIIIIIIIIIIIIIIIII				BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB'
		elif ((self.block >= 1) & (self.block <= 48)):
			decobytes = '!HHIIIIIIIIIIIIIIIIIIIIIIII				BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB'
		elif self.block == 49:
			decobytes = '!HHIIIIIIIIIIIIIIIIIIIIIIII				BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBIIHH'
		else:
			print("Error en el valor ingresado a decobytes_lid")
		return decobytes

	def sub_set(self, block, packet, map_lid):
		if self.block == 0:
			offset = self.packetByte + self.offset_null
			subset = self.map_lid[offset : offset + 152]
		elif ((self.block >= 1) & (self.block <= 48)):
			offset = 152 + ((self.block-1)*132) + self.packetByte + self.offset_null
			subset = self.map_lid[offset : offset + 132]
		elif self.block == 49:
			offset = 152 + ((self.block-1)*132) + self.packetByte + self.offset_null
			subset = self.map_lid[offset : offset + 144]
		return subset

	def values(self):
		pre_values = struct.unpack(self.deco_lid(self.block), self.sub_set(self.block, self.packetByte, self.map_lid))
		self.lid_values = dict(list(zip (self.estructura_lid(self.block) , pre_values)))
		self.lid_values['Azimuth'] = ( (self.lid_values['posicion']+1 - 0) / (10400 - 0) ) * (2*math.pi - 0) + 0
		if (self.block != 0):
			self.lid_values['bandera'] = 256
		
		if ((self.block == 49)|(self.block == 0)):		
			SoW = time.strftime("%w:%H:%M:%S:dia %d", time.gmtime(self.lid_values['timeseco']))
			SoW = SoW.split(':')
			for x in range (0,4):
				SoW[x] = int(SoW[x])	
			seconds_week = round((SoW[3]+SoW[2]*60+SoW[1]*3600+SoW[0]*86400),3)
			self.lid_values['tiempo'] = seconds_week + (self.lid_values['timenano']/1000000000) + 18
			self.time_packet = self.lid_values['tiempo']
		else:
			self.lid_values['tiempo'] = None
		self.lid_values['block'] = self.block
		
		
	@property
	def values_data(self):
		#self.values()
		return self.lid_values

	def distance(self):
		self.distan = [0,0,0]
		if(self.channel == 0):
			self.distan[0] = self.lid_values['dis_0_1']
			self.distan[1] = self.lid_values['dis_0_2']
			self.distan[2] = self.lid_values['dis_0_3']
		elif(self.channel == 1):
			self.distan[0] = self.lid_values['dis_1_1']
			self.distan[1] = self.lid_values['dis_1_2']
			self.distan[2] = self.lid_values['dis_1_3']
		elif(self.channel == 2):
			self.distan[0] = self.lid_values['dis_2_1']
			self.distan[1] = self.lid_values['dis_2_2']
			self.distan[2] = self.lid_values['dis_2_3']
		elif(self.channel == 3):
			self.distan[0] = self.lid_values['dis_3_1']
			self.distan[1] = self.lid_values['dis_3_2']
			self.distan[2] = self.lid_values['dis_3_3']
		elif(self.channel == 4):
			self.distan[0] = self.lid_values['dis_4_1']
			self.distan[1] = self.lid_values['dis_4_2']
			self.distan[2] = self.lid_values['dis_4_3']
		elif(self.channel == 5):
			self.distan[0] = self.lid_values['dis_5_1']
			self.distan[1] = self.lid_values['dis_5_2']
			self.distan[2] = self.lid_values['dis_5_3']
		elif(self.channel == 6):
			self.distan[0] = self.lid_values['dis_6_1']
			self.distan[1] = self.lid_values['dis_6_2']
			self.distan[2] = self.lid_values['dis_6_3']
		elif(self.channel == 7):
			self.distan[0] = self.lid_values['dis_7_1']
			self.distan[1] = self.lid_values['dis_7_2']
			self.distan[2] = self.lid_values['dis_7_3']
	
	def Intensities(self):
		self.refle = [0,0,0]
		if(self.channel == 0):
			self.refle[0] = self.lid_values['ref_0_1']
			self.refle[1] = self.lid_values['ref_0_2']
			self.refle[2] = self.lid_values['ref_0_3']
		elif(self.channel == 1):
			self.refle[0] = self.lid_values['ref_1_1']
			self.refle[1] = self.lid_values['ref_1_2']
			self.refle[2] = self.lid_values['ref_1_3']
		elif(self.channel == 2):
			self.refle[0] = self.lid_values['ref_2_1']
			self.refle[1] = self.lid_values['ref_2_2']
			self.refle[2] = self.lid_values['ref_2_3']
		elif(self.channel == 3):
			self.refle[0] = self.lid_values['ref_3_1']
			self.refle[1] = self.lid_values['ref_3_2']
			self.refle[2] = self.lid_values['ref_3_3']
		elif(self.channel == 4):
			self.refle[0] = self.lid_values['ref_4_1']
			self.refle[1] = self.lid_values['ref_4_2']
			self.refle[2] = self.lid_values['ref_4_3']
		elif(self.channel == 5):
			self.refle[0] = self.lid_values['ref_5_1']
			self.refle[1] = self.lid_values['ref_5_2']
			self.refle[2] = self.lid_values['ref_5_3']
		elif(self.channel == 6):
			self.refle[0] = self.lid_values['ref_6_1']
			self.refle[1] = self.lid_values['ref_6_2']
			self.refle[2] = self.lid_values['ref_6_3']
		elif(self.channel == 7):
			self.refle[0] = self.lid_values['ref_7_1']
			self.refle[1] = self.lid_values['ref_7_2']
			self.refle[2] = self.lid_values['ref_7_3']

	def xyz_channel(self):

		self.distance()
		
		if self.distan[0] >= self.filtered_range:#5.5e+6:

			self.Intensities()	
			self.data_channel = {'Azimuth':self.lid_values['Azimuth'], 'packet_time':self.time_packet, 'block':self.block,				'distance':0, 'Intensities': 0, 'channel':self.channel, 'return':0}			
						
		else:
			self.data_channel = None
			self.distan = [0,0,0]
			self.refle = [0,0,0]
		
	def xyz_return(self):
		
		if self.distan[0] >= self.filtered_range:#5.5e+6:
			if self.retorno == 0:
				self.data_channel['return'] = 1
				self.data_channel['distance'] = self.distan[0] * 0.00001
				self.data_channel['Intensities'] = self.refle[0]					
				#self.output = self.data_channel
				self.xyz_generate()
			elif (self.distan[0] != self.distan[1]) & (self.retorno == 1):
				self.data_channel['return'] = 2
				self.data_channel['distance'] = self.distan[1] * 0.00001
				self.data_channel['Intensities'] = self.refle[1]					
				#self.output = self.data_channel
				self.xyz_generate()
			elif (self.distan[0] != self.distan[2]) & (self.distan[1] != self.distan[2]) & (self.retorno == 2) :
				self.data_channel['return'] = 3
				self.data_channel['distance'] = self.distan[2] * 0.00001
				self.data_channel['Intensities'] = self.refle[2]					
				#self.output = self.data_channel
				self.xyz_generate()
			else:
				self.output = None
		else:
			self.output = None
	
	def xyz_generate(self):

		dis_cos = self.data_channel['distance'] * self.sin_and_cos_IDS[1][self.channel]
		self.data_channel['X'] = dis_cos * self.sin_and_cos_IDS[0][self.channel]
		self.data_channel['Y'] = dis_cos * self.sin_and_cos_IDS[1][self.channel]
		self.data_channel['Z'] = self.data_channel['distance'] * self.sin_and_cos_IDS[0][self.channel]
		self.output = self.data_channel

###############################################################################################################################################################


# In[13]:


class scann_offsets(object):
	"""docstring for ClassName"""
	def __init__(self, map_lid):
		self.map_lid = map_lid
		#offset_data_null = self.scann_nulls_data() 
		#offset_initial_data = self.scann_nulls_data_end()
		
	def scann_nulls_data(self):
		offset_data_null = 0
		while True:
			subset = self.map_lid[ offset_data_null : offset_data_null + 4]
			values = struct.unpack('!I', subset)
			values = int(values[0])
			if(values == 1975352983):
				break			
			else:
				offset_data_null+=4
		return offset_data_null

	def scann_nulls_data_end(self):
		offset_data_null = len(self.map_lid)
		while True:
			subset = self.map_lid[ offset_data_null-4 : offset_data_null]
			values = struct.unpack('!I', subset)
			values = int(values[0])
			if(values == 1975352983):
				offset_data_null-=4
				break			
			else:
				offset_data_null-=4

		if((((len(self.map_lid) -1)-offset_data_null)+1)==6632):
			return 0
		else:
			return ((len(self.map_lid) -1)-offset_data_null)+1
		
###############################################################################################################################################################


# In[14]:


class initial_packets(object):
	"""docstring for initial_packets"""
	def __init__(self, map_lid, data_null_initial, data_null_final, LineTimes):
		self.map_lid = map_lid
		self.LineTimes = LineTimes
		self.data_null_initial = data_null_initial
		self.offset_data_null = data_null_initial + data_null_final

		self.num_packets_lid = self.num_datagrams(self.map_lid,6632, self.offset_data_null)

		self.menu_print()
		self.check_times()
		self.initial_data()
		

	def __iter__(self):
		return self.initial_data()

	def num_datagrams(self, data,datagram_size,rest=0):
		longi = len(data)-rest
		if(longi%datagram_size) != 0:#------reemplaza (assert(longi%datagram_size== 0))
			print("Error in file length")
			print("longi%datagram_size =",longi%datagram_size)
			sys.exit()
		return int(longi / datagram_size)

	def menu_print(self):
		#--------------lid
		print("-----------------------------------------------------")
		print("Number of data packets in the .lid file : ",self.num_packets_lid)
		#---------------limite inferior por defecto
		lid_values = readLidBlock(self.map_lid, 0, 0, self.data_null_initial, 0)
		self.limite_inf_tiempo_lid = lid_values.values_data['tiempo']
		print("Initial time in the .lid file : ",self.limite_inf_tiempo_lid)
		
		#---------------limite superior por defecto
		lid_values = readLidBlock(self.map_lid, self.num_packets_lid-1, 0, self.data_null_initial, self.num_packets_lid-1)
		self.limite_sup_tiempo_lid = lid_values.values_data['tiempo']
		print("Final time in the .lid file : ",self.limite_sup_tiempo_lid)   
		print("-----------------------------------------------------")

	def check_times(self):
		for m in range(0,len(self.LineTimes)):
			if (self.LineTimes[m][0] > self.LineTimes[m][1]):
				backup = self.LineTimes[m][0]
				self.LineTimes[m][0] = self.LineTimes[m][1]
				self.LineTimes[m][1] = backup
			if(self.LineTimes[m][0] < self.limite_inf_tiempo_lid):
				print("###################################################################################")
				print("Tiempo ",self.LineTimes[m][0],"es inferrior al tiempo encontrado en el archivo .lid")
				print("###################################################################################")
				sys.exit()
			elif(self.LineTimes[m][1] > self.limite_sup_tiempo_lid):
				print("###################################################################################")
				print("Tiempo ",self.LineTimes[m][1],"es superior al tiempo encontrado en el archivo .lid")
				print("###################################################################################")
				sys.exit()
			
	def initial_data(self):
		self.inicios = [[0] * 2 for i in range(len(self.LineTimes))]
		for m in range(0,len(self.LineTimes)):
			#########################--busco paquete tiempo inferior--##################################################
			segundo_inicio_lid = round(self.limite_inf_tiempo_lid,6)
			segundo_inicio_lid = math.modf(self.LineTimes[m][0] - segundo_inicio_lid)
			cont_inicio_int = (segundo_inicio_lid[1]/0.0009293510229326785)
			cont_inicio_float = (segundo_inicio_lid[0]/0.0009293510229326785)
			cont_inicio_bot = int(cont_inicio_int+cont_inicio_float)
			cont_inicio = self.ajusta_point_lid(cont_inicio_bot,self.LineTimes[m][0])

			#########################--busco paquete tiempo superior--##################################################
			segundo_inicio_lid = round(self.limite_inf_tiempo_lid,6)
			segundo_fin_lid = math.modf(self.LineTimes[m][1] - segundo_inicio_lid)
			cont_fin_int = (segundo_fin_lid[1]/0.0009293510229326785)
			cont_fin_float = (segundo_fin_lid[0]/0.0009293510229326785)
			cont_fin_top = int(cont_fin_int+cont_fin_float)
			cont_fin = self.ajusta_point_lid(cont_fin_top,self.LineTimes[m][1])

			self.inicios[m][0] = int(cont_inicio)              #contador posicion incial lid
			self.inicios[m][1] = int(cont_fin)                 #contador posicion final lid
		return self.inicios

	def ajusta_point_lid(self, cont,tiempo_lid):
		i = cont
		posi = 0
		lid_values = readLidBlock(self.map_lid, i, 0, self.data_null_initial)#read_packets_lid(i,0,self.map_lid,self.offset_data_null)
		tiempo = lid_values.values_data['tiempo']
		if(tiempo_lid > tiempo):
			while True:
				if(tiempo_lid <= tiempo):
					posi = i
					break
				i += 1
				lid_values = readLidBlock(self.map_lid, i, 0, self.data_null_initial)
				tiempo = lid_values.values_data['tiempo']
		elif(tiempo_lid < tiempo):
			while True:
				if(tiempo_lid >= tiempo):
					posi = i
					break
				i -= 1
				lid_values = readLidBlock(self.map_lid, i, 0, self.data_null_initial)
				tiempo = lid_values.values_data['tiempo']
		return posi


# #### 3. Main method for getting Point Cloud information.

# In[15]:


def readLidarTimeIntervalDF(fileNameInput, iniTime, endTime):
    file_lid = fileNameInput
    lid_file = open(file_lid,'rb')
    lid_size = os.path.getsize(file_lid)
    mapa_lid = mmap.mmap(lid_file.fileno(),lid_size, access=mmap.ACCESS_READ)

    data_offsets = scann_offsets(mapa_lid)
    offset_initial_data = data_offsets.scann_nulls_data() 
    offset_final_data = data_offsets.scann_nulls_data_end() 


    datos_tabla = [[iniTime,endTime]]
    inicial_data =  initial_packets(mapa_lid, offset_initial_data, offset_final_data, datos_tabla)
    inicial_data = inicial_data.inicios


    colNames = ["PacketTime(SoW)", "Block","Azimuth","Channel","Return","Distance","Intensity","X","Y","Z"]
    data = []

    for line in range(0,len(inicial_data)):
        ots = readLidBlock(mapa_lid, inicial_data[line][0],0,offset_initial_data,inicial_data[line][1])
        for char in ots:
            if char != None:
                packet_time = float(char['packet_time'])  # Timestamp (s)
                block = int(char['block'] )               # Block Id
                azimuth = np.rad2deg(char['Azimuth'])     # Degrees (°)
                channel = int(char['channel'])            # Id. laser 0 - 7 (8 lasers for Quanergy M8)
                echo = int(char['return'])                # 1, 2, 3 for Quanergy M8
                distance = float(char['distance'])        # Meters
                intensity = int(char['Intensities'])      # 0 - 255 reflectivity
                x = float(char['X'])                      # Meters
                y = float(char['Y'])                      # Meters
                z = float(char['Z'])                      # Meters
                register = [packet_time, block, azimuth, channel, echo, distance, intensity, x, y, z]
                data.append(register.copy())
    # Create the pandas DataFrame 
    df = pd.DataFrame(data, columns = colNames) 
    
    return df


# #### 4.  Example:

# In[10]:


# Comment when exporting to .py
'''
fileNameInput = 'Data_samples/Data1_Lidar.lid'
iniTime = 146890.0
endTime = 146990.0

df = readLidarTimeIntervalDF(fileNameInput, iniTime, endTime)
df.head()
'''
