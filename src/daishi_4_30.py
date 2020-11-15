#!/usr/bin/env python
# --*-- coding:utf-8 --*--

import rospy
from ctypes import *
import time
import math
from geometry_msgs.msg import *
from std_msgs.msg import *
from daishi.msg import * #自定义消息编译在哪一功能包下还需确认
from vehiclestate.msg import veh_info

####定义数据帧####
class CAN_InitConfig(Structure):
    _fields_ = [('bMode',c_ubyte),('nBtrType',c_ubyte),('dwBtr',c_ubyte *4),('dwAccCode',c_uint),
                ('dwAccMask',c_uint),('nFilter',c_ubyte),('dwReserved',c_ubyte)]
class CAN_DataFrame(Structure):
    _fields_ = [('uTimeFlag',c_uint),('nSendType',c_ubyte),('bRemoteFlag',c_ubyte),
                ('bExternFlag',c_ubyte), 
                ('nDataLen',c_ubyte),('uID',c_uint),('arryData',c_ubyte *8)]


####设定端口属性####
config = CAN_InitConfig()
config.bMode = 0
config.nBtrType = 1
config.dwAccCode = 0
config.dwAccMask = 0xffffffff
config.nFilter  = 0
config.dwBtr[0] = 0x00
config.dwBtr[1] = 0x1C
config.dwBtr[2] = 0
config.dwBtr[3] = 0

####实例化收发数据帧#### 
rec1 = CAN_DataFrame()
reclist_vehicle = (CAN_DataFrame * 10)()
reclist_daishi = (CAN_DataFrame * 40)()

####实例化自定义消息####
daishi_info = daishi()
daishi_info.header.frame_id = "daishi_map"
daishi_info.header.stamp.secs = 0
daishi_info.sendtime.secs = 0
daishi_info.GpsAntHeading = 0.0
daishi_info.AngRateRawX = 0.0
daishi_info.AngRateRawY = 0.0
daishi_info.AngRateRawZ = 0.0
daishi_info.AccelRawX = 0.0
daishi_info.AccelRawY = 0.0
daishi_info.AccelRawZ = 0.0

daishi_info.GpsPosLon = 0.0
daishi_info.GpsPosLat = 0.0
daishi_info.GpsPosAlt = 0.0
daishi_info.HoriPosErr = 0.0
daishi_info.VertPosErr = 0.0
daishi_info.HDOP = 0
daishi_info.InsPosMode = 0
daishi_info.GpsNumSats = 0.0
daishi_info.PosLocalDown = 0.0
daishi_info.DateTime = ''
daishi_info.PosLat = 0.0
daishi_info.PosLon = 0.0
daishi_info.PosAlt = 0.0
daishi_info.VelNorth = 0.0
daishi_info.VelEast = 0.0
daishi_info.VelDown = 0.0
daishi_info.Speed2D = 0.0
daishi_info.VelForward = 0.0
daishi_info.VelLateral = 0.0

daishi_info.AccelX = 0.0
daishi_info.AccelY = 0.0
daishi_info.AccelZ = 0.0
daishi_info.AccelForward = 0.0
daishi_info.AccelLateral = 0.0
daishi_info.AccelDown = 0.0

daishi_info.AngleHeading = 0.0
daishi_info.AnglePitch = 0.0
daishi_info.AngleRoll = 0.0

daishi_info.AngRateX = 0.0
daishi_info.AngRateY = 0.0
daishi_info.AngRateZ = 0.0
daishi_info.AngRateForward = 0.0
daishi_info.AngRateLateral = 0.0
daishi_info.AngRateDown = 0.0

daishi_info.AngleTrack = 0.0
daishi_info.AngleSlip = 0.0
daishi_info.PosLocalNorth = 0.0
daishi_info.PosLocalEast = 0.0

daishi_info.AngAccelX = 0.0
daishi_info.AngAccelY = 0.0
daishi_info.AngAccelZ = 0.0
daishi_info.AngAccelForward = 0.0
daishi_info.AngAccelLateral = 0.0
daishi_info.AngAccelDown = 0.0

####定义数据帧处理函数####
def dataprocessIntel(rec):
	pStr0 = "{:0>8b}".format(rec.arryData[0])
	pStr1 = "{:0>8b}".format(rec.arryData[1])
	pStr2 = "{:0>8b}".format(rec.arryData[2])
	pStr3 = "{:0>8b}".format(rec.arryData[3])
	pStr4 = "{:0>8b}".format(rec.arryData[4])
	pStr5 = "{:0>8b}".format(rec.arryData[5])
	pStr6 = "{:0>8b}".format(rec.arryData[6])
	pStr7 = "{:0>8b}".format(rec.arryData[7])
	pSum = '%s%s%s%s%s%s%s%s' % (pStr7,pStr6,pStr5,pStr4,pStr3,pStr2,pStr1,pStr0)

	return pSum

def dataprocessMoto(rec):
    pStr0 = "{:0>8b}".format(rec.arryData[0])
    pStr1 = "{:0>8b}".format(rec.arryData[1])
    pStr2 = "{:0>8b}".format(rec.arryData[2])
    pStr3 = "{:0>8b}".format(rec.arryData[3])
    pStr4 = "{:0>8b}".format(rec.arryData[4])
    pStr5 = "{:0>8b}".format(rec.arryData[5])
    pStr6 = "{:0>8b}".format(rec.arryData[6])
    pStr7 = "{:0>8b}".format(rec.arryData[7])
    dpSum = '%s%s%s%s%s%s%s%s' % (pStr0, pStr1, pStr2, pStr3, pStr4, pStr5, pStr6, pStr7)

    return dpSum

####定义负数反码函数####
def complement(src):
	if src[0]=='0':
		return int(src,2)
	else:
		src=src[1:len(src)].replace('1','t').replace('0','1').replace('t','0')
		return -1*int(src,2)

####定义转译函数####
def parseData(reclist_daishi , reclist_daishi_len):
	for i in range(reclist_daishi_len):
		tmp = reclist_daishi[i]
		pSum = dataprocessIntel(tmp)
		daishi_info.header.stamp = rospy.Time.now()
		if tmp.uID==0x506:  #GpsAntAngles
			daishi_info.GpsAntHeading = complement(pSum[48:64])*0.01
		
		elif tmp.uID==0x505:   #AngRateRaw
			daishi_info.AngRateRawX = complement(pSum[48:64])*0.01
			daishi_info.AngRateRawY = complement(pSum[32:48])*0.01
			daishi_info.AngRateRawZ = complement(pSum[16:32])*0.01
		
		elif tmp.uID==0x504:   #AccelRaw
			daishi_info.AccelRawX = complement(pSum[48:64])*0.01
			daishi_info.AccelRawY = complement(pSum[32:48])*0.01
			daishi_info.AccelRawZ = complement(pSum[16:32])*0.01
		
		elif tmp.uID==0x502:  #GpsLatitudeLongtitude
			daishi_info.GpsPosLon = complement(pSum[0:32])*1e-7
			daishi_info.GpsPosLat = complement(pSum[32:64])*1e-7
		
		elif tmp.uID==0x503:  #GpsAltitude
			daishi_info.GpsPosAlt = complement(pSum[32:64])*0.001
		
		elif tmp.uID==0x501:  #PosError
			daishi_info.HoriPosErr = int(pSum[48:64],2)*0.001
			daishi_info.VertPosErr = int(pSum[32:48],2)*0.001
			daishi_info.HDOP = int(pSum[24:32],2)*0.1
		
		elif tmp.uID==0x500:  #InsStatus
			daishi_info.InsPosMode = int(pSum[56:64],2)
			daishi_info.GpsNumSats = int(pSum[48:56],2)
		
		elif tmp.uID==0x60d:  #PosLocalD
			daishi_info.PosLocalDown = complement(pSum[0:32]) *0.001
		
		elif tmp.uID==0x600: #DateTime
			Year = int(pSum[56:64],2)
			Century = int(pSum[48:56],2)
			Month = int(pSum[40:48],2)
			Day = int(pSum[32:40],2)
			HSecond = int(pSum[24:32],2)*0.01
			Second = int(pSum[16:24],2)
			Minute = int(pSum[8:16],2)
			Hour = int(pSum[0:8],2)        
			daishi_info.DateTime = "%s%s-%s-%s %s:%s:%f" % (Century,Year,Month,Day,Hour+8,Minute,float(Second)+HSecond)
	

		elif tmp.uID==0x601: #LatitudeLongitude
			daishi_info.PosLat = complement(pSum[32:64])*1e-7
			daishi_info.PosLon = complement(pSum[0:32])*1e-7
		
		elif tmp.uID==0x602: #Altitude
			daishi_info.PosAlt = complement(pSum[32:64])*0.001
		
		elif tmp.uID==0x603: #Velocity
			daishi_info.VelNorth = complement(pSum[48:64])*0.01
			daishi_info.VelEast = complement(pSum[32:48])*0.01
			daishi_info.VelDown = complement(pSum[16:32])*0.01
			daishi_info.Speed2D = int(pSum[0:16],2)*0.01
		
		elif tmp.uID==0x604: #VelocityLevel
			daishi_info.VelForward = complement(pSum[48:64])*0.01
			daishi_info.VelLateral = complement(pSum[32:48])*0.01
		
		elif tmp.uID==0x605: #AccelVehicle
			daishi_info.AccelX = complement(pSum[48:64])*0.01
			daishi_info.AccelY = complement(pSum[32:48])*0.01
			daishi_info.AccelZ = complement(pSum[16:32])*0.01
		        #print(complement(pSum[16:32])*0.01)
		elif tmp.uID==0x606: #AccelLevel
			daishi_info.AccelForward = complement(pSum[48:64])*0.01
			daishi_info.AccelLateral = complement(pSum[32:48])*0.01
			daishi_info.AccelDown = complement(pSum[16:32])*0.01
		
		elif tmp.uID==0x607: #HeadingPitchRoll
			daishi_info.AngleHeading = int(pSum[48:64],2)*0.01
			daishi_info.AnglePitch = complement(pSum[32:48])*0.01
			daishi_info.AngleRoll = complement(pSum[16:32])*0.01
		
		elif tmp.uID==0x608: #AngRateVehicle
	 		daishi_info.AngRateX = complement(pSum[48:64])*0.01
			daishi_info.AngRateY = complement(pSum[32:48])*0.01
			daishi_info.AngRateZ = complement(pSum[16:32])*0.01
		
		elif tmp.uID==0x609: #AngRateLevel
			daishi_info.AngRateForward = complement(pSum[48:64])*0.01
			daishi_info.AngRateLateral = complement(pSum[32:48])*0.01
			daishi_info.AngRateDown = complement(pSum[16:32])*0.01
		
		elif tmp.uID==0x60e: #TrackSlip
			daishi_info.AngleTrack = int(pSum[48:64],2)*0.01
			daishi_info.AngleSlip = complement(pSum[32:48])*0.01
		
		elif tmp.uID==0x60c: #PosLocalNE
			daishi_info.PosLocalNorth = complement(pSum[32:64])*0.001
			daishi_info.PosLocalEast = complement(pSum[0:32])*0.001
		
		elif tmp.uID==0x60a: #AngAccelVehicle
			daishi_info.AngAccelX = complement(pSum[48:64])*0.1
			daishi_info.AngAccelY = complement(pSum[32:48])*0.1
			daishi_info.AngAccelZ = complement(pSum[16:32])*0.1
		
		elif tmp.uID==0x60b: #AngAccelLevel
			daishi_info.AngAccelForward = complement(pSum[48:64])*0.1
			daishi_info.AngAccelLateral = complement(pSum[32:48])*0.1
			daishi_info.AngAccelDown = complement(pSum[16:32])*0.1
	
		else:
			pass




####定义发送函数####
def ds():

	#reccount1 = 0
	#reccount2 = 0

	pressure_to_p=0
	throttle_to_p=0
	plight=0
	steer_to_p=0
	veh_speed=0
	while not rospy.is_shutdown():
		reclen1 = clib.CAN_ChannelReceive(dwDeviceHandle, 0 , byref(reclist_daishi), 22, 20)##戴世
		reclen2 = clib.CAN_ChannelReceive(dwDeviceHandle, 1, byref(reclist_vehicle), 10, 10)####整车
		#reccount1 = reccount1 + reclen1
		#reccount2 = reccount2 + reclen2
		parseData(reclist_daishi , reclen1)
	
		for i in range(reclen2):
			if(reclist_vehicle[i].uID==0xF1):    #刹车信号
				#print('CAN 1 receive %d frames. This is BrakePedal info' % reccount)
				pSum=dataprocessMoto(reclist_vehicle[i])
				pres_bi=pSum[8:16]
				ppressure=int(pres_bi,2)
				pressure_to_p=ppressure*0.392157
				#print('pressure:',pressure_to_p)

			elif(reclist_vehicle[i].uID==0xC9):   #油门,车速信号
				#print('CAN 1 receive %d frames. This is Throttle info' % reccount)
				pSum = dataprocessMoto(reclist_vehicle[i])
				throttle_bi = pSum[48:56]
				vehspeed_bi=pSum[56:64]
				pthrottle=int(throttle_bi,2)
				veh_speed=int(vehspeed_bi,2)
				throttle_to_p=0.390625*pthrottle
				#print('throttle:', throttle_to_p)
				print('vehicle speed:', veh_speed)

			elif (reclist_vehicle[i].uID == 0x46A):  # 灯光信号
				#print('CAN 1 receive %d frames. This is Light info' % reccount)
				pSum = dataprocessMoto(reclist_vehicle[i])
				light_bi = pSum[33:35]
				#print(light_bi)
				plight=int(light_bi,2)
				#if (plight>0):
				#print('light',plight)

			elif (reclist_vehicle[i].uID == 0x1E5):  # 转向信号
				#print('CAN 1 receive %d frames. This is Steering info' % reccount)
				pSum = dataprocessMoto(reclist_vehicle[i])
				psteer1 = pSum[0:8]
				psteer2 = pSum[8:16]
				psteer_str=str(psteer1)+str(psteer2)
				#psteer_bi=int(psteer_str)
				psteer=int(psteer_str,2)
				steer_to_p=psteer*0.0625-2048

		v=veh_info()
		v.brake_percentage=pressure_to_p
		v.throttle=throttle_to_p
		v.light=plight
		v.steer=steer_to_p
		v.vehicle_speed=veh_speed
		info_pub.publish(v)

		#if reccount1%22==0:
			#daishi_info.sendtime = rospy.Time.now()
		daishi_pub.publish(daishi_info)


		


####主程序####
if __name__ == '__main__':
    
	rospy.init_node('daishi_talker',anonymous=True)
	clib = cdll.LoadLibrary('/home/jiaofei/catkin_ws/src/daishi/src/libCanCmd.so')

	dwDeviceHandle = clib.CAN_DeviceOpen(2,1,0)

	if dwDeviceHandle == 0:
		print('open device error')
	else:
		print('device opened')
   
	if clib.CAN_ChannelStart(dwDeviceHandle, 0, byref(config)) != 1 :
		print('Start CAN 0 error') 
	else:
		print('CAN 0 Started')
     
	if clib.CAN_ChannelStart(dwDeviceHandle, 1, byref(config)) != 1 :
		print('Start CAN 1 error') 
	else:
		print('CAN 1 Started')


	daishi_pub = rospy.Publisher('daishi_info',daishi,queue_size=10)
	info_pub = rospy.Publisher('Vehicle_Info',veh_info,queue_size=64)


	ds()
	clib.CAN_ChannelStop(dwDeviceHandle,0)
	clib.CAN_ChannelStop(dwDeviceHandle,1)
	clib.CAN_DeviceClose(dwDeviceHandle)
