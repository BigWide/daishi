#!/usr/bin/env python
# --*-- coding:utf-8 --*--
import rospy
import math
from daishi.msg import daishi     
from daishi.msg import pos_all_x  

class SubscribeAndPublish:
  
  def __init__(self):
    self.sub = rospy.Subscriber('daishi_info', daishi, self.callback)
    self.pub = rospy.Publisher('pos/allx', pos_all_x,queue_size=1)
    self.dataofpos_all_x = pos_all_x()
  def callback(self, data):
      ##1. transfer latitude,longitude and altitude to local position dn and dw
      
      point = [data.GpsPosLat,data.GpsPosLon,data.GpsPosAlt,data.GpsAntHeading]
      L = point[0]
      lamda = point[1]
      h = point[2]
      RE = R0/(math.sqrt(1 - e*e*math.sin(L*math.pi/180)*math.sin(L*math.pi/180)))
      x = (RE + h)*math.cos(L*math.pi/180)*math.cos(lamda*math.pi/180)
      y = (RE + h)*math.cos(L*math.pi/180)*math.sin(lamda*math.pi/180)
      z = ((1 - e*e)*RE + h)*math.sin(L*math.pi/180)
    
      dx = x - x0
      dy = y - y0
      dz = z - z0

      dn = -math.sin(L*math.pi/180)*math.cos(lamda*math.pi/180)*dx - math.sin(L*math.pi/180)*math.sin(lamda*math.pi/180)*dy + math.cos(L*math.pi/180)*dz
      de = -math.sin(lamda*math.pi/180)*dx + math.cos(lamda*math.pi/180)*dy
      dd = -math.cos(L*math.pi/180)*math.cos(lamda*math.pi/180)*dx - math.cos(L*math.pi/180)*math.sin(lamda*math.pi/180)*dy - math.sin(L*math.pi/180)*dz

      dn = dn
      dw = -de
      
    
      
      self.dataofpos_all_x.latitude=point[0]
      self.dataofpos_all_x.longitude=point[1]
      self.dataofpos_all_x.altitude=point[2]
      self.dataofpos_all_x.x = dn
      self.dataofpos_all_x.y = dw
      self.dataofpos_all_x.heading = point[3]
      self.dataofpos_all_x.speed = data.Speed2D
      #self.dataofpos_all_x.header.stamp=rospy.Time.now()
      
      
  def loop(self):
     rate=rospy.Rate(40)
     while not rospy.is_shutdown():
      self.dataofpos_all_x.header.stamp=rospy.Time.now()
      self.pub.publish(self.dataofpos_all_x)
      rate.sleep()

if __name__ == '__main__':
    R0 = 6378137.0
    e = 0.0818191908425

    L0 = 31.029763683
    lamda0 = 121.4423159583
    #L0 = 31.5870902
    #lamda0 = 120.7709885
    hb = 0
    RE0 = R0/(math.sqrt(1 - e*e*math.sin(L0*math.pi/180)*math.sin(L0*math.pi/180)))
    x0 = (RE0 + hb)*math.cos(L0*math.pi/180)*math.cos(lamda0*math.pi/180)
    y0 = (RE0 + hb)*math.cos(L0*math.pi/180)*math.sin(lamda0*math.pi/180)
    z0 = ((1 - e*e)*RE0 + hb)*math.sin(L0*math.pi/180)
    
    rospy.init_node('trans')
    ic = SubscribeAndPublish()
    ic.loop()
    rospy.spin()  
