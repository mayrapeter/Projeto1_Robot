#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cormodule
from math import pi
import visao_module
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt8


bridge = CvBridge()

cv_image = None
media = []
centro = []
atraso = 0.5E9 # 1 segundo e meio. Em nanossegundos
distancia = None


area = 0.0 # Variavel com a area do maior contorno

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
	# print("frame")
	global cv_image
	global media
	global centro


	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime # calcula o lag
	delay = lag.nsecs
	#print("delay ", "{:.3f}".format(delay/1.0E9))
	if delay > atraso and check_delay==True:
		print("Descartando por causa do delay do frame:", delay)
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		media, centro, area =  cormodule.identifica_cor(cv_image)
		



		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)






def scaneou(dado):
	global distancia	
	# print("Faixa valida: ", dado.range_min , " - ", dado.range_max )	
	#print(np.array(dado.ranges).round(decimals=2)[0])
	distancia = np.array(dado.ranges).round(decimals=2)

	distancia[distancia < 0.0001] = 999999
	# print(distancia)

	
if __name__=="__main__":
	rospy.init_node("cor")

	#topico_imagem = "/kamera"
	topico_imagem = "/raspicam_node/image/compressed"

	
	# Para renomear a *webcam*
	# 
	# 	rosrun topic_tools relay  /cv_camera/image_raw/compressed /kamera
	# 
	# Para renomear a câmera simulada do Gazebo
	# 
	# 	rosrun topic_tools relay  /camera/rgb/image_raw/compressed /kamera
	# 
	# Para renomear a câmera da Raspberry
	# 
	# 	rosrun topic_tools relay /raspicam_node/image/compressed /kamera
	# 

	recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
	#print("Usando ", topico_imagem)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

	x = 30

	try:
 
		while not rospy.is_shutdown():
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			velocidade_saida.publish(vel)

			

			#seguindo a caixa

			if len(media) != 0 and len(centro) != 0 and area > 50000:

				if media[0] > centro[1] + x:
					print('o')
					vel = Twist(Vector3(0.1,0,0), Vector3(0,0,-pi/5))
					velocidade_saida.publish(vel)
					rospy.sleep(0.1)
				elif media[0] < centro[1] - x:
					vel = Twist(Vector3(0.1,0,0), Vector3(0,0,pi/5))
					velocidade_saida.publish(vel)
					rospy.sleep(0.1)
				else:
					vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0))
					velocidade_saida.publish(vel)
					rospy.sleep(0.1)
				continue

			#reagindo ao laser
			if not distancia is None:
				if np.min(distancia) <=0.25:

					if 0 <= np.argmin(distancia) < (90):

						vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
						velocidade_saida.publish(vel)
						vel = Twist(Vector3(-0.1,0,0), Vector3(0,0,-0.3))
						velocidade_saida.publish(vel)
						rospy.sleep(2.0)
						vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
						velocidade_saida.publish(vel)
						rospy.sleep(1.0)
						distancia = None

					elif (90) <= np.argmin(distancia) < (180):

						vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
						velocidade_saida.publish(vel)
						vel = Twist(Vector3(0.1,0,0), Vector3(0,0,-0.3))
						velocidade_saida.publish(vel)
						rospy.sleep(2.0)
						vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
						velocidade_saida.publish(vel)
						rospy.sleep(1.0)
						distancia = None

					elif (180) <= np.argmin(distancia) < (270):

						vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
						velocidade_saida.publish(vel)
						vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0.3))
						velocidade_saida.publish(vel)
						rospy.sleep(2.0)
						vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
						velocidade_saida.publish(vel)
						rospy.sleep(1.0)
						distancia = None

					else:

						vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
						velocidade_saida.publish(vel)
						vel = Twist(Vector3(-0.1,0,0), Vector3(0,0,0.3))
						velocidade_saida.publish(vel)
						rospy.sleep(2.0)
						vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
						velocidade_saida.publish(vel)
						rospy.sleep(1.0)
						distancia = None



			rospy.sleep(0.09)




	except rospy.ROSInterruptException:
		print("Ocorreu uma exceção com o rospy")


