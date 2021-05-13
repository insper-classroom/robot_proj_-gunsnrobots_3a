#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
import numpy
import tf
import math
import cv2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from numpy import linalg
from tf import transformations
from tf import TransformerROS
import tf2_ros
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from sensor_msgs.msg import LaserScan
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Header
import os

import projeto_utils

ranges = None
minv = 0
maxv = 10
angulo = 0
distancia = 10
const_angulo = True


bridge = CvBridge()

def quart_to_euler(orientacao):
    """
    Converter quart. para euler (XYZ)
    Retorna apenas o Yaw (wz)
    """
    r = R.from_quat(orientacao)
    wx, wy, wz = (r.as_euler('xyz', degrees=True))

    return wz

## ROS
def mypose(msg):
    """
    Recebe a Leitura da Odometria.
    Para esta aplicacao, apenas a orientacao esta sendo usada
    """
    x = msg.pose.pose.orientation.x
    y = msg.pose.pose.orientation.y
    z = msg.pose.pose.orientation.z
    w = msg.pose.pose.orientation.w

    orientacao_robo = [[x,y,z,w]]

def scaneou(dado):
    """
    Rebe a Leitura do Lidar
    Para esta aplicacao, apenas a menor distancia esta sendo usada
    """
    global distancia
    
    ranges = np.array(dado.ranges).round(decimals=2)
    distancia = ranges[0]
            

def recebe_odometria(data):
    global angulo

    quat = data.pose.pose.orientation
    lista = [quat.x, quat.y, quat.z, quat.w]
    angulos = np.degrees(transformations.euler_from_quaternion(lista))    
    
    angulo = round(angulos[2], 2)
    if angulo < 0:
        angulo += 360

    print(angulo)

## Variáveis novas criadas pelo gabarito

centro_yellow = (320,240)
frame = 0
skip = 3
m = 0
angle_yellow = 0 # angulo com a vertical

low = projeto_utils.low
high = projeto_utils.high

centro_caixa = (320, 240)

## 

# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    global centro_yellow
    global m
    global angle_yellow

    ### 
    ## Vamos fazer o gabarito para a caixa azul, que era mais distante 

    try:
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        ##
        copia = cv_image.copy() # se precisar usar no while

        if frame%skip==0: # contamos a cada skip frames
            mask = projeto_utils.filter_color(copia, low, high)                
            img, centro_yellow  =  projeto_utils.center_of_mass_region(mask, 200, 300, mask.shape[1], mask.shape[0])  

            saida_bgr, m, h = projeto_utils.ajuste_linear_grafico_x_fy(mask)

            ang = math.atan(m)
            ang_deg = math.degrees(ang)

            angle_yellow = ang_deg

            projeto_utils.texto(cv_image, f"Distancia obstaculo: {distancia}", (15,50), color=(0,0,255))
            
            cv2.imshow("Camera", cv_image)

        cv2.waitKey(1)
    except CvBridgeError as e:
        print('ex', e)


if __name__=="__main__":

    rospy.init_node("projeto")

    topico_imagem = "/camera/image/compressed"
    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
    cmd_vel = velocidade_saida

    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
    pose_sub = rospy.Subscriber('/odom', Odometry , mypose)
    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    ref_odometria = rospy.Subscriber("/odom", Odometry, recebe_odometria)
    
    zero = Twist(Vector3(0,0,0), Vector3(0,0,0))         


    x = 0
    tol_centro = 10 # tolerancia de fuga do centro
    tol_ang = 15 # tolerancia do angulo

    c_img = (320,240) # Centro da imagem  que ao todo é 640 x 480

    v_slow = 0.3
    v_rapido = 0.85
    w_slow = 0.2
    w_rapido = 0.75

    
    INICIAL= -1
    AVANCA = 0
    AVANCA_RAPIDO = 1
    ALINHA = 2
    MEIA_VOLTA = 3

    state = INICIAL

    def inicial():
        # Ainda sem uma ação específica
        pass

    def avanca():
        vel = Twist(Vector3(v_slow,0,0), Vector3(0,0,0)) 
        cmd_vel.publish(vel) 

    def avanca_rapido():
        vel = Twist(Vector3(v_rapido,0,0), Vector3(0,0,0))         
        cmd_vel.publish(vel)

    def alinha():
        delta_x = c_img[x] - centro_yellow[x]
        max_delta = 150.0
        w = (delta_x/max_delta)*w_rapido
        vel = Twist(Vector3(v_slow,0,0), Vector3(0,0,w)) 
        cmd_vel.publish(vel)        

    def terminou():
        zero = Twist(Vector3(0,0,0), Vector3(0,0,0))         
        cmd_vel.publish(zero)
        
    rodando = False
    angulo_final_calibrado = None
    angulo_final_original = None
    
    def meia_volta():
        global const_angulo
        global rodando
        global angulo_final_original
        global angulo_final_calibrado 
        
        rodando = True
        
        if const_angulo:
            angulo_final_original = angulo + 180
            if angulo_final_original > 360:
                angulo_final_calibrado = angulo_final_original - 360
            else:
                angulo_final_calibrado = angulo_final_original
            
            const_angulo = False
        
        if angulo_final_original > 360:
                
                if angulo > 180:
                    if angulo > angulo_final_calibrado:
                        vel = Twist(Vector3(0,0,0), Vector3(0,0,0.4))
                        cmd_vel.publish(vel)
                    else:
                        rodando = False
                        const_angulo = True
                else:
                    if angulo < angulo_final_calibrado:
                        vel = Twist(Vector3(0,0,0), Vector3(0,0,0.4))
                        cmd_vel.publish(vel)
                    else:
                        rodando = False
                        const_angulo = True
                    
        else:
            if angulo < angulo_final_calibrado:
                vel = Twist(Vector3(0,0,0), Vector3(0,0,0.4))
                cmd_vel.publish(vel)
            else:
                rodando = False
                const_angulo = True

    def dispatch():
        "Logica de determinar o proximo estado"
        global state
        
        if distancia < 1 or rodando:
            state = MEIA_VOLTA
            
        else:
            if c_img[x] - tol_centro < centro_yellow[x] < c_img[x] + tol_centro:
                state = AVANCA
                if - tol_ang < angle_yellow  < tol_ang:  # para angulos centrados na vertical, regressao de x = f(y) como está feito
                    state = AVANCA_RAPIDO
            else: 
                state = ALINHA        

    acoes = {INICIAL:inicial, AVANCA: avanca, AVANCA_RAPIDO: avanca_rapido, ALINHA: alinha, MEIA_VOLTA: meia_volta}


    r = rospy.Rate(200) 

    while not rospy.is_shutdown():
        os.system('clear')
        print("Estado: ", state)
        print("Angulo: ", angulo)
        print("Angulo Final Original: ", angulo_final_original)
        print("Angulo Final Calibrado: ", angulo_final_calibrado)
        print("Const", const_angulo)
        print("Rodando", rodando)
        acoes[state]()  # executa a funcão que está no dicionário
        dispatch()            
        r.sleep()