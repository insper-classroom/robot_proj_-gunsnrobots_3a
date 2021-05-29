#! /usr/bin/env python3
# -*- coding:utf-8 -*-

# importando bibliotecas e arquivos utlizados ######################
from __future__ import print_function, division
from numpy.lib.function_base import copy
from numpy.linalg.linalg import _matrix_rank_dispatcher
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
from std_msgs.msg import Float64
import os
import projeto_utils
import cv2.aruco as aruco
####################################################################

# inicializando variaveis ###########################
ranges = None
minv = 0
maxv = 10
angulo = 0
distancia = 10
const_angulo = True
bridge = CvBridge()
centro_yellow = (320,240)
frame = 0
skip = 3
m = 0
angle_yellow = 0 
low = projeto_utils.low
high = projeto_utils.high
centro_caixa = (320, 240)
creeper_vermelho = np.zeros((640, 480, 1), np.uint8)
creeper_verde = np.zeros((640, 480, 1), np.uint8)
creeper_azul = np.zeros((640, 480, 1), np.uint8)
dic_creepers = {}
matando = False
voltar_pista = True
id_encontrado = False
agarradinha = False
maior_contorno_area = 0
ids = 0
rodando = False
angulo_final_calibrado = None
angulo_final_original = None
marker_size  = 20
font = cv2.FONT_HERSHEY_PLAIN
scan_dist = 0
##################~~~~'_'~~~~~#######################
cor = input('Qual a cor do creeper?: ').lower()
id_to_find  = int(input('Qual o ID do creeper?: '))
#####################################################

#################################--CONFIG_ARUCO--##############################################
#--- Get the camera calibration path
calib_path  = "/home/borg/catkin_ws/src/robot21.1/ros/exemplos211/scripts/"
camera_matrix   = np.loadtxt(calib_path+'cameraMatrix_raspi.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_raspi.txt', delimiter=',')

#--- Define the aruco dictionary
aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters  = aruco.DetectorParameters_create()
parameters.minDistanceToBorder = 0
parameters.adaptiveThreshWinSizeMax = 1000
#######################################################################################


#funcao responsavel por retornar a distancia do laser fronteiro 
def scaneou(dado):
    """
    Rebe a Leitura do Lidar
    """
    global distancia
    
    ranges = np.array(dado.ranges).round(decimals=2)
    distancia = ranges[0]
            
#funcao responsavel por devolver o angulo atual do robo
def recebe_odometria(data):
    global angulo

    quat = data.pose.pose.orientation
    lista = [quat.x, quat.y, quat.z, quat.w]
    angulos = np.degrees(transformations.euler_from_quaternion(lista))    
    
    angulo = round(angulos[2], 2)
    if angulo < 0:
        angulo += 360

# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    global centro_yellow
    global m
    global angle_yellow
    global creeper_vermelho
    global creeper_verde
    global creeper_azul
    global dic_creepers
    global ids 
    global id_encontrado

    try:
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        ##
        copia = cv_image.copy() # se precisar usar no while

        if frame%skip==0: # contamos a cada skip frames
            mask = projeto_utils.filter_color(copia, low, high)                
            img, centro_yellow  =  projeto_utils.center_of_mass_region(mask, 200, 300, mask.shape[1], mask.shape[0])  

            #----------------------------------------- CREEPERS ----------------------------------------#
            creeper_vermelho, creeper_verde, creeper_azul  = projeto_utils.identifica_creeper(cv_image)
            dic_creepers = {
                        'azul':creeper_azul, 
                        'vermelho':creeper_vermelho,
                        'verde':creeper_verde
                        }
            #-------------------------------------------------------------------------------------------#



            #----------------------------------------- REGRESSAO ---------------------------------------#
            saida_bgr, m, h = projeto_utils.ajuste_linear_grafico_x_fy(mask)
            ang = math.atan(m)
            ang_deg = math.degrees(ang)
            angle_yellow = ang_deg
            #-------------------------------------------------------------------------------------------#



            #----------------------------------------- ARUCO -------------------------------------------# 
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            
            # verificando se o id encontrado é o desejado
            try:
                for id in ids:
                    if id_to_find == int(id[0]):
                        id_encontrado = True
            except:
                pass
            
            ######################################## ARUCO #############################################
            
            if ids is not None:
                ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
                rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

                #-- Desenha um retanculo e exibe Id do marker encontrado
                aruco.drawDetectedMarkers(cv_image, corners, ids) 
                aruco.drawAxis(cv_image, camera_matrix, camera_distortion, rvec, tvec, 1)

                #-- Print tvec vetor de tanslação em x y z
                str_position = "Marker x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0], tvec[1], tvec[2])
                print(str_position)
                cv2.putText(cv_image, str_position, (0, 100), font, 1, (0, 255, 0), 1, cv2.LINE_AA)

                #####################---- Distancia Euclidiana ----#####################
                # Calcula a distancia usando apenas a matriz tvec, matriz de tanslação
                # Pode usar qualquer uma das duas formas
                distance = np.sqrt(tvec[0]**2 + tvec[1]**2 + tvec[2]**2)
                distancenp = np.linalg.norm(tvec)

                #-- Print distance
                str_dist = "Dist aruco=%4.0f  dis.np=%4.0f"%(distance, distancenp)
                print(str_dist)
                cv2.putText(cv_image, str_dist, (0, 15), font, 1, (0, 255, 0), 1, cv2.LINE_AA)

                #####################---- Distancia pelo foco ----#####################

                # raspicam v2 focal legth 
                FOCAL_LENGTH = 3.6 #3.04
                # pixel por unidade de medida
                m = (camera_matrix[0][0]/FOCAL_LENGTH + camera_matrix[1][1]/FOCAL_LENGTH)/2
                # corners[0][0][0][0] = [ID][plano?][pos_corner(sentido horario)][0=valor_pos_x, 1=valor_pos_y]	
                pixel_length1 = math.sqrt(math.pow(corners[0][0][0][0] - corners[0][0][1][0], 2) + math.pow(corners[0][0][0][1] - corners[0][0][1][1], 2))
                pixel_length2 = math.sqrt(math.pow(corners[0][0][2][0] - corners[0][0][3][0], 2) + math.pow(corners[0][0][2][1] - corners[0][0][3][1], 2))
                pixlength = (pixel_length1+pixel_length2)/2
                dist = marker_size * FOCAL_LENGTH / (pixlength/m)

                #-- Print distancia focal
                str_distfocal = "Dist focal=%4.0f"%(dist)
                print(str_distfocal)
                cv2.putText(cv_image, str_distfocal, (0, 30), font, 1, (0, 255, 0), 1, cv2.LINE_AA)	


                ####################--------- desenha o cubo -----------#########################			
                m = marker_size/2
                pts = np.float32([[-m,m,m], [-m,-m,m], [m,-m,m], [m,m,m],[-m,m,0], [-m,-m,0], [m,-m,0], [m,m,0]])
                imgpts, _ = cv2.projectPoints(pts, rvec, tvec, camera_matrix, camera_distortion)
                imgpts = np.int32(imgpts).reshape(-1,2)
                cv_image = cv2.drawContours(cv_image, [imgpts[:4]],-1,(0,0,255),4)
                for i,j in zip(range(4),range(4,8)):
                    cv_image = cv2.line(cv_image, tuple(imgpts[i]), tuple(imgpts[j]),(0,0,255),4)
                cv_image = cv2.drawContours(cv_image, [imgpts[4:]],-1,(0,0,255),4)
            #-------------------------------------------------------------------------------------------#    
            
            projeto_utils.texto(cv_image, f"Distancia obstaculo: {distancia}", (15,50), color=(0,0,255))
            
            cv2.imshow("Camera", cv_image)
            cv2.imshow("Creeper", dic_creepers[cor])

        cv2.waitKey(1)
    except CvBridgeError as e:
        print('ex', e)


# execucao principal

if __name__=="__main__":
    
    # inicializacao topicos, publishers e subscribers
    
    rospy.init_node("projeto")

    topico_imagem = "/camera/image/compressed"
    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
    cmd_vel = velocidade_saida

    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
    pose_sub = rospy.Subscriber('/odom', Odometry , mypose)
    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    ref_odometria = rospy.Subscriber("/odom", Odometry, recebe_odometria)
    
    # garra
    ombro = rospy.Publisher("/joint1_position_controller/command", Float64, queue_size=1)
    garra = rospy.Publisher("/joint2_position_controller/command", Float64, queue_size=1)
    
    zero = Twist(Vector3(0,0,0), Vector3(0,0,0))         

    # variaveis a serem utilizadas
    x = 0
    tol_centro = 10 # tolerancia de fuga do centro
    tol_ang = 15 # tolerancia do angulo

    c_img = (320,240) # Centro da imagem  que ao todo é 640 x 480

    v_slow = 0.3
    v_rapido = 0.4
    w_slow = 0.2
    w_rapido = 0.75

    # constantes de estado
    
    INICIAL= -1
    AVANCA = 0
    AVANCA_RAPIDO = 1
    ALINHA = 2
    MEIA_VOLTA = 3
    KILL_CREEPER = 4

    state = INICIAL
    
    # funcoes de estado
    
    def inicial():
        # Ainda sem uma ação específica
        pass
    # robo avanca com velocidade normal
    def avanca():
        vel = Twist(Vector3(v_slow,0,0), Vector3(0,0,0)) 
        cmd_vel.publish(vel) 
        
    # robo avanca com velocidade rapida
    def avanca_rapido():
        vel = Twist(Vector3(v_rapido,0,0), Vector3(0,0,0))         
        cmd_vel.publish(vel)
        
    # robo alinha com a pista
    def alinha():
        delta_x = c_img[x] - centro_yellow[x]
        max_delta = 150.0
        w = (delta_x/max_delta)*w_rapido
        vel = Twist(Vector3(v_slow,0,0), Vector3(0,0,w)) 
        cmd_vel.publish(vel)        
        
    # robo gira 180 graus
    def meia_volta():
        global const_angulo
        global rodando
        global angulo_final_original
        global angulo_final_calibrado 
        global id_encontrado
        
        # agarra o creeper apenas se o id tiver sido encontrado
        if id_encontrado == True:
            garra.publish(0.0) # fecha a garra
            rospy.sleep(0.4) 
            ombro.publish(1.5) # levanta o braco
            
        rodando = True
        
        # ajuste do angulo 
        if const_angulo:
            angulo_final_original = angulo + 180
            if angulo_final_original > 360:
                angulo_final_calibrado = angulo_final_original - 360
            else:
                angulo_final_calibrado = angulo_final_original
            
            const_angulo = False
            
        # logica da rotacao
        if angulo_final_original > 360:
                
                if angulo > 180:
                    if angulo > angulo_final_calibrado:
                        vel = Twist(Vector3(0,0,0), Vector3(0,0,0.4))
                        cmd_vel.publish(vel)
                    else:
                        rodando = False
                        const_angulo = True
                        id_encontrado = False
                else:
                    if angulo < angulo_final_calibrado:
                        vel = Twist(Vector3(0,0,0), Vector3(0,0,0.4))
                        cmd_vel.publish(vel)
                    else:
                        rodando = False
                        const_angulo = True
                        id_encontrado = False
                    
        else:
            if angulo < angulo_final_calibrado:
                vel = Twist(Vector3(0,0,0), Vector3(0,0,0.4))
                cmd_vel.publish(vel)
            else:
                rodando = False
                const_angulo = True
                id_encontrado = False
    
    # robo se aproxima do creeper, abaixando o ombro e abrindo a garra
    def kill_creeper():

        global matando
        global maior_contorno_area        
        
        if distancia < 0.18:
            matando = False
            
        else:
            matando = True
            ombro.publish(-0.5) # sobe o braco
            garra.publish(-1.0) # abre a garra
        
        # robo se alinha ao centro de massa do creeper da cor alvo
        def centraliza(media, centro, maior_contorno_area):
            if media is not None:
                if len(media) != 0:
                    print("DISTANCIA: " + str(distancia))
                    if (media[0] > centro[0] - 10 and media[0] < centro[0] + 10):
                        vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0))
                    else:
                        if (media[0] > centro[0]):
                            vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.1))
                        if (media[0] < centro[0]):
                            vel = Twist(Vector3(0,0,0), Vector3(0,0,0.1))
                    cmd_vel.publish(vel)
        
        media, centro, maior_contorno_area = projeto_utils.area_creeper(dic_creepers[cor])
        centraliza(media, centro, maior_contorno_area)

    # funcao responsavel por controlar a LLogica de determinar o proximo estado

    def dispatch():
        
        global state
        global voltar_pista
        
        # entra no estado MEIA_VOLTA
        if (distancia < 0.5 and state != KILL_CREEPER) or (distancia < 0.18 and state == KILL_CREEPER) or rodando:
            state = MEIA_VOLTA
    
        # entra no estado KILL_CREEPER
        elif (1600 < maior_contorno_area < 3000 or matando) and id_encontrado:
            state = KILL_CREEPER
            
        # verifica o alinhamento com a pista para decidir se avanca normal, rapido ou alinha
        else:
            if c_img[x] - tol_centro < centro_yellow[x] < c_img[x] + tol_centro:
                state = AVANCA
                if - tol_ang < angle_yellow  < tol_ang:  # para angulos centrados na vertical, regressao de x = f(y) como está feito
                    state = AVANCA_RAPIDO
            else: 
                state = ALINHA        

    # dicionario de estados 
    
    acoes = {
            INICIAL:inicial, 
            AVANCA: avanca, 
            AVANCA_RAPIDO: avanca_rapido, 
            ALINHA: alinha, 
            MEIA_VOLTA: meia_volta,
            KILL_CREEPER: kill_creeper
            }

    r = rospy.Rate(200) 

    # loop principal
    while not rospy.is_shutdown(): 
        try:
            media, centro, maior_contorno_area = projeto_utils.area_creeper(dic_creepers[cor])
        except:
            pass 
        print("Estado: ", state)
        print("Encontrou ID: ", id_encontrado)
        print("Angulo: ", angulo)
        print("Angulo Final Original: ", angulo_final_original)
        print("Angulo Final Calibrado: ", angulo_final_calibrado)
        print("Rodando", rodando)
        print('Matando:', matando)
        acoes[state]()  # executa a funcão que está no dicionário de acordo com o estado
        dispatch()            
        r.sleep()