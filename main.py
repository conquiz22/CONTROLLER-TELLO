# PRIMER VERSION DRONE TELLO MANEJO AUTOMATICO
# CREADO POR CONQUIZ 09/10/2022

# LIBRERIAS
from djitellopy import Tello  # VERSION 2.4.0
import cv2  # VERSION 2.4.0
import pygame  # 2.1.3 DEV6
import numpy as np  # VERSION 1.23.3
import time

# ELEMENTOS DE CONFIGURACION
S = 180

FPS = 120  # POR DEFECTO 120
tamaño_ventana = [900, 600]
x_line_mitad = round(tamaño_ventana[0] / 2)
y_line_mitad = round(tamaño_ventana[1] / 2)

faceClassif = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

# VARIABLES Y ELEMENTOS DE CONTROL
Estado_control = 0
x_punto_medio = round(tamaño_ventana[0] / 2)
y_punto_medio = round(tamaño_ventana[1] / 2)

ERROR_ANTERIOR_X = 0
ERROR_ANTERIOR_Y = 0

PID_D_X_ANTERIOR = 0
ERROR_ANTERIOR = 0

Kp_x = 0.2
Ki_x = 0.005
Kd_x = 0.00

Kp_y = 1
Ki_y = 0
Kd_y = 0

def lineas_de_centro(frame):
    cv2.line(frame, (x_line_mitad, 0), (x_line_mitad, tamaño_ventana[1]), (255, 0, 0), 2)
    cv2.line(frame, (0, y_line_mitad), (tamaño_ventana[0], y_line_mitad), (255, 0, 0), 2)
    return frame


def reconocimiento_rostro(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = faceClassif.detectMultiScale(gray, 1.3, 5)[0:]
    x_drone = 0
    y_drone = 0
    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        x_drone = round(x + (w / 2))
        y_drone = round(y + (h / 2))
        cv2.line(frame, (x_drone, y_drone), (x_drone, y_line_mitad), (50, 100, 80), 2)
        cv2.line(frame, (x_drone, y_drone), (x_line_mitad, y_drone), (50, 100, 80), 2)

    return x_drone,y_drone

def control(x_leido,y_leido):
    global PID_D_X_ANTERIOR,ERROR_ANTERIOR
    Error_x = round (x_punto_medio - x_leido)
    Error_y = y_punto_medio - y_leido

    PID_P_X = Kp_x*Error_x
    PID_P_Y = Kp_y*Error_y

    PID_I_X = PID_D_X_ANTERIOR + (Ki_x * Error_x)

    PID_D_X_ANTERIOR = PID_I_X

    PID_D_X = Kd_x*(Error_x-ERROR_ANTERIOR)/(1000 // FPS)

    ERROR_ANTERIOR = Error_x

    PID_X = -round(PID_P_X + PID_I_X + PID_D_X)

    if(PID_X > 150):
        PID_X = 150
    if(PID_X < -150):
        PID_X = -150

    print(PID_X)

    return PID_X




class FrontEnd(object):

    def __init__(self):
        # CREA LA VENTANA DE PYGAME
        pygame.init()
        pygame.display.set_caption("CODIGO TELLO V2.1")
        self.screen = pygame.display.set_mode(tamaño_ventana)
        # INICIALIZA DRONE TELLO
        self.tello = Tello()
        # INICIALIZACION DE VARIABLES DE INICIO
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0
        self.speed = 10
        # DESACTIVAMOS EL USO DEL CONTROL TELLO
        self.send_rc_control = False

        # CREAR TEMPIRACION DE ACTUALIZACION
        pygame.time.set_timer(pygame.USEREVENT + 1, 1000 // FPS)

    def run(self):

        # CONECTAMOS EL DRONE A LA APLICACION

        self.tello.connect()
        self.tello.set_speed(self.speed)

        self.tello.streamoff()
        self.tello.streamon()
        # SE RESIVE LA IMAGENE DEL DRONE
        frame_read = self.tello.get_frame_read()

        should_stop = False

        while not should_stop:

            for event in pygame.event.get():
                if event.type == pygame.USEREVENT + 1:  # SE ACTAULZIA EL ESTADO DEL DRONE (TEMPORIZADOR)
                    self.update()
                elif event.type == pygame.QUIT:  # TERMINA TODO SI CERRAMOS LA VENTANA EMERGENTE DEL DRONE
                    should_stop = True
                elif event.type == pygame.KEYDOWN:  # SI EL EVENTO ES DE TIPO PRESIONAR TECLA UNA TECLA
                    if event.key == pygame.K_ESCAPE:  # TECLA ESPACIO PARA CERRAR VENTANA Y PROCESO
                        should_stop = True
                    else:
                        self.keydown(event.key)
                elif event.type == pygame.KEYUP:  # SI SE SUELTA LA TECLA ACTUALZIAR ESTADOS
                    self.keyup(event.key)

            if frame_read.stopped:
                break

            frame = frame_read.frame
            # battery n. 电池
            text = "Battery: {}%".format(self.tello.get_battery())
            text1 = "Est control : {}".format(Estado_control)
            if(Estado_control == 0):
                cv2.putText(frame, text, (5, 35 - 5), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            else:
                cv2.putText(frame, text, (5, 35 - 5), cv2.FONT_HERSHEY_SIMPLEX, 1, (222, 10, 255), 2)
            cv2.putText(frame, text1, (5, 70 - 5), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            frame = lineas_de_centro(frame)

            x,y = reconocimiento_rostro(frame)
            if(Estado_control == 1 ) and (x != 0):
                x_control = control(x,y)
                self.yaw_velocity = x_control
            if(Estado_control == 1) and (x == 0):
                self.yaw_velocity = 0


            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame = np.rot90(frame)
            frame = np.flipud(frame)

            frame = pygame.surfarray.make_surface(frame)
            self.screen.blit(frame, (0, 0))
            pygame.display.update()

            #time.sleep(1 / FPS)

        # Call it always before finishing. To deallocate resources.
        # 通常在结束前调用它以释放资源
        self.tello.end()

    def keydown(self, key):

        if key == pygame.K_UP:  # set forward velocity
            self.for_back_velocity = S
        elif key == pygame.K_DOWN:  # set backward velocity
            self.for_back_velocity = -S
        elif key == pygame.K_LEFT:  # set left velocity
            self.left_right_velocity = -S
        elif key == pygame.K_RIGHT:  # set right velocity
            self.left_right_velocity = S
        elif key == pygame.K_w:  # set up velocity
            self.up_down_velocity = S
        elif key == pygame.K_s:  # set down velocity
            self.up_down_velocity = -S
        elif key == pygame.K_a:  # set yaw counter clockwise velocity
            self.yaw_velocity = -S
        elif key == pygame.K_d:  # set yaw clockwise velocity
            self.yaw_velocity = S
        elif key == pygame.K_z:
            global Estado_control
            if (Estado_control == 1):
                Estado_control = 0
            else:
                Estado_control = 1

    def keyup(self, key):
        if key == pygame.K_UP or key == pygame.K_DOWN:  # set zero forward/backward velocity
            self.for_back_velocity = 0
        elif key == pygame.K_LEFT or key == pygame.K_RIGHT:  # set zero left/right velocity
            self.left_right_velocity = 0
        elif key == pygame.K_w or key == pygame.K_s:  # set zero up/down velocity
            self.up_down_velocity = 0
        elif key == pygame.K_a or key == pygame.K_d:  # set zero yaw velocity
            self.yaw_velocity = 0
        elif key == pygame.K_q:  # takeoff
            self.tello.takeoff()
            self.send_rc_control = True
        elif key == pygame.K_e:  # land
            not self.tello.land()
            self.send_rc_control = False

    # FUNCION ENCARGADA DE ENVIAR LOS NUEVOS ESTADOS AL DRONE
    def update(self):

        if self.send_rc_control:
            self.tello.send_rc_control(self.left_right_velocity, self.for_back_velocity,
                                       self.up_down_velocity, self.yaw_velocity)


def main():
    frontend = FrontEnd()

    # run frontend
    frontend.run()


if __name__ == '__main__':
    main()
