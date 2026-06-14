from controller import Motor
import math

# Esta clase se utiliza para poder controlar el movimiento del robot
class Movimiento:

    def __init__(self, robot, timestep, velocidad_maxima=2):
        self.robot = robot
        self.timestep = timestep
        self.velocidad_maxima = velocidad_maxima
        
        # Obtenemos el control de los motores
        self.motor_izquierdo = self.robot.getDevice('left wheel motor') 
        self.motor_derecho = self.robot.getDevice('right wheel motor') 
        
        # Configuramos los motores en velocidad continua
        self.motor_izquierdo.setPosition(float('inf')) 
        self.motor_derecho.setPosition(float('inf')) 
        
        # Obtener y activar los sensores de posición (encoders)
        self.encoder_izquierdo = self.robot.getDevice('left wheel sensor')
        self.encoder_derecho = self.robot.getDevice('right wheel sensor')
        self.encoder_izquierdo.enable(self.timestep)
        self.encoder_derecho.enable(self.timestep)
        
        
        self.radio_rueda = 0.0205
        
        
        #obtener y activar el giroscopio
        self.gyro = self.robot.getDevice("gyro")
        self.gyro.enable(self.timestep)
        
        #Para obtener las datos de nuestro robot
        self.robot_node = robot.getSelf()
        
        self.detenerse()

    # Esta funcion permite que ambas ruedas giren a la misma velocidad hacia adelante
    def avanzar(self, velocidad=None):
        vel = velocidad if velocidad is not None else self.velocidad_maxima
        self.motor_izquierdo.setVelocity(vel)
        self.motor_derecho.setVelocity(vel)
        print("[START] En marcha")
        
        
    # Esta funcion permite que ambas ruedas giren a la misma velocidad hacia atras
    def retroceder(self, velocidad=None):
        vel = velocidad if velocidad is not None else self.velocidad_maxima
        self.motor_izquierdo.setVelocity(-vel)
        self.motor_derecho.setVelocity(-vel)
        print("[START] En marcha")


    # Esta funcion hace que los motores se detengan
    def detenerse(self):
        self.motor_izquierdo.setVelocity(0.0)
        self.motor_derecho.setVelocity(0.0)
        print("[STOP] En reposo")
        
    def girarEnBucle(self,vel=2): 
        velocidad = vel
        self.motor_izquierdo.setVelocity(velocidad)
        self.motor_derecho.setVelocity(-velocidad)
    
    
    def girar_angulo(self, angulo_grados, direccion="derecha"):
    
        # Converti el angulo que se mide del giroscopio a radianes
        angulo_objetivo = math.radians(angulo_grados)
    
        # Velocidades de giro
        velocidad = 2.0
    
        if direccion == "derecha":
            self.motor_izquierdo.setVelocity(velocidad)
            self.motor_derecho.setVelocity(-velocidad)
            signo = -1
        else:
            self.motor_izquierdo.setVelocity(-velocidad)
            self.motor_derecho.setVelocity(velocidad)
            signo = 1
    
        # Variables para integración
        angulo_actual = 0.0
        tiempo_anterior = self.robot.getTime()
    
        # Bucle principal
        while self.robot.step(self.timestep) != -1:
    
            # Tiempo actual
            tiempo_actual = self.robot.getTime()
            dt = tiempo_actual - tiempo_anterior
            tiempo_anterior = tiempo_actual
    
            # Leer giroscopio
            gyro_values = self.gyro.getValues()
    
            # Eje Z es la rotación horizontal
            velocidad_angular = gyro_values[2]
    
            # Integrar velocidad angular
            angulo_actual += velocidad_angular * dt * signo
    
            # Verificar si alcanzó el ángulo
            if abs(angulo_actual) >= angulo_objetivo:
                break

        # Detener robot
        self.detenerse();
    

    # Hace que el robot avance o retroceda una distancia exacta en metros.
    # distancia_metros: positivo para avanzar, negativo para retroceder.  
    def trasladar_distancia(self, distancia_metros, vel=2):
        # 1. Calcular cuántos radianes debe girar la rueda para recorrer esa distancia
        radianes_objetivo = distancia_metros / self.radio_rueda
        
        # 2. Leer la posición inicial actual de los encoders
        pos_inicial_izq = self.encoder_izquierdo.getValue()
        
        # 3. Configurar la dirección del movimiento usando la velocidad máxima configurada
        velocidad =vel if distancia_metros > 0 else -vel
        self.motor_izquierdo.setVelocity(velocidad)
        self.motor_derecho.setVelocity(velocidad)
    
        # 4. Bucle de control interno: Esperar a que las ruedas giren los radianes calculados
        while self.robot.step(self.timestep) != -1:
            act_izq = self.encoder_izquierdo.getValue()
            
            # Calculamos el progreso absoluto
            if abs(act_izq - pos_inicial_izq) >= abs(radianes_objetivo):
                break # Objetivo alcanzado
                
        self.detenerse();
        
    
    #Esta funcion permite que el robot se detenga los segundos configurados
    def esperar(self, segundos):
            
        tiempo_inicio = self.robot.getTime()
        
        while self.robot.step(self.timestep) != -1:
            tiempo_actual = self.robot.getTime()
            if (tiempo_actual - tiempo_inicio) >= segundos:
                break


    #Con esta funcion verificamos la posicion actual de nuestro robot
    def posicion(self):
        pos = self.robot_node.getPosition()
    
        print(
            f"[POSICION] X={pos[0]:.3f} "
            f"Y={pos[1]:.3f} "
        )
    
        return pos
        
        
    #Esta función sirve para verificar si el robot ya llegó a una posición 
    #objetivo (meta) dentro de un margen de error llamado tolerancia.  
    def distancia_meta(self, meta_x, meta_y, tolerancia=0.15):
    
        pos = self.posicion()
    
        distancia = math.sqrt(
            (meta_x - pos[0])**2 +
            (meta_y - pos[1])**2
        )
    
        print(f"[OBJETIVO] Distancia: {distancia:.3f}")
    
        return distancia
    
    
    