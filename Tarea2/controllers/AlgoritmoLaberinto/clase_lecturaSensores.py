from controller import DistanceSensor, LightSensor, Gyro, Accelerometer, Camera, LED
import time
import math

class SensoresActuadores:
    
    def __init__(self, robot, timestep):
        self.robot = robot
        self.timestep = timestep
        
        # ---------------------------------------------------------
        # 1. SENSORES DE PROXIMIDAD (Infrarrojos - ps0 a ps7)
        # ---------------------------------------------------------
        self.nombres_proximidad = [f'ps{i}' for i in range(8)]
        self.sensores_proximidad = {}
        for nombre in self.nombres_proximidad:
            self.sensores_proximidad[nombre] = self.robot.getDevice(nombre)
            self.sensores_proximidad[nombre].enable(self.timestep)
            
        # ---------------------------------------------------------
        # 2. SENSORES DE LUZ (ls0 a ls7)
        # ---------------------------------------------------------
        self.nombres_luz = [f'ls{i}' for i in range(8)]
        self.sensores_luz = {}
        for nombre in self.nombres_luz:
            self.sensores_luz[nombre] = self.robot.getDevice(nombre)
            self.sensores_luz[nombre].enable(self.timestep)

        # ---------------------------------------------------------
        # 3. GIROSCOPIO Y ACELERÓMETRO
        # ---------------------------------------------------------
        self.giroscopio = self.robot.getDevice('gyro')
        self.giroscopio.enable(self.timestep)
        
        self.acelerometro = self.robot.getDevice('accelerometer')
        self.acelerometro.enable(self.timestep)
        
        # Lectura inicial para detección de choques
        self.aceleracion_anterior = self.acelerometro.getValues()

        # ---------------------------------------------------------
        # 4. CÁMARA
        # ---------------------------------------------------------
        self.camera = self.robot.getDevice('camera')
        self.camera.enable(self.timestep)

        # ---------------------------------------------------------
        # 5. ACTUADORES: LEDs (led0 a led9)
        # ---------------------------------------------------------
        # El e-puck tiene 10 LEDs (0-7 alrededor, 8 el frontal, 9 el de cuerpo entero)
        self.nombres_leds = [f'led{i}' for i in range(10)]
        self.leds = {}
        for nombre in self.nombres_leds:
            self.leds[nombre] = self.robot.getDevice(nombre)
            

    # Dentro de tu clase SensoresActuadores:
    def actualizar_camara(self):
        image = self.camera.getImage()   
        return self.camera.getImage()

    # Devuelve un diccionario con los valores actuales de los 8 sensores IR.
    # Nota: Valores altos significan que el obstáculo está MUY CERCA (ej. > 70 o 80)

    def obtener_proximidad(self):
        return {nombre: sensor.getValue() for nombre, sensor in self.sensores_proximidad.items()}

    #Devuelve un diccionario con la intensidad de luz medida por los 8 sensores.
    def obtener_luz(self):
        return {nombre: sensor.getValue() for nombre, sensor in self.sensores_luz.items()}
    
    #Devuelve la velocidad angular en los tres ejes [X, Y, Z]. Z es la rotación sobre el suelo.
    def obtener_orientacion_giro(self):
        return self.giroscopio.getValues()

    #Devuelve las fuerzas G o aceleración en [X, Y, Z]. Útil para detectar choques bruscos.
    def obtener_aceleracion(self):
        return self.acelerometro.getValues()
        
        
    #Detecta un choque mediante un cambio brusco en la aceleración.
  
    def detectar_choque(self):
    
        actual = self.acelerometro.getValues()
    
        dx = actual[0] - self.aceleracion_anterior[0]
        dy = actual[1] - self.aceleracion_anterior[1]
        dz = actual[2] - self.aceleracion_anterior[2]
    
        impacto = math.sqrt(dx**2 + dy**2 + dz**2) if math.sqrt(dx**2 + dy**2 + dz**2) > 0 else 0
    
        self.aceleracion_anterior = actual
        
        if impacto > 0.5:
            print(f"[UMBRAL IMPACTO] {impacto:.3f} ")
    
        return impacto
        
        
    def mostrar_cambio_aceleracion(self):
        actual = self.acelerometro.getValues()
    
        dx = self.aceleracion_anterior[0]
        dy = self.aceleracion_anterior[1]
        dz = self.aceleracion_anterior[2]
    
        print(f"[ACELERACION] X={dx:.3f}  Y={dy:.3f}  Z={dz:.3f}")
    
        self.aceleracion_anterior = actual
    
        return dx, dy, dz

    #Funciones para mostrar el estado del robot E-puck 
    
    #Enciende un LED específico (0 al 9).
    def encender_led(self, numero_led):
        if 0 <= numero_led <= 9:
            self.leds[f'led{numero_led}'].set(1)
            
    #Apaga un LED específico (0 al 9).
    def apagar_led(self, numero_led):
        if 0 <= numero_led <= 9:
            self.leds[f'led{numero_led}'].set(0)
            
    #Activa un juego de leds que muestra un estado del robot

    def fijar_estado_leds(self, modo):
        for led in self.leds.values():
            led.set(0)
        
        # Encender LEDs de adelante
        if modo == "AVANCE":
            for i in [0,1,7]: self.encender_led(i)
            
        # Encender LEDs laterales para indicar giro a la derecha
        elif modo == "RETROCESO":
            for i in [3,4]: self.encender_led(i)
         
         # Encender LEDs laterales para indicar giro a la derecha
        elif modo == "GIRODER":
            for i in [1, 2, 3]: self.encender_led(i)
         
        # Encender LEDs laterales para indicar giro a la derecha
        elif modo == "GIROIZQ":
            for i in [4,5,6]: self.encender_led(i)
        
        # Encender los led frontales     
        elif modo == "ERROR_CHOQUE":
           for i in [6,7]: self.encender_led(i)
            