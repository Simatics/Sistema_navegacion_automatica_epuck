from controller import Supervisor, Camera
from clase_movimiento import Movimiento # Modulo para controlar el movimiento del robot
from clase_lecturaSensores import SensoresActuadores # Modulo para la lectura de los sensores y los leds

# Creamos la instancia del robot
robot = Supervisor()

timestep = int(robot.getBasicTimeStep())
robotMovimiento = Movimiento(robot, timestep, velocidad_maxima = 5)
leerEntorno = SensoresActuadores(robot, timestep)
contador = 0
orientacion = True

#Variables para realizar un reporte final
tiempo_inicio = robot.getTime()
total_choques = 0
giros_izquierda = 0
giros_derecha = 0

# Umbral para detectar pared
UMBRAL_PARED = 100.0  
UMBRAL_PARED_LATERAL = 80

# avance antes de cada evaluacion de los sensores
DISTANCIA_PASO = 0.06  

while robot.step(timestep) != -1:
    
    # Leemos las variables del entorno con la clase leerEntorno
    distancias = leerEntorno.obtener_proximidad()
    luces = leerEntorno.obtener_luz()
    datos_giro = leerEntorno.obtener_orientacion_giro()
    
    # Mantener activa la ventana visual de la cámara
    leerEntorno.actualizar_camara()
    
    # Mapeo los sensores del epuk para saber en cual pared se encuentra
    pared_frente =(
                    (distancias['ps0'] > UMBRAL_PARED) or 
                    (distancias['ps7'] > UMBRAL_PARED)
                  )
        
    pared_derecha =( 
                     #(distancias['ps1'] > UMBRAL_PARED_LATERAL) or 
                     (distancias['ps2'] > UMBRAL_PARED_LATERAL)
                    )

    pared_izquierda = (
                        (distancias['ps5'] > UMBRAL_PARED_LATERAL)
                        #distancias['ps6'] > UMBRAL_PARED_LATERAL)
                      )
    pared_atras =(
                    (distancias['ps3'] > UMBRAL_PARED) or 
                    (distancias['ps4'] > UMBRAL_PARED)
                  )
                  
    orientado = (distancias['ps1'] > UMBRAL_PARED_LATERAL )

    
    # Lógica de Toma de Decisiones: Algoritmo de la mano derecha
    
    # Prioridad 1: Si no hay pared a la derecha, buscamos acercarnos a la pared derecha
    if not pared_derecha:        
        
        orientacion = True
        
        if pared_izquierda:
            print("[GIRO IZQ] Pared izquierda giro 90")
            robotMovimiento.girar_angulo(90, "derecha")
            giros_derecha += 1
            
        elif pared_frente:
            leerEntorno.fijar_estado_leds("RETROCESO")
            robotMovimiento.retroceder(1)
            print("[GIRO IZQ] Pared al frente, sigo avanzando")
            leerEntorno.fijar_estado_leds("GIROIZQ")
            robotMovimiento.girar_angulo(90, "izquierda")
            giros_izquierda += 1
            
        elif pared_atras:
            print("[MARCHA AL FRENTE] Pared atras, sigo avanzando")
            for i in range(4): robotMovimiento.avanzar(i)
            
        else:
                     
            if contador == 4: 
                print(f"[GIRO EN CIRCULOS {contador}] Saliendo del bucle circular")
                for i in range(4):
                    for i in range(4): robotMovimiento.avanzar(i)
                contador = 0  
                 
            print("[GIRO DER] Buscando pared derecha.")
            leerEntorno.fijar_estado_leds("GIRODER")
            for i in range(12):
                robotMovimiento.girar_angulo(90, "derecha")
            robotMovimiento.trasladar_distancia(DISTANCIA_PASO,4)
            giros_derecha += 1
            
            # Controlamos que no se quede girando en un solo lugar si no encuentra la pared
            contador += 1
            
    # Prioridad 2: Callejón sin salida, se da media vuelta
    elif pared_frente and pared_derecha:
        leerEntorno.fijar_estado_leds("RETROCESO")
        robotMovimiento.retroceder(1)
        print("[GIRO IZQ] Callejón sin salida. Vuelta de 90 grados.")
        leerEntorno.fijar_estado_leds("GIROIZQ")
        robotMovimiento.girar_angulo(90, "izquierda")
        giros_izquierda += 1
        
    # Prioridad 3: Si ya estamos en la pared derecha nos orientamos para que no pegue en la pared y sigue avanzando  
    elif pared_derecha:
        
        if orientado and orientacion:
            print("[Orientacion]")
            for i in range(12):
                robotMovimiento.girar_angulo(90, "izquierda")
            robotMovimiento.trasladar_distancia(DISTANCIA_PASO,4)
            giros_izquierda += 1
            orientacion = False
        else:
            print("[GIRO DER] Pared derecha, sigo avanzando")
            leerEntorno.fijar_estado_leds("AVANCE")
            robotMovimiento.avanzar(4)
            
    # Con esta condicion evaluamos si no hay choques       
    if leerEntorno.detectar_choque() > 3:
        print("¡CHOQUE DETECTADO!")
        total_choques += 1  # Registro de choque
        leerEntorno.fijar_estado_leds("ERROR_CHOQUE")
        robotMovimiento.detenerse()
        robotMovimiento.retroceder(4)
        robotMovimiento.girar_angulo(90, "izquierda")
        giros_izquierda += 1
        
    elif leerEntorno.detectar_choque() > 2 and leerEntorno.detectar_choque() < 3:
        robotMovimiento.retroceder(1)
        giros_izquierda += 1
    
    
    # Esta condicion evalua si encontramos la meta
    if robotMovimiento.distancia_meta(0.77, 0.61) < 0.15:
        
        flag = 0
        print("META ALCANZADA")

        tiempo_final = robot.getTime()
        tiempo_total = tiempo_final - tiempo_inicio
        
        while robot.step(timestep) != -1:
        
            robotMovimiento.girarEnBucle(5)
            
            for i in range(10): leerEntorno.encender_led(i)
            
            flag += 1
            
            if flag >= 150:
                break
        
        for i in range(10): leerEntorno.apagar_led(i)
        robotMovimiento.detenerse()
        
        #Reporte para evaluar el rendimiento del robot
        print("\n=============================================")
        print("       REPORTE DE RENDIMIENTO DEL ROBOT      ")
        print("=============================================")
        print(f" Tiempo total en el laberinto : {tiempo_total:.2f} segundos")
        print(f" Cantidad de choques críticos : {total_choques}")
        print(f" Giros realizados a la Izq.   : {giros_izquierda}")
        print(f" Giros realizados a la Der.   : {giros_derecha}")
        print("=============================================\n")
        
        break

    robotMovimiento.posicion()
    leerEntorno.mostrar_cambio_aceleracion()