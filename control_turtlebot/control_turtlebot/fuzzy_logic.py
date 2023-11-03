import skfuzzy as fuzz
from skfuzzy import control as crtl
import numpy as np

def setup_fuzzy_logic(indice, lidar):
    # Definir variables de entrada y salida
    distancia = crtl.Antecedent(np.arange(0.2, 1, 0.01), "distancia")
    orientacion = crtl.Antecedent(np.arange(0, 80, 1), "orientacion")
    velocidad_angular = crtl.Consequent(np.arange(-1, 1, 0.01), "velocidad_angular")
    velocidad_lineal = crtl.Consequent(np.arange(0.0, 0.2, 0.01), "velocidad_lineal")

    # Definir funciones de pertenencia
    distancia['cerca'] = fuzz.trimf(distancia.universe, [0.19, 0.3, 0.6])
    distancia['lejos'] = fuzz.trimf(distancia.universe, [0.04, 0.7, 1.1])
    orientacion['izq'] = fuzz.trimf(distancia.universe, [-1, 45, 45])
    orientacion['der'] = fuzz.trimf(distancia.universe, [45, 45, 91])

    velocidad_angular['izq'] = fuzz.trimf(velocidad_angular.universe, [-1, -0.575, -0.25])
    velocidad_angular['der'] = fuzz.trimf(velocidad_angular.universe, [-0.25, 0, 0.25])
    velocidad_angular['centro'] = fuzz.trimf(velocidad_angular.universe, [0.25, 0.575, 1])

    velocidad_lineal['baja'] = fuzz.trimf(velocidad_lineal.universe, [0.0, 0.07, 0.15])
    velocidad_lineal['alta'] = fuzz.trimf(velocidad_lineal.universe, [0.07, 0.15, 0.2])

    # Definir reglas de lógica difusa
    regla1 = crtl.Rule(
        distancia['lejos'] & orientacion["der"],
        (velocidad_lineal["alta"], velocidad_angular["centro"])
    )
    regla2 = crtl.Rule(
        distancia["lejos"] & orientacion["izq"],
        (velocidad_lineal["alta"], velocidad_angular["centro"])
    )
    regla3 = crtl.Rule(
        distancia["cerca"] & orientacion["der"],
        (velocidad_lineal["baja"], velocidad_angular["izq"])
    )
    regla4 = crtl.Rule(
        distancia["cerca"] & orientacion["izq"],
        (velocidad_lineal["baja"], velocidad_angular["der"])
    )

    controlador = crtl.ControlSystem([regla1,regla2,regla3,regla4])
    simulador = crtl.ControlSystemSimulation(controlador)

    #indice = 25

    #print("Input antes de *************************************** :" + str( indice ))
    simulador.input["orientacion"] = indice
    simulador.input["distancia"] = lidar

    # print("Observar el input :" + str( indice ))
    # print("Observar el lidar :" + str( lidar ))

    simulador.compute()

    lineal = simulador.output[ 'velocidad_lineal']  
    angular = simulador.output[ 'velocidad_angular']  
    
    return lineal, angular
