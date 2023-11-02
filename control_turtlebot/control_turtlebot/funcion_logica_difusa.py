import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import sys
print(sys.path)

def funcion_logica_difusa():
    # Paso 1) Declara variables de entrada y salida
    distancia = ctrl.Antecedent(np.arange(0.2, 1, 0.01), "distancia")
    orientacion = ctrl.Antecedent(np.arange(0, 80, 1), "orientacion")
    velocidad_angular = ctrl.Consequent(np.arange(-1, 1, 0.01), "velocidad_angular")
    velocidad_lineal = ctrl.Consequent(np.arange(0.0, 0.2, 0.01), "velocidad_lineal")

    # Paso 2) Define las funciones de pertenencia
    distancia['cerca'] = fuzz.trimf(distancia.universe, [0.19, 0.3, 0.6])
    distancia['lejos'] = fuzz.trimf(distancia.universe, [0.4, 0.7, 1.0])
    orientacion['izq'] = fuzz.trimf(orientacion.universe, [0, 20, 40])
    orientacion['der'] = fuzz.trimf(orientacion.universe, [40, 60, 80])
    velocidad_angular['izq'] = fuzz.trimf(velocidad_angular.universe, [-1, -0.5, 0])
    velocidad_angular['der'] = fuzz.trimf(velocidad_angular.universe, [0, 0.5, 1])
    velocidad_lineal['baja'] = fuzz.trimf(velocidad_lineal.universe, [0.0, 0.1, 0.2])
    velocidad_lineal['alta'] = fuzz.trimf(velocidad_lineal.universe, [0.1, 0.15, 0.2])

    # Paso 3) Define las reglas
    regla1 = ctrl.Rule(distancia['lejos'] & orientacion['der'], (velocidad_lineal['alta'], velocidad_angular['der']))
    regla2 = ctrl.Rule(distancia['lejos'] & orientacion['izq'], (velocidad_lineal['alta'], velocidad_angular['izq']))
    regla3 = ctrl.Rule(distancia['cerca'] & orientacion['der'], (velocidad_lineal['baja'], velocidad_angular['izq']))
    regla4 = ctrl.Rule(distancia['cerca'] & orientacion['izq'], (velocidad_lineal['baja'], velocidad_angular['der']))

    # Paso 4) Crea y retorna el sistema de control
    controlador = ctrl.ControlSystem([regla1, regla2, regla3, regla4])
    return ctrl.ControlSystemSimulation(controlador)