import matplotlib.pyplot as plt
import skfuzzy as fuzz
from skfuzzy import control as crtl
import numpy as np



distancia = crtl.Antecedent(np.arange(0.2,1,0.01),"distancia")
orientacion = crtl.Antecedent(np.arange(0,80,1),"orientacion")
velocidad_angular = crtl.Antecedent(np.arange(-1,1,0.01),"velocidad_angular")
velocidad_lineal = crtl.Antecedent(np.arange(0,1,0.01),"velocidad_lineal")

#paso 2)#DEfiniendo funciones de pertenencia

#Variable entrada de distancia

distancia['cerca'] = fuzz.trimf(distancia.universe, [0.19,0.3,0.6])#punta del robot 0.19 y termina en 0.3 
distancia['lejos'] = fuzz.trimf(distancia.universe, [0.4,0.7,1.1])

#distancia.view()
#plt.show()


#Variable entrada de distancia

orientacion['izq'] = fuzz.trimf(orientacion.universe, [0.0,45,45])#[-1,45,45])
orientacion['der'] = fuzz.trimf(orientacion.universe, [45,45,90])#[45,45,91])

import matplotlib.pyplot as plt
orientacion.view()
plt.show()


#Variable entrada de distancia

velocidad_lineal['baja'] = fuzz.trimf(velocidad_lineal.universe, [0.0, 0.07, 0.15])
velocidad_lineal['alta'] = fuzz.trimf(velocidad_lineal.universe, [0.07, 0.015, 0.2])

velocidad_angular['izq'] = fuzz.trimf(velocidad_angular.universe, [-1,-0.5,-0.01])
velocidad_angular['der'] = fuzz.trimf(velocidad_angular.universe, [0.01,0.5,1])
velocidad_angular['centro'] = fuzz.trimf(velocidad_angular.universe, [-0.01,0,0.01])




regla1 = crtl.Rule(
    distancia['lejos'] & orientacion["der"],
    (velocidad_lineal["alta"], velocidad_angular["centro"])
    )

regla2 = crtl.Rule(
    distancia["lejos"] & orientacion["izq"], 
    (velocidad_lineal["alta"],velocidad_angular["centro"])
    )

regla3 = crtl.Rule(
    distancia["cerca"] & orientacion["der"], 
    (velocidad_lineal["baja"],velocidad_angular["izq"])
    )

regla4 = crtl.Rule(
    distancia["cerca"] & orientacion["izq"], 
    (velocidad_lineal["baja"],velocidad_angular["der"])
    )

controlador = crtl.ControlSystem([regla1,regla2,regla3,regla4])
simulador = crtl.ControlSystemSimulation(controlador)

simulador.input["distancia"] = 0.3
simulador.input["orientacion"] = 80
#simulador.input["distancia"] = lidar

simulador.compute()

print("VElocidad del ventilador MIRAR JULIAN y ANdres:" , simulador.output[ "velocidad_ventilador" ], "km/s")


#GRaficar las funciones de pertenencia
distancia.view()
#velocidad_ventilador.view()
plt.show()
