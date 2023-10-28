import matplotlib.pyplot as plt
import skfuzzy as fuzz
from skfuzzy import control as crtl
import numpy as np

temperatura          = crtl.Antecedent(np.arange(15,40,0.1), "temperatura")
velocidad_ventilador = crtl.Consequent(np.arange(0,10,0.1), "velocidad_ventilador")

#FUnciones de pertenencia
temperatura[ "baja" ]  = fuzz.trimf(temperatura.universe, [ 15,15,  25 ] )
temperatura[ "media" ] = fuzz.trimf(temperatura.universe, [ 20,27.5,35 ] )
temperatura[ "alta" ]  = fuzz.trimf(temperatura.universe, [ 30,40,  40 ] )

velocidad_ventilador[ "baja" ]  = fuzz.trimf(velocidad_ventilador.universe, [ 0, 0 ,5 ] )
velocidad_ventilador[ "media" ] = fuzz.trimf(velocidad_ventilador.universe, [ 3,5.5,8 ] )
velocidad_ventilador[ "alta" ]  = fuzz.trimf(velocidad_ventilador.universe, [ 6,10 ,10] )

#DEfinir las reglas

regla1 = crtl.Rule(temperatura[ "baja" ] , velocidad_ventilador[ "baja" ])
regla2 = crtl.Rule(temperatura[ "media" ] , velocidad_ventilador[ "media" ])
regla3 = crtl.Rule(temperatura[ "alta" ] , velocidad_ventilador[ "alta" ])


control_velocidad = crtl.ControlSystem( [ regla1,regla2,regla3 ]  )

#SImular el sistema de control
simulador = crtl.ControlSystemSimulation( control_velocidad )

#Asignar un paràmetro de entrada
simulador.input[ "temperatura" ] = 20


#Obtener el valor de salida ( Velocidad del ventilador )
simulador.compute()

print("VElocidad del ventilador MIRAR JULIAN:" , simulador.output[ "velocidad_ventilador" ], "km/s")


#GRaficar las funciones de pertenencia
temperatura.view()
velocidad_ventilador.view()
plt.show()