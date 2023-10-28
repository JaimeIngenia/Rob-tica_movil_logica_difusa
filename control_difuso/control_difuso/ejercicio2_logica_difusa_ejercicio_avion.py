import matplotlib.pyplot as plt
import skfuzzy as fuzz
from skfuzzy import control as crtl
import numpy as np

velocidad = crtl.Antecedent(np.arange(0,1000,1), "velocidad")
angulo = crtl.Antecedent(np.arange(-10,10,1), "angulo")

posicion = crtl.Consequent(np.arange(-5,5,0.1), "posicion")

#FUnciones de pertenencia
velocidad[ "detenido" ]  = fuzz.trimf(velocidad.universe, [ 0,0,  400 ] )
velocidad[ "rapido" ] = fuzz.trimf(velocidad.universe, [ 300,500,700 ] )
velocidad[ "muyRapido" ]  = fuzz.trimf(velocidad.universe, [ 600,800,  1000 ] )

angulo[ "inclinadoAbajo" ]  = fuzz.trimf(angulo.universe, [ -10,-10,0 ] )
angulo[ "rectoDerecho" ] = fuzz.trimf(angulo.universe, [ -5,0,5 ] )
angulo[ "inclinadoArriba" ]  = fuzz.trimf(angulo.universe, [ 0,5,10 ] )


posicion[ "muyBajo" ]  = fuzz.trimf(posicion.universe, [ -5, -5 ,-3 ] )
posicion[ "bajo" ]  =    fuzz.trimf(posicion.universe, [ -4, 2 ,0 ] )
posicion[ "medio" ] =    fuzz.trimf(posicion.universe, [ -1, 0.5 ,2 ] )
posicion[ "alto" ]  =    fuzz.trimf(posicion.universe, [ 1,2.5 ,4] )
posicion[ "muyAlto" ]  = fuzz.trimf(posicion.universe, [ 3, 4 ,5 ] )

#DEfinir las reglas

regla1 = crtl.Rule( velocidad[ "detenido" ] , angulo[ "inclinadoAbajo" ] , posicion[ "muyBajo" ] ) 
regla2 = crtl.Rule( velocidad[ "rapido" ] ,   angulo[ "inclinadoAbajo" ] , posicion[ "bajo" ] ) 
regla3 = crtl.Rule( velocidad[ "rapido" ] ,   angulo[ "rectoDerecho" ] ,   posicion[ "medio" ] ) 
regla4 = crtl.Rule( velocidad[ "muyRapido" ] , angulo[ "rectoDerecho" ] , posicion[ "alto" ] ) 
regla5 = crtl.Rule( velocidad[ "muyRapido" ] , angulo[ "inclinadoArriba" ] , posicion[ "muyAlto" ] ) 

control_velocidad = crtl.ControlSystem( [ regla1,regla2,regla3,regla4,regla5 ]  )

#SImular el sistema de control
simulador = crtl.ControlSystemSimulation( control_velocidad )

#Asignar un paràmetro de entrada
simulador.input[ "velocidad" ] = 515
simulador.input[ "angulo" ] = -2.5

#Obtener el valor de salida ( Velocidad del ventilador )
simulador.compute()

print("VElocidad del ventilador:" , simulador.output[ "posicion" ], "xxx")


#GRaficar las funciones de pertenencia
velocidad.view()
posicion.view()
angulo.view()
plt.show()