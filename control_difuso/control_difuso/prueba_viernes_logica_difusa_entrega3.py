import matplotlib.pyplot as plt
import skfuzzy as fuzz
from skfuzzy import control as crtl
import numpy as np

longitud = crtl.Antecedent(np.arange(0.2,1,0.01),"longitud")
rumbo = crtl.Antecedent(np.arange(0,80,1),"rumbo")
vel_W = crtl.Consequent(np.arange(-1,1,0.01),"vel_W")
vel_v = crtl.Consequent(np.arange(0,1,0.01),"vel_v")

longitud['cerca'] = fuzz.trimf(longitud.universe, [0.19,0.3,0.6])
longitud['lejos'] = fuzz.trimf(longitud.universe, [0.4,0.7,1.1])

longitud.view()
plt.show()

rumbo['izq'] = fuzz.trimf(rumbo.universe, [0.0,45,45])#[-1,45,45])
rumbo['der'] = fuzz.trimf(rumbo.universe, [45,45,90])#[45,45,91])
rumbo.view()
plt.show()

vel_v['baja'] = fuzz.trimf(vel_v.universe, [0.0, 0.14, 0.30])
vel_v['alta'] = fuzz.trimf(vel_v.universe, [0.14, 0.30, 0.4])

vel_v.view()
plt.show()

vel_W['izq'] = fuzz.trimf(vel_W.universe, [-1,-0.5,-0.01])#[-1,-0.5,-0.01])
vel_W['centro'] = fuzz.trimf(vel_W.universe, [-0.01,0,0.01])#[-0.01,0,0.01])
vel_W['der'] = fuzz.trimf(vel_W.universe, [0.01,0.5,1])#[0.01,0.5,1])

vel_W.view()
plt.show()

regla1 = crtl.Rule(
    longitud['lejos'] & rumbo["der"],
    (vel_v["alta"], vel_W["centro"])
    )

regla2 = crtl.Rule(
    longitud["lejos"] & rumbo["izq"], 
    (vel_v["alta"],vel_W["centro"])
    )

regla3 = crtl.Rule(
    longitud["cerca"] & rumbo["der"], 
    (vel_v["baja"],vel_W["izq"])
    )

regla4 = crtl.Rule(
    longitud["cerca"] & rumbo["izq"], 
    (vel_v["baja"],vel_W["der"])
    )

controlador = crtl.ControlSystem([regla1,regla2,regla3,regla4])
simulador = crtl.ControlSystemSimulation(controlador)

simulador.input["longitud"] = 0.3
simulador.input["rumbo"] = 50
simulador.compute()

print("VElocidad del velocidad v y la velocidad w MIRAR JULIAN y ANdres:" , simulador.output[ "vel_v" ], simulador.output[ "vel_W" ] , "km/s")

 

