from setuptools import setup

package_name = 'control_turtlebot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaime',
    maintainer_email='jamoncayop@unal.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "nodo_publicador = control_turtlebot.odomeuler:main",
            "nodo_publicador_viernes = control_turtlebot.viernes_navegacion_odometria:main",
            "nodo_publicador_profe_no_tocar = control_turtlebot.viernes_profe_navegacion_odom_no_tocar:main",
            "nodo_publicador_domingo = control_turtlebot.domingo_julian_pruebas:main",
            "nodo_publicador_difuso = control_turtlebot.logica_difusa:main",
            "nodo_publicador_difuso_v2 = control_turtlebot.logica_difusa_v2:main",
            
        ],
    },
)
