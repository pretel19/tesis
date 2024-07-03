import rbdl
import numpy as np

#Imprimir sin notacion cientifica
np.set_printoptions(suppress=True)

# Lectura del modelo del robot a partir de URDF (parsing)
modelo = rbdl.loadModel('/home/sergio/ros_ws/src/utec/urdf/ur5_robot.urdf')
# Grados de libertad
ndof = modelo.q_size

# Configuracion articular
q = np.array([0.5, 0.2, 0.3, 0.8, 0.5, 0.6])
# Velocidad articular
dq = np.array([0.8, 0.7, 0.8, 0.6, 0.9, 1.0])
# Aceleracion articular
ddq = np.array([0.2, 0.5, 0.4, 0.3, 1.0, 0.5])

# Arrays numpy
zeros = np.zeros(ndof)          # Vector de ceros
tau   = np.zeros(ndof)          # Para torque
g     = np.zeros(ndof)          # Para la gravedad
c     = np.zeros(ndof)          # Para el vector de Coriolis+centrifuga
Mtemp = np.zeros(ndof)          # Vector temporal para la matriz de inercia
M     = np.zeros([ndof, ndof])  # Para la matriz de inercia
e     = np.eye(6)               # Vector identidad

#Numero de decimales a redondear
prec = 4

# Torque dada la configuracion del robot
rbdl.InverseDynamics(modelo, q, dq, ddq, tau)

# Parte 1: Calcular vector de gravedad, vector de Coriolis/centrifuga,
# y matriz M usando solamente InverseDynamics
print("")
#Vector gravedad
rbdl.InverseDynamics(modelo, q, zeros, zeros, g)
print("Vector gravedad: "); print(np.round(g,prec));
print("")
#Vector de fuerza centrifuga y Coriolis
rbdl.InverseDynamics(modelo, q, dq, zeros, c)
c = c-g
print("Vector de fuerza centrifuga y Corolis: "); print(np.round(c,prec));
print("")
#Matriz de inercia
for i in range(ndof):
    rbdl.InverseDynamics(modelo, q, zeros, e[i], Mtemp)
    M[:,i] = Mtemp - g
print("Matriz de inercia: "); print(np.round(M,prec));
print("")

# Parte 2: Calcular M y los efectos no lineales b usando las funciones
# CompositeRigidBodyAlgorithm y NonlinearEffects. Almacenar los resultados
# en los arreglos llamados M2 y b2
b2 = np.zeros(ndof)          # Para efectos no lineales
M2 = np.zeros([ndof, ndof])  # Para matriz de inercia
#Matriz de inercia con CompositeRigidBodyAlgorithm
rbdl.CompositeRigidBodyAlgorithm(modelo,q,M2)
print("Matriz de inercia 2: "); print(np.round(M2,prec))
print("")
#Vector de efectos no lineales con NonlinearEffects
rbdl.NonlinearEffects(modelo,q,dq,b2)
print("Vector de efectos no lineales: "); print(np.round(b2,prec))
print("")

# Parte 3: Verificacion de valores
#Booleanos de verificacion
ver_M = (np.round(M,prec) == np.round(M2,prec)).all()
ver_cg = (c+g == b2).all()
print("Matrices de inercia iguales?"); print(ver_M)
print("")
print("Vector de efectos no lineales igual a c + g?"); print(ver_cg)
print("")

# Parte 4: Verificacion de la expresion de la dinamica
#Calculo del vector de fuerzas generalizadas
tau2 = np.dot(M,ddq) + c + g
#Booleano de verificacion
ver_tau = (np.round(tau,prec) == np.round(tau2,prec)).all()
print("Matrices y vectores M, C y g satisfacen la expresion 1?"); print(ver_tau)
print("")
