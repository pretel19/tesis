import numpy as np

#Longitudes (en metros) del DH
l1 = 0.163
l2 = 0.425
l3 = 0.39225
l4 = 0.134
l5 = 0.100
l6 = 0.100
l = [l1, l2, l3, l4, l5, l6]

#Limites articulares
q_lim = np.around(np.radians(np.array([[-360.0, 360.0],[-360.0, 360.0],[-180.0, 180.0],[-360.0, 360.0],[-360.0, 360.0],[-360.0, 360.0]])),4)
dq_lim = np.around(np.radians(np.array([[-180.0, 180.0],[-180.0, 180.0],[-180.0, 180.0],[-180.0, 180.0],[-180.0, 180.0],[-180.0, 180.0]])),4)

#Workspace
#R1 = 0.85 #Radio horizontal de la esfera
R1 = 0.95
#R2 = 0.9735 #Radio de la esfera
R2 = 1.1
ri = 0.151 #Radio del cilindro
r_ws = [R1, R2, ri]
