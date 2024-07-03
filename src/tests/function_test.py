import sys
sys.path.insert(0, '/home/sergio/ros_ws/src/utec/src/functions')
from ur5_params import *
from ur_kinematics_functions import *

q0 = np.radians(np.array([7.44, -79.62, 91.1, -102.93, -90.72, 4.78]))

q = np.array([0.1298525, -1.38963115, 1.58999495, -1.7964674, -1.5833627, 0.08342674])
xd = np.array([-0.7892,  0.1302,  0.1092])


#J = jacobian_position(q0,l)

#qdes = ikine(xd, q, l)

J = jacobian_pose(q,l)

print(J)
