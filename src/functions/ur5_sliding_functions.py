import rbdl
import numpy as np

class Robot(object):
	def __init__(self, q0, dq0, ddq0, ndof, dt):
		self.q = q0    # numpy array (ndof x 1)
		self.dq = dq0  # numpy array (ndof x 1)
		self.ddq = ddq0  # numpy array (ndof x 1)
		self.M = np.zeros([ndof, ndof])
		self.b = np.zeros(ndof)
		self.dt = dt
		self.robot = rbdl.loadModel('/home/sergio/ros_ws/src/utec/urdf/ur5_xacro_mod.urdf')

	def send_command(self, tau):
		rbdl.CompositeRigidBodyAlgorithm(self.robot, self.q, self.M)
		rbdl.NonlinearEffects(self.robot, self.q, self.dq, self.b)
		ddq = np.linalg.inv(self.M).dot(tau-self.b)
		self.dq = self.dq + self.dt*ddq
		self.q = self.q + self.dt*self.dq

	def torque_to_pos(self, tau):
		rbdl.ForwardDynamics(self.robot, self.q, self.dq, tau, self.ddq)
		#rbdl.CompositeRigidBodyAlgorithm(self.robot, self.q, self.M)
		#rbdl.NonlinearEffects(self.robot, self.q, self.dq, self.b)		
		#self.ddq = np.linalg.inv(self.M).dot(tau-self.b)
		dqu = self.dq + self.dt*self.ddq
		qu = self.q + self.dt*dqu
		return qu
		
	def update_q(self, q_r, dq_r):
		self.q = q_r
		self.dq = dq_r	

	def read_joint_positions(self):
		return self.q

	def read_joint_velocities(self):
		return self.dq

# Funcion saturacion        
def sat_fnc(s,phi):

	sf = np.zeros(s.shape[0])

	for i in range(s.shape[0]):
	
		if abs(s[i]/phi) <= 1:
			sf[i] = s[i]/phi
		else:
			sf[i] = np.sign(s[i]/phi)

	return sf
