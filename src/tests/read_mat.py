import scipy.io
mat = scipy.io.loadmat('/home/sergio/ros_ws/src/utec/data/trajectory1.mat')

mat_contents = sp.loadmat("data.mat")
data= mat_contents['data']
Y=data['labels'] [0][0]
X=data['features'] [0][0]
