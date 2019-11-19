# Matrix and quaternions manipulations
import numpy as np
from pyquaternion import Quaternion

# Registration
from functools import partial
from pycpd import rigid_registration
import time

def rotateQuaternion(x, y, z, Q, use_multicore=False, cores=1):
    xyz = np.transpose(np.vstack([x,y,z]))
    if use_multicore == False:
        listxyz = []
        for i in range(xyz.shape[0]):
            q = Q[i]
            v = xyz[i,:]
            v_prime = q.rotate(v)
            listxyz.append(v_prime)
    else:
        import pymp
        xyz = np.transpose(np.vstack([x,y,z]))
        totaldata = len(x)
        listxyz = pymp.shared.list()
        with pymp.Parallel(cores) as p:
            for i in p.range(totaldata):
                q = Q[i]
                v = xyz[i,:]
                v_prime = q.rotate(v)
                listxyz.append(v_prime)
        listxyz = list(listxyz)
    xyz_prime = np.vstack(listxyz)
    return xyz_prime[:,0], xyz_prime[:,1], xyz_prime[:,2]
    
def fromRPYtoQuaternion(yaw, pitch, roll): # yaw (Z), pitch (Y), roll (X)
    # Abbreviations for the various angular functions
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    w = ( cy * cp * cr) + (sy * sp * sr)
    x = (cy * cp * sr) - (sy * sp * cr)
    y = (sy * cp * sr) + (cy * sp * cr)
    z = (sy * cp * cr) - (cy * sp * sr)
    q = Quaternion([w,x,y,z]) 
    return q
    
def runregistration(iteration, error, X, Y, val):
    print('iteration: ' + str(iteration) + ', error: ' + str(error))
    
def registerSourceTarget(Source, Target, max_iterations=200, tolerance = 0.001):
    reg = rigid_registration(**{ 'X': Source, 'Y': Target, 'max_iterations': max_iterations, 'tolerance':tolerance})
    callback = partial(runregistration, val=1)
    reg.register(callback)
