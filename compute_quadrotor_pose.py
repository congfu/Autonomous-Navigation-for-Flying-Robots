import numpy as np

class Pose3D:
    def __init__(self, rotation, translation):
        self.rotation = rotation
        self.translation = translation
        
    def inv(self):
        '''
        Inversion of this Pose3D object
        
        :return inverse of self
        '''
        # TODO: implement inversion
        #inv_rotation = self.rotation
        #inv_translation = self.translation
        m1 = np.vstack((np.hstack((self.rotation, self.translation)), np.array([[0, 0, 0, 1]])))
        m3 = np.linalg.inv(m1)
        inv_rotation = m3[0:3, 0:3]
        inv_translation = m3[0:3, 3:4]
        return Pose3D(inv_rotation, inv_translation)
    
    def __mul__(self, other):
        '''
        Multiplication of two Pose3D objects, e.g.:
            a = Pose3D(...) # = self
            b = Pose3D(...) # = other
            c = a * b       # = return value
        
        :param other: Pose3D right hand side
        :return product of self and other
        '''
        # TODO: implement multiplication
        m1 = np.vstack((np.hstack((self.rotation, self.translation)), np.array([[0, 0, 0, 1]])))
        m2 = np.vstack((np.hstack((other.rotation, other.translation)), np.array([[0, 0, 0, 1]])))
        m3 = np.dot(m1, m2)
        self.rotation = m3[0:3, 0:3]
        self.translation = m3[0:3, 3:4]
        return Pose3D(self.rotation, self.translation)
    
    def __str__(self):
        return "rotation:\n" + str(self.rotation) + "\ntranslation:\n" + str(self.translation.transpose())

def compute_quadrotor_pose(global_marker_pose, observed_marker_pose):
    '''
    :param global_marker_pose: Pose3D 
    :param observed_marker_pose: Pose3D
    
    :return global quadrotor pose computed from global_marker_pose and observed_marker_pose
    '''
    # TODO: implement global quadrotor pose computation
    global_quadrotor_pose = global_marker_pose * observed_marker_pose.inv()

    return global_quadrotor_pose

