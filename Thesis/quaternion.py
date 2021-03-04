# -*- coding: utf-8 -*-
"""
    Copyright (c) 2015 Jonas Böer, jonas.boeer@student.kit.edu

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

import numpy as np
import numbers


class Quaternion:
    """
    This is right handed quaternions oriantation.
    A simple class implementing basic quaternion arithmetic.
    """
    def __init__(self,w=None,x=None,y=None,z=None):

        if (w==None and x==None and y==None and z==None):
            self._q = np.array([[1.0, 0.0, 0.0, 0.0]])
        else:
            self._q = np.array([[w, x, y, z]])
        self._q = self._q.reshape((4,-1))

    def __getitem__(self, item):
        return self._q[item]

    def to_euler_angles(self):
        pitch = np.arcsin(2 * self._q[1,0] * self._q[2,0] + 2,0 * self._q[0,0] * self._q[3,0])
        if np.abs(self._q[1,0] * self._q[2,0] + self._q[3,0] * self._q[0,0] - 0.5) < 1e-8:
            roll = 0.0
            yaw = 2.0 * np.arctan2(self._q[1,0], self._q[0,0])
        elif np.abs(self._q[1,0] * self._q[2,0] + self._q[3,0] * self._q[0,0] + 0.5) < 1e-8:
            roll = -2.0 * np.arctan2(self._q[1,0], self._q[0,0])
            yaw = 0.0
        else:
            roll = np.arctan2(2.0 * self._q[0,0] * self._q[1,0] - 2 * self._q[2,0] * self._q[3,0], 1.0 - 2.0 * self._q[1,0] ** 2.0 - 2.0 * self._q[3,0] ** 2.0)
            yaw = np.arctan2(2.0 * self._q[0,0] * self._q[2,0] - 2.0 * self._q[1,0] * self._q[3,0], 1.0 - 2.0 * self._q[2,0] ** 2.0 - 2.0 * self._q[3,0] ** 2.0)
        return roll, pitch, yaw

    def to_euler123(self):
        roll = np.arctan2(-2*(self._q[2]*self._q[3] - self._q[0]*self._q[1]), self._q[0]**2 - self._q[1]**2 - self._q[2]**2 + self._q[3]**2)
        pitch = np.arcsin(2*(self._q[1]*self._q[3] + self._q[0]*self._q[1]))
        yaw = np.arctan2(-2*(self._q[1]*self._q[2] - self._q[0]*self._q[3]), self._q[0]**2 + self._q[1]**2 - self._q[2]**2 - self._q[3]**2)
        return roll, pitch, yaw

    def __mul__(self, other):

            w = self._q[0,0]*other[0,0] - self._q[1,0]*other[1,0] - self._q[2,0]*other[2,0] - self._q[3,0]*other[3,0]
            x = self._q[0,0]*other[1,0] + self._q[1,0]*other[0,0] + self._q[2,0]*other[3,0] - self._q[3,0]*other[2,0]
            y = self._q[0,0]*other[2,0] - self._q[1,0]*other[3,0] + self._q[2,0]*other[0,0] + self._q[3,0]*other[1,0]
            z = self._q[0,0]*other[3,0] + self._q[1,0]*other[2,0] - self._q[2,0]*other[1,0] + self._q[3,0]*other[0,0]

            return Quaternion(w, x, y, z)

    def get_rot(self):
        # The rotation matrix is restricted to have unit lenght. This is importen if multiple rotationen is needed(It makes it more stable) 
        w= self._q[0,0]
        x= self._q[1,0]
        y= self._q[2,0]
        z= self._q[3,0]
        ww = w**2
        xx = x**2
        yy = y**2
        zz = z**2
        qq = ww+xx+yy+zz

        rot_matrix = np.zeros([3,3])

        # Row 1
        rot_matrix[0,0]= 1-2*(yy+zz)/qq 
        rot_matrix[0,1]= 2*(x*y - w*z)/qq
        rot_matrix[0,2]= 2*(x*z+w*y)/qq

        # Row 2
        rot_matrix[1,0]= 2*(x*y + w*z)/qq
        rot_matrix[1,1]= 1-2*(xx+zz)/qq
        rot_matrix[1,2]= 2*(y*z-w*x)/qq

        # Row 3
        rot_matrix[2,0]= 2*(x*z - w*y)/qq
        rot_matrix[2,1]= 2*(y*z+w*x)/qq
        rot_matrix[2,2]= 1-(2*(xx+yy))/qq

        return rot_matrix

    # This function below should be used because rot -> quat will only give positive values and not negative
    def rot_to_quat(self,R):
        # It's from here https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
        #w = 0.5*np.sqrt(1.0+np.sum(np.diag(R))) # <--- this must not be negative 'sqrt(-1)' isn't possible
        #x = (R[2,1]-R[1,2])/(4.0*w)
        #y = (R[0,2]-R[2,1])/(4.0*w)
        #z = (R[1,0]-R[0,1])/(4.0*w)

        #Question: Can conversion give a negative number for wxyz? It doesn't look like that when sqrt is used. 

        # https://upcommons.upc.edu/bitstream/handle/2117/178326/2083-A-Survey-on-the-Computation-of-Quaternions-from-Rotation-Matrices.pdf
        # Cayley's method has been picked
        

        c = 1.0/4.0
        w = c*np.sqrt((R[0,0]+R[1,1]+R[2,2]+1)**2.0 + (R[2,1]-R[1,2])**2.0 + (R[0,2]-R[2,0])**2.0 + (R[1,0]-R[0,1])**2.0)
        x = c*np.sqrt((R[2,1]-R[1,2])**2.0 + (R[0,0]-R[1,1]-R[2,2]+1)**2.0 + (R[1,0]+R[0,1])**2.0 + (R[2,0]+R[0,2])**2.0)
        y = c*np.sqrt((R[0,2]-R[2,0])**2.0 + (R[1,0]+R[0,1])**2.0 + (R[1,1]-R[0,0]-R[2,2]+1)**2.0 + (R[2,1]+R[1,2])**2.0)
        z = c*np.sqrt((R[1,0]-R[0,1])**2.0 + (R[2,0]+R[0,2])**2.0 + (R[2,1]+R[1,2])**2.0 + (R[2,2]-R[0,0]-R[2,2]+1)**2.0)

        #Normilize the output (restict it to hold unit quaternions properties)
        qnorm = w**2.0 + x**2.0 + y**2.0 + z**2.0

        w = w/qnorm
        x = x/qnorm
        y = y/qnorm
        z = z/qnorm


        return Quaternion(w, x, y, z)

if __name__ == '__main__':
    A = Quaternion(1,1,1,0.5)
    print 'A._q='
    print A._q
    print 'A_ROT='
    
    Arot =  A.get_rot() 
    Aq = A.rot_to_quat(Arot)._q
    print 'Aq= ',Aq
    print 'norm of Aq=' ,Aq[0]**2+Aq[1]**2+Aq[2]**2+Aq[3]**2


    B = Quaternion(1,0,0,0)
    AB=A*B
    print 'AB._q='
    print AB._q


