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
from math import sqrt, atan2, asin


class Quaternion:
    """
    A simple class implementing basic quaternion arithmetic.
    """
    def __init__(self, w_or_q, x=None, y=None, z=None):
        """
        Initializes a Quaternion object
        :param w_or_q: A scalar representing the real part of the quaternion, another Quaternion object or a
                    four-element array containing the quaternion values
        :param x: The first imaginary part if w_or_q is a scalar
        :param y: The second imaginary part if w_or_q is a scalar
        :param z: The third imaginary part if w_or_q is a scalar
        """
        self._q = np.array([1, 0, 0, 0],dtype=np.float64)

        if x is not None and y is not None and z is not None:
            w = w_or_q
            q = np.array([w, x, y, z],dtype=np.float64)
        elif isinstance(w_or_q, Quaternion):
            q = np.array(w_or_q.q,dtype=np.float64)
        else:
            q = np.array(w_or_q,dtype=np.float64)
            if len(q) != 4:
                raise ValueError("Expecting a 4-element array or w x y z as parameters")

        self._set_q(q)

    # Quaternion specific interfaces

    def conj(self):
        """
        Returns the conjugate of the quaternion
        :rtype : Quaternion
        :return: the conjugate of the quaternion
        """
        return Quaternion(self._q[0], -self._q[1], -self._q[2], -self._q[3])

    def norm_quat(self):
        
        w = self._q[0]
        x = self._q[1]
        y = self._q[2]
        z = self._q[3]

        qnorm = sqrt(w**2 + x**2 + y**2 + z**2)

        w = w/qnorm 
        x = x/qnorm 
        y = y/qnorm 
        z = z/qnorm 
        return Quaternion(w_or_q=w,x=x,y=y,z=z)

    def to_angle_axis(self):
        """
        Returns the quaternion's rotation represented by an Euler angle and axis.
        If the quaternion is the identity quaternion (1, 0, 0, 0), a rotation along the x axis with angle 0 is returned.
        :return: rad, x, y, z
        """
        if self[0] == 1 and self[1] == 0 and self[2] == 0 and self[3] == 0:
            return 0, 1, 0, 0
        rad = np.arccos(self[0]) * 2
        imaginary_factor = np.sin(rad / 2)
        if abs(imaginary_factor) < 1e-8:
            return 0, 1, 0, 0
        x = self._q[1] / imaginary_factor
        y = self._q[2] / imaginary_factor
        z = self._q[3] / imaginary_factor
        return rad, x, y, z

    @staticmethod
    def from_angle_axis(rad, x, y, z):
        s = np.sin(rad / 2)
        return Quaternion(np.cos(rad / 2), x*s, y*s, z*s)

    def to_euler_angles(self):
        # The output is in radians
        pitch = np.arcsin(2 * self[1] * self[2] + 2 * self[0] * self[3])
        if np.abs(self[1] * self[2] + self[3] * self[0] - 0.5) < 1e-8:
            roll = 0
            yaw = 2 * np.arctan2(self[1], self[0])
        elif np.abs(self[1] * self[2] + self[3] * self[0] + 0.5) < 1e-8:
            roll = -2 * np.arctan2(self[1], self[0])
            yaw = 0
        else:
            roll = np.arctan2(2 * self[0] * self[1] - 2 * self[2] * self[3], 1 - 2 * self[1] ** 2 - 2 * self[3] ** 2)
            yaw = np.arctan2(2 * self[0] * self[2] - 2 * self[1] * self[3], 1 - 2 * self[2] ** 2 - 2 * self[3] ** 2)
        return roll, pitch, yaw

    def to_euler123(self):
        roll = np.arctan2(-2*(self[2]*self[3] - self[0]*self[1]), self[0]**2 - self[1]**2 - self[2]**2 + self[3]**2)
        pitch = np.arcsin(2*(self[1]*self[3] + self[0]*self[1]))
        yaw = np.arctan2(-2*(self[1]*self[2] - self[0]*self[3]), self[0]**2 + self[1]**2 - self[2]**2 - self[3]**2)
        return roll, pitch, yaw

    def __mul__(self, other):
        """
        multiply the given quaternion with another quaternion or a scalar
        :param other: a Quaternion object or a number
        :return:
        """
        if isinstance(other, Quaternion):
            w = self._q[0]*other._q[0] - self._q[1]*other._q[1] - self._q[2]*other._q[2] - self._q[3]*other._q[3]
            x = self._q[0]*other._q[1] + self._q[1]*other._q[0] + self._q[2]*other._q[3] - self._q[3]*other._q[2]
            y = self._q[0]*other._q[2] - self._q[1]*other._q[3] + self._q[2]*other._q[0] + self._q[3]*other._q[1]
            z = self._q[0]*other._q[3] + self._q[1]*other._q[2] - self._q[2]*other._q[1] + self._q[3]*other._q[0]

            return Quaternion(w, x, y, z)
        elif isinstance(other, numbers.Number):
            q = self._q * other
            return Quaternion(q[0],q[1],q[2],q[3])

    def __add__(self, other):
        """
        add two quaternions element-wise or add a scalar to each element of the quaternion
        :param other:
        :return:
        """
        if not isinstance(other, Quaternion):
            if len(other) != 4:
                raise TypeError("Quaternions must be added to other quaternions or a 4-element array")
            q = self.q + other
        else:
            q = self.q + other.q

        return Quaternion(q)

    # Implementing other interfaces to ease working with the class

    def _set_q(self, q):
        self._q = q.reshape((4,1))

    def _get_q(self):
        return self._q

    q = property(_get_q, _set_q)

    def __getitem__(self, item):
        return self._q[item]

    def __array__(self):
        return self._q


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
        qq = sqrt(ww+xx+yy+zz)

        rot_matrix = np.zeros([3,3])

        # Row 1
        rot_matrix[0,0]= (ww+xx-yy-zz)/qq#1-2*(yy+zz)/qq 
        rot_matrix[0,1]= 2*(x*y - w*z)/qq
        rot_matrix[0,2]= 2*(x*z+w*y)/qq

        # Row 2
        rot_matrix[1,0]= 2*(x*y + w*z)/qq
        rot_matrix[1,1]= (ww-xx+yy+zz)/qq#1-2*(xx+zz)/qq
        rot_matrix[1,2]= 2*(y*z-w*x)/qq

        # Row 3
        rot_matrix[2,0]= 2*(x*z - w*y)/qq
        rot_matrix[2,1]= 2*(y*z+w*x)/qq
        rot_matrix[2,2]= (ww-xx-yy+zz)/qq#1-(2*(xx+yy))/qq

        return rot_matrix

    # This function below should be used because rot -> quat will only give positive values and not negative
    def rot_to_quat(self,R):

        # This doesn't produce good results
        #(w, x, y, z) = self.rot_to_quat_m1(R) 

        
        # This produce good results but for the w, but the rest can't be negative because of sqrt...
        (w, x, y, z) = self.rot_to_quat_m2(R)

        # This doesn't produce good results.
        #(w, x, y, z) = self.rot_to_quat_m3(R) 

        return Quaternion(w, x, y, z)

    def rot_to_quat_m1(self,R):
        # Method 1
        # It's from here https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
        w = 0.5*np.sqrt(1.0+np.sum(np.diag(R))) # <--- this must not be negative 'sqrt(-1)' isn't possible
        x = (R[2,1]-R[1,2])/(4.0*w)
        y = (R[0,2]-R[2,1])/(4.0*w)
        z = (R[1,0]-R[0,1])/(4.0*w)

        qnorm = w**2.0 + x**2.0 + y**2.0 + z**2.0

        w = w/qnorm
        x = x/qnorm
        y = y/qnorm
        z = z/qnorm

        return (w, x, y, z)

    def rot_to_quat_m2(self,R):
        # Method 2
        # --- !! QUESTION !!: Can conversion give a negative number for wxyz? It doesn't look like that when sqrt is used.  

        # https://upcommons.upc.edu/bitstream/handle/2117/178326/2083-A-Survey-on-the-Computation-of-Quaternions-from-Rotation-Matrices.pdf
        # Cayley's method has been picked
        
        
        c = 1.0/4.0
        w = c*np.sqrt((R[0,0]+R[1,1]+R[2,2]+1.0)**2.0 + (R[2,1]-R[1,2])**2.0 + (R[0,2]-R[2,0])**2.0 + (R[1,0]-R[0,1])**2.0)
        x = c*np.sqrt((R[2,1]-R[1,2])**2.0 + (R[0,0]-R[1,1]-R[2,2]+1.0)**2.0 + (R[1,0]+R[0,1])**2.0 + (R[2,0]+R[0,2])**2.0)
        y = c*np.sqrt((R[0,2]-R[2,0])**2.0 + (R[1,0]+R[0,1])**2.0 + (R[1,1]-R[0,0]-R[2,2]+1.0)**2.0 + (R[2,1]+R[1,2])**2.0)
        z = c*np.sqrt((R[1,0]-R[0,1])**2.0 + (R[2,0]+R[0,2])**2.0 + (R[2,1]+R[1,2])**2.0 + (R[2,2]-R[0,0]-R[2,2]+1.0)**2.0)

        #Normilize the output (restict it to hold unit quaternions properties)
        qnorm = w**2.0 + x**2.0 + y**2.0 + z**2.0

        w = w/qnorm
        x = x/qnorm
        y = y/qnorm
        z = z/qnorm

        return (w, x, y, z)

    def rot_to_quat_m3(self,R):
        # Method 3 A simple way of doing it  From https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/ 

        trace = R[0,0] + R[1,1] + R[2,2]+1.0 # is removed to avoiding results that is close so zero if its negativ number(R00+R11+R22)
        if( trace > 0 ):
            s = 0.5 / np.sqrt(trace+ 1.0)
            w = 0.25 / s
            x = ( R[2,1] - R[1,2] ) * s
            y = ( R[0,2] - R[2,0] ) * s
            z = ( R[1,0] - R[0,1] ) * s
        else :
            if( R[0,0] > R[1,1] and R[0,0] > R[2,2] ):
                s = 2.0 * np.sqrt( 1.0 + R[0,0] - R[1,1] - R[2,2])
                w = (R[2,1] - R[1,2] ) / s
                x = 0.25 * s
                y = (R[0,1] + R[1,0] ) / s
                z = (R[0,2] + R[2,0] ) / s
            elif(R[1,1] > R[2,2]):
                s = 2.0 * np.sqrt( 1.0 + R[1,1] - R[0,0] - R[2,2])
                w = (R[0,2] - R[2,0] ) / s
                x = (R[0,1] + R[1,0] ) / s
                y = 0.25 * s
                z = (R[1,2] + R[2,1] ) / s
            else:
                s = 2.0 * np.sqrt( 1.0 + R[2,2] - R[0,0] - R[1,1] )
                w = (R[1,0] - R[0,1] ) / s
                x = (R[0,2] + R[2,0] ) / s
                y = (R[1,2] + R[2,1] ) / s
                z = 0.25 * s

        #Normilize the output (restict it to hold unit quaternions properties)
        qnorm = w**2.0 + x**2.0 + y**2.0 + z**2.0

        w = w/qnorm
        x = x/qnorm
        y = y/qnorm
        z = z/qnorm


        return (w, x, y, z)

if __name__ == '__main__':
    A = Quaternion(0.70696229,-0.0608467,0.05761432, 0.7022696) 
    Arot =  A.get_rot() 
    print ('A._q=')
    print (A._q)
    print ('A_ROT=')
    print(Arot)
    
    Aq = A.rot_to_quat(Arot)._q
    print ('Aq= ',Aq)
    print ('norm of Aq=' ,sqrt(Aq[0]**2+Aq[1]**2+Aq[2]**2+Aq[3]**2))


    B = Quaternion(1,0,0,0)
    AB=A*B
    print ('AB._q=')
    print (AB._q)
    print('norm of AB._q= ',AB.norm_quat()._q)

    print('')
    C = Quaternion(B)
    print('C= ',C.norm_quat()._q)

