from matplotlib import pyplot as plt
from matplotlib import colors
import numpy as np
import cv2
import time
from numpy import *
from math import comb

from scipy.interpolate import BPoly,splrep,splev
# import scipy.interpolate.bisplev

def Bernstein_basis(n,v,t):
    if v<0 or v>n:
        if type(t) == type(2):
            return 0
        else:
            return np.zeros(t.shape)
    return comb(n,v)*power(t,v)*power(1-t,n-v)


def points2coef(points,segment = None,num_control_point = None):
    '''
    input: control points with shape = ((num_control_point-1)*num_segment+1, dimension). Auto append if not appropriate length.
    output: coefficient with shape = (num_control_point, num_segment, dimension)
    '''
    if num_control_point == None and segment == None:
        print("invalid input parameter")
        return
    if num_control_point == None:
        while (len(points)-1) % segment != 0:
            if type(points) == type([]):
                points.append(points[-1])
            elif type(points) == type(np.array([])):
                points = np.append(points,points[-1].reshape(1,2), axis = 0)
        num_control_point = (1+(len(points)-1)//segment)
    if segment == None:
        while len(points) % (num_control_point-1) != 1:
            if type(points) == type([]):
                points.append(points[-1])
            elif type(points) == type(np.array([])):
                points = np.append(points,points[-1].reshape(1,2), axis = 0)
        segment = (len(points)-1) // (num_control_point-1)
    
    coef = np.zeros(shape=(num_control_point, segment, len(points[0])))
    for seg in range(segment):
        for k in range(coef.shape[0]):
            coef[k,seg,:] = points[k+(num_control_point-1)*seg]
    return coef

def points2dense_smooth_points(points,segment = None,num_control_point = None,n = 100):
    coef = points2coef(points,segment,num_control_point)
    x = np.array([i for i in range(coef.shape[1]+1)])
    bp = BPoly(coef, x)

    t_point = np.arange(n)*((np.max(x) - np.min(x)))/n + np.min(x)
    y_point = bp(t_point)
    return t_point,y_point

def points2derivative_points(points,segment = None,num_control_point = None,n = 100,nu = 1):
    coef = points2coef(points,segment,num_control_point)
    x = np.array([i for i in range(coef.shape[1]+1)])
    bp = BPoly(coef, x)
    bp_d = bp.derivative(nu)

    t_point = np.arange(n)*((np.max(x) - np.min(x)))/n + np.min(x)
    y_point = bp_d(t_point)
    return t_point,y_point

def points2spline_points(points,n=100):
    if isinstance(points, np.ndarray)==False:
        points = np.array(points)
    # inputshape = (n_point*dim,)
    t = np.array([i for i in range(points.shape[0]//2)])
    xy = points.reshape(-1,2)
    spline_order = 3 if xy.shape[0] > 3 else 1
    # print('spline_order:',spline_order)
    x_spline_tck = splrep(t, xy[:,0],k = spline_order)
    y_spline_tck = splrep(t, xy[:,1],k = spline_order)
    

    t_point = np.arange(n)*((np.max(t) - np.min(t)))/n + np.min(t)
    x_point = splev(t_point, x_spline_tck)
    y_point = splev(t_point, y_spline_tck)
    # print(np.stack([x_point,y_point],axis = 1).shape,x_point.shape,y_point.shape)
    return t_point,np.stack([x_point,y_point],axis = 1)

def points2spline_angle(points, n=100):
    # 将列表转换为 NumPy 数组
    points_array = np.array(points)
    
    # 创建参数数组
    t = np.array([i for i in range(points_array.shape[0])])
    
    # 确定样条插值的阶数
    spline_order = 3 if points_array.shape[0] > 3 else 1
    
    # 进行样条插值计算
    spline_tck = splrep(t, points_array, k=spline_order)
    
    # 生成插值点
    t_point = np.linspace(np.min(t), np.max(t), n)
    spline_points = splev(t_point, spline_tck)
    
    return t_point, spline_points

def points2spline_points3D(points,n=100):
    if isinstance(points, np.ndarray)==False:
        points = np.array(points)
    t = np.array([i for i in range(points.shape[0])])
    xy = points.reshape(-1,3)
    spline_order = 3 if xy.shape[0] > 3 else 1
    # print('spline_order:',spline_order)
    x_spline_tck = splrep(t, xy[:,0],k = spline_order)
    y_spline_tck = splrep(t, xy[:,1],k = spline_order)
    z_spline_tck = splrep(t, xy[:,2],k = spline_order)
    

    t_point = np.arange(n)*((np.max(t) - np.min(t)))/n + np.min(t)
    x_point = splev(t_point, x_spline_tck)
    y_point = splev(t_point, y_spline_tck)
    z_point = splev(t_point, z_spline_tck)
    # print(np.stack([x_point,y_point],axis = 1).shape,x_point.shape,y_point.shape)
    return t_point,np.stack([x_point,y_point,z_point],axis = 1)







# points = [[1,3],[2,2],[3,8],[9,0],[10,10],[5,9],[20,5],[21,3],[25,10],[21,2],[21,2]]
# x,y = points2dense_smooth_points(points, num_control_point=5)

# print(x)
# for i in range(y.shape[0]):
#     print(y[i])


# zipped = zip(x,y)
# print(list(zipped))



# coef = points2coef(points=points,num_control_point=5)
# x = np.array([i for i in range(coef.shape[1]+1)])





# plt.plot(x,y[:,1])
# plt.show()


