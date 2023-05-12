import scipy.optimize as opt
from scipy.optimize import minimize_scalar
from scipy.optimize import minimize, rosen, rosen_der
from scipy.optimize import lsq_linear
from scipy.sparse import rand
import matplotlib.pyplot as plt
import matplotlib
import sys
import scipy
import numpy as np

# v = np.arange(15)
# m = v.reshape(3, 5)

np.set_printoptions(formatter={'float': lambda x: "{0:0.2f}".format(x)})

q = np.array([2.5, 3.5])
norm = np.linalg.norm(q)
print(norm)


sys.exit()

# print(m.dtype)

# m = np.empty((3, 4))
# m = np.linspace(0, 2 * np.pi, 10)

# m = np.array([[1, 2, 3], [4, 5, 6.1]])
# v = np.arange(0, 10)**3
# print(v)

# print(v[2:5])

# v = np.array([1, 2]).reshape(2, 1)

# m1 = np.array([[1, 2]]).reshape(2, 1)  # , [-2, 1]])

# q = np.zeros((2, 0))

# print(q)

# q = np.c_[q, np.ones(q.shape[0])]

# print(q)

# q = np.c_[q, np.full((q.shape[0], 1), 9)]

# print(q)


def functionyouwanttofit(x, y, z, t, u):
    return np.array([x+y+z+t+u, x+y+z+t-u, x+y+z-t-u, x+y-z-t-u])  # baby test here but put what you want


def calc_chi2(parameters):
    x, y, z, t, u = parameters
    data = np.array([100, 250, 300, 500])
    chi2 = sum((data-functionyouwanttofit(x, y, z, t, u))**2)
    return chi2


# baby example for init, min & max values
x_init = 0
x_min = -1
x_max = 10

y_init = 1
y_min = -2
y_max = 9

z_init = 2
z_min = 0
z_max = 1000

t_init = 10
t_min = 1
t_max = 100

u_init = 10
u_min = 1
u_max = 100

parameters = [x_init, y_init, z_init, t_init, u_init]
bounds = [[x_min, x_max], [y_min, y_max], [z_min, z_max], [t_min, t_max], [u_min, u_max]]
result = opt.minimize(calc_chi2, parameters, bounds=bounds)

print(result)

sys.exit(0)


def ry_matrix(theta):
    """Rotation matrix around the Y axis"""
    return np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])


def translation(x, y, z):
    return np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])


def rot_90():
    rotmat = ry_matrix(np.deg2rad(90))

    q = np.identity(4)
    q[:-1, :-1] = rotmat
    return q


frame_matrx = np.identity(4)

# frame_matrx = frame_matrx.dot(transform(0, 0, 1.08))
# frame_matrx = frame_matrx.dot(transform(0.16, 0, 0))
frame_matrx = frame_matrx.dot(translation(6, 0, 0))

frame_matrx = frame_matrx.dot(rot_90())

frame_matrx = frame_matrx.dot(translation(2.97, 0, 0))

axis = np.array([0, 1, 0])

# print(axis * 0.17)


def fk_test(v):
    def rotation_pitch(theta):
        rotmat = ry_matrix(theta)

        q = np.identity(4)
        q[:-1, :-1] = rotmat
        return q

    frame_matrx = np.identity(4)

    frame_matrx = frame_matrx.dot(rotation_pitch(v[0]))
    frame_matrx = frame_matrx.dot(translation(6, 0, 0))
    frame_matrx = frame_matrx.dot(rotation_pitch(v[1]))
    frame_matrx = frame_matrx.dot(translation(3, 0, 0))

    return frame_matrx


# set_val = [np.deg2rad(-45), np.deg2rad(50)]
set_val = [np.deg2rad(-181.57), np.deg2rad(2.22)]

print(set_val)

fk_calc = fk_test(set_val)

print(fk_calc[:3, 3])
# sys.exit(0)

target = [7.23, 0, 3.98]

initial_angles = [-1, 1.57]

# print("initial angles: ", initial_angles[::-1])


matplotlib.use('TkAgg')

# t = np.arange(0.0, 2.0, 0.01)
# s = 1 + np.sin(2 * np.pi * t)

# fig, ax = plt.subplots()
# ax.plot(t, s)

# ax.set(xlabel='time (s)', ylabel='voltage (mV)',
#        title='About as simple as it gets, folks')
# ax.grid()

# plt.show()

# sys.exit(0)


def optimize_function(x):
    rev = x

    print("new angles: ", np.vectorize(np.rad2deg)(rev))
    fk = fk_test(rev)

    target_error = fk[:3, -1] - target

    return target_error


# res = scipy.optimize.least_squares(optimize_function, initial_angles, bounds=([-1.57, 0.5236], [1.57, 2]))
# print(res)
# print(np.vectorize(np.rad2deg)(res.x))


# minimizer_kwargs = {"jac": True}
# x0 = [0, 0]
# ret = scipy.optimize.basinhopping(optimize_function, x0, minimizer_kwargs=minimizer_kwargs, niter=100)
# print(ret)
# print(np.vectorize(np.rad2deg)(ret.x))


# index = 0
# target = np.array([
#     [0, 0, 0],
#     [5.36, 0, 1.11],
#     [0, 5.36, 1.11],
# ])

# print("target", target[index+1])

print("\nOptimizer")


t = 2.00


def obj(x):
    print("X", x)
    return t - x


res = minimize_scalar(obj)
print(res)


# t = [0.00, 0.00, -0.52, 2.00, 0.00]

# def obj(x):
#     print("X", x)
#     return t - x

# x0 = [0.00, 0.00, 0.00, 0.53, 0.00]
# res = minimize(obj, x0, method='Nelder-Mead', tol=1e-6)
# print(res.x)

# A = np.array([[0.00, 0.00, 0.00, 0.53, 0.00]])
# b = np.array([0.00, 0.00, -0.52, 2.00, 0.00])

# A = rand(10, 5, density=1e-4, random_state=np.random.default_rng())

# res = lsq_linear(A, b)

# print(A)
# print(b)

# A = np.array([[1, 0], [1, 0], [0, 1]])
# b = np.array([2, 1, 1])

# nnls(A, b)
