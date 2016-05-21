import numpy as np
from scipy import integrate
from matplotlib import pyplot as plt

def heaviside(t, a):
    return 1 if t > a else 0

m = 1
b = 1
k = 2
set_point = 10

kp = 0.04

def y_a(t): return set_point / kp # np.sin(t) #set_point * b

def solver(Y, t):
    return [m * Y[1], -k * Y[0] - b * Y[1] - kp * (Y[0] - y_a(t)) + set_point * kp]# heaviside(t, 5)] # np.sin(t)]

def main():
    t = np.arange(0, 25.0, 0.01)
    x = integrate.odeint(solver, [0, 0], t)
    plt.plot(t, x[:, 0])
    # plt.plot(t, np.sin(t))
    plt.show()

if __name__ == '__main__':
    main()
