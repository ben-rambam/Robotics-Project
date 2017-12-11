from math import *
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Ellipse


# Part (a)

def FK(omega1,omega2,theta,r,L):
    xdot = r/2.0*(omega1+omega2)*np.cos(theta)
    ydot = r/2.0*(omega1+omega2)*np.sin(theta)
    tdot = r/2.0/L*(omega1-omega2)
    temp =  np.array([[xdot],[ydot],[tdot]])
    return temp

# create a vector of wheel velocities for the differential drive robot
dt = 0.2 # time step
sigma = 0.1 # wheel velocity standard deviation

t = 0
k = 0
r = 5
L = 12

n = 82

x = np.zeros((n,3,1))
u = np.zeros((n,2,1))
z = np.zeros((n,2,1))
W = np.matrix([[.08,.02],[.02,.07]])
H = np.matrix([[1,0,0],[0,1,0]])

x[0] = np.array([[0],[0],[0]])

while t < 5:
    u[k,0] = 3 + np.random.normal(0,sigma)
    u[k,1] = 3 + np.random.normal(0,sigma)
    xdot = FK(u[k,0,0],u[k,1,0],x[k,2,0],r,L)
    x[k+1] = x[k] + xdot*dt
    k = k+1
    t = t+dt

while t < 6:
    u[k,0] = 1 + np.random.normal(0,sigma)
    u[k,1] = -1 + np.random.normal(0,sigma)
    xdot = FK(u[k,0,0],u[k,1,0],x[k,2,0],r,L)
    x[k+1] = x[k] + xdot*dt
    k = k+1
    t = t+dt

while t < 10:
    u[k,0] = 3 + np.random.normal(0,sigma)
    u[k,1] = 3 + np.random.normal(0,sigma)
    xdot = FK(u[k,0,0],u[k,1,0],x[k,2,0],r,L)
    x[k+1] = x[k] + xdot*dt
    k = k+1
    t = t+dt

while t < 11:
    u[k,0] = -1 + np.random.normal(0,sigma)
    u[k,1] = 1 + np.random.normal(0,sigma)
    xdot = FK(u[k,0,0],u[k,1,0],x[k,2,0],r,L)
    x[k+1] = x[k] + xdot*dt
    k = k+1
    t = t+dt

while t < 16:
    u[k,0] = 3 + np.random.normal(0,sigma)
    u[k,1] = 3 + np.random.normal(0,sigma)
    xdot = FK(u[k,0,0],u[k,1,0],x[k,2,0],r,L)
    x[k+1] = x[k] + xdot*dt
    k = k+1
    t = t+dt

# generate some noise
Q = np.linalg.cholesky(W)

ex = np.random.normal(0,1.0,n)
ey = np.random.normal(0,1.0,n)

D = np.vstack((ex,ey))

M = np.cov(D)
MC = np.linalg.cholesky(M)
MCI = np.linalg.inv(MC)
MD = np.dot(MCI,D)

LD = np.dot(Q,MD)


for k in range(0,n):
    z[k][0][0] = x[k][0][0] + LD[0,k]
    z[k][1][0] = x[k][1][0] + LD[1,k]

#print(x[:,0,0])

# Part (c)

def f(x,u,r,L,dt):
    f = np.array([
        [x[0]+r*dt/2*(u[0]+u[1])*cos(x[2])],
        [x[1] +r*dt/2*(u[0]+u[1])*sin(x[2])],
        [x[2]+r*dt/2/L*(u[0]-u[1])]])
    return f

def F(x,u,r,L,dt):
    f = np.array([  [1, 0, -r*dt/2*(u[0]+u[1])*sin(x[2])],
        [0, 1, r*dt/2*(u[0]+u[1])*cos(x[2])],
        [0,0,1]])
    return f

def h(x,u,r,L,dt):
    h = np.array([[x[0]],[x[1]]])
    return h

#initialize the matrices
xhat = np.ndarray(shape=(n,3,1))
P = np.ndarray(shape=(n,3,3))
y = np.ndarray(shape=(n,1,1))
S = np.ndarray(shape=(n,1,1))
K = np.ndarray(shape=(n,2,1))

t = range(0,n)
#x[0] = np.matrix([[0],[0],[0]])
xhat[0] = np.matrix([[0.5],[0.5],[0.5]])
P[0] = np.matrix([[2,0,0],[0,1,0],[0,0,0.5]])
#F = np.matrix([[0,.1],[-0.02,0.2]]) #H = np.matrix([[1,0]])
V = np.matrix([[.05,.02,.01],[.02,.05,.01],[.01,.01,.1]])

for k in range(1,n):
    R = np.matrix([[np.random.normal()],[np.random.normal()],[np.random.normal()]])
    xhat[k] = f(xhat[k-1,:,0],u[k,:,0],r,L,dt)
    #print(xhat[k])
    P[k] = np.dot(np.dot(F(xhat[k,:,0],u[k,:,0],r,L,dt),P[k-1]),F(xhat[k,:,0],u[k,:,0],r,L,dt).T) + V
    #print(P[k])
    #K = np.linalg.solve((P[k]+W).T,P[k].T).T
    K = np.linalg.solve((np.dot(H,np.dot(P[k],H.T))+W).T,np.dot(H,P[k].T)).T
    #print(K)
    #print(x[k])
    #print(W)
    #print(R)
    xhat[k] = xhat[k] + np.dot(K,z[k]-h(xhat[k,:,0],u[k:,0],r,L,dt))
    temp = P[k]
    P[k] = np.dot(np.eye(3) - np.dot(K,H),temp)

plt.plot(x[1:80,0,0],x[1:80,1,0], color='g')
plt.show()
plt.plot(z[1:80,0,0],z[1:80,1,0], color='r', linestyle='-', marker='x')
plt.plot(xhat[1:80,0,0],xhat[1:80,1,0], color='b', linestyle='-', marker='x')
plt.show()
#plt.plot(t, xhat[:,0,:], color='b')

# plot some error ellipses on the path

w,v=np.linalg.eig(P[20]);

print w
print v

angle = atan2(v[0][1],v[0,0])
a = w[0]
b = w[1]

ellipse2 = Ellipse((100,5),2,1,0)
ellipse2.set_alpha(1)
c.add_artist(ellipse2)
plt.xlim([0,200])
plt.ylim([0,15])
plt.show()
