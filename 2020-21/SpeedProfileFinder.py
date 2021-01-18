import numpy as np
import math

def energy(X, Y, Z, SP, m, g, ca, cr):
    """
    Returns the cost of a given speed profile.
    """
    assert X.shape == Y.shape and Y.shape == Z.shape and len(Z) == len(SP)+2
    V = np.zeros(len(SP)+2)
    V[1:-1] = SP
    # V = SP
    avgV = (V[1:]+V[:-1])/2
    dX = X[1:]-X[:-1]
    dY = Y[1:]-Y[:-1]
    dZ = Z[1:]-Z[:-1]
    D = np.sqrt(dX**2+dY**2+dZ**2)
    sinVA = dZ/D
    F = m*(V[1:]**2-V[:-1]**2)/(2*D) + m*g*sinVA + ca*avgV**2 + cr*m*g*np.cos(np.arcsin(sinVA))
    F = np.where(F>0, F, 0)
    return np.dot(F, D)


def energySanityTest():
    #TB Added
    X = np.array([0, 1, 0]); Y = np.array([0, 0, 0]); Z=np.array([0, 0, 0]); SP=np.array([1])
    assert energy(X, Y, Z, SP, 1, 10, 0, 0)==0.5, "Expected: 0.5 but was "+str(energy(X, Y, Z, SP, 1, 10, 0, 0))
    X = np.array([0, 1, 2]); Y = np.array([0, 0, 0]); Z=np.array([0, 0, 0]); SP=np.array([1])
    assert energy(X, Y, Z, SP, 1, 10, 0, 0)==0.5, "Expected: 0.5 but was "+str(energy(X, Y, Z, SP, 1, 10, 0, 0))
    X = np.array([0, 1, 2]); Y = np.array([0, 0, 0]); Z=np.array([0, 2, 0]); SP=np.array([1])
    assert energy(X, Y, Z, SP, 1, 10, 0, 0)==20.5, "Expected: 20.5 but was "+str(energy(X, Y, Z, SP, 1, 10, 0, 0))

# energySanityTest()
#
#     e=0;
#     time=0;
#     for i in range(len(hor)):
#         va=vangle(i);
#         vel=(v(i)+v(i+1))/2;
#         dist=(hor(i+1)-hor(i))/cosd(va);
#         time=time+dist/vel;
#         acc=vel*(v(i+1)-v(i))/dist;
#         f=m*acc+ca*vel^2+cr*m*g*cosd(va)+m*g*sind(va)+m*g*cc*vel*hangle(i);
#         t=tr/gr*f;
#         if t<0:
#             t=0;
# #    ADD EFFICIENCY
#         e=e+gr/tr*dist*t;
#         if i==length(hor)-1:
#             va=vangle(i+1);
#             dist=(hor(i+1)-hor(i))/cosd(va);
#             time=time+dist/vel;
#             acc=vel*(v(i+1)-v(i))/dist;
#             f=m*acc+ca*vel^2+cr*m*g*cosd(va)+m*g*sind(va)+m*g*cc*vel^hangle(i+1);
#             t=tr/gr*f;
#             if t<0:
#                 t=0
#         e=e+gr/tr*dist*t;
#     return e+k*(time-tmax)^2;



########################### Helper functions ###################################
