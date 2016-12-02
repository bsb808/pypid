'''
PID controller class
Independent of ROS
'''

from math import copysign
from math import exp
from math import tan
import numpy as np

# For convenience...
def saturate(num,level):
    return copysign(1,num)*min(abs(num),level)      

def angleError(A,B=0.0,bound=180.0):
    ''' 
    Find difference/error of A-B within range +/-bound.
    For angles in degrees, bound is +/-180 degrees
    '''
    err = A-B
    while err < -bound:
        err += 2*bound
    while err > bound:
        err -= 2*bound
    return err
        
# Low-pass Filter Classes
class Firstlowpass(object):
    ''' 
    Implements a first-order lowpass filter based on 
    specified cut-off freq. (wc) in rad/s
    '''
    def __init__(self,wc):
        self.wc = wc
        self.previn = 0  # initialize previous input
    def execute(self,x,dt):
        '''
        One step of the filter with input x and sample time dt
        '''
        a = self.wc*dt/(self.wc*dt+1)
        out = (1-a)*self.previn + a*x
        self.previn = x
        return out
        
class Secondbutter(object):
    '''
    Implementation of second-order lowpass Butterworth fitler based
    on cut-off freq (wc) in rad/s
    '''
    def __init__(self,wc):
        self.wc = wc
        self.prevy = np.zeros(2)  # memory for prev. outputs
        self.prevx = np.zeros(2)  # memory for prev. inputs
    def execute(self,x,dt):
        wac = tan(self.wc*dt/2.0)
        # Make sure this is sufficiently far from zero
        eps = 0.01  # minimum allowable value for wac
        wac = copysign(1,wac)*max(abs(wac),eps)
        c = 1/wac
        out = (1/(c*c+1.4142*c+1)) * (2*(c*c-1)*self.prevy[0] -
                                      (c*c-1.4142*c+1)*self.prevy[1] +
                                      x + 2*self.prevx[0]+self.prevx[1])
        # Shift the memory
        self.prevy[1]=self.prevy[0]
        self.prevy[0]=out
        self.prevx[1]=self.prevx[0]
        self.prevx[0]=x
        return out

# Enumeration classes for describing variants
class Dtype():
    STANDARD=1
    FIRSTORDER=2   # First-order filter, require Td in Pid obj.


class Pid(object):
    def __init__(self,Kp,Kd,Td,Ki,maxIout=None,
                 InputIsAngle=False):
        self.Kp = float(Kp)
        self.Kd = float(Kd)
        self.Td = float(Td)
        self.Ki = float(Ki)
        self.maxIout = maxIout
        self.prevErr = 0.0
        self.prevState = None
        self.prevDest = 0.0
        self.I = 0.0
        self.InputIsAngle=InputIsAngle
        self.setpoint = 0.0
    def set_setpoint(self,setpoint):
        self.setpoint=setpoint
    def execute(self,dt,state,dstate=None):
        # Error
        if self.InputIsAngle:
            err=angleError(self.setpoint,state)
        else:
            err=self.setpoint-state

        # Proportional
        P = self.Kp*err 
        # Derivative
        # Estimate the derivative - with a filter
        # Take care of initialization
        if self.prevState== None:
            self.prevState=state
        dA = state-self.prevState
        if self.InputIsAngle:
            dA = angleError(dA)
        derivdiff = -1*(dA)/(dt)
        Dest = ((self.Td*self.prevDest) - (dA))/(self.Td+dt)
        D = self.Kd*Dest

        # Integral 
        self.I = self.I + err*dt
        # Anti-Windup - simple limit on the size of the I contribution
        if self.Ki > 0.0:
            maxI = self.maxIout/self.Ki
            self.I = saturate(self.I,maxI)
        I = self.Ki*self.I
        
        # Sum
        Out = P+I+D

        # Memory
        self.prevErr = err
        self.prevState = state
        self.prevDest = Dest

        return (Out,P,I,D)

