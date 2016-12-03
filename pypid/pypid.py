"""PID controller class
Independent of ROS
"""
__version__ = "0.0.1"

from math import copysign
from math import exp, tan, pi
import numpy as np

# For convenience...
def saturate(num,level):
    return copysign(1,num)*min(abs(num),level)      

def angleError(A,B=0.0,bound=180.0):
    """ 
    Find difference/error of A-B within range +/-bound.
    For angles in degrees, bound is +/-180 degrees
    """
    err = A-B
    while err < -bound:
        err += 2*bound
    while err > bound:
        err -= 2*bound
    return err
        
# Low-pass Filter Classes
class Zerolowpass(object):
    """
    No filter
    """
    def __init(self):
        pass
    def execute(self,dt,x):
        return x

class Firstlowpass(object):
    """ 
    Implements a first-order lowpass filter based on 
    specified cut-off freq. (wc) in rad/s
    """
    def __init__(self,wc):
        self.wc = float(wc)
        self.prevout = 0  # initialize previous input
    def execute(self,dt,x):
        """
        One step of the filter with input x and sample time dt
        """
        a = self.wc*dt/(self.wc*dt+1)
        self.prevout = (1-a)*self.prevout + a*x
        return self.prevout
        
class Secondbutter(object):
    """
    Implementation of second-order lowpass Butterworth fitler based
    on cut-off freq (wc) in rad/s
    """
    def __init__(self,wc):
        self.wc = float(wc)
        self.prevy = np.zeros(2)  # memory for prev. outputs
        self.prevx = np.zeros(2)  # memory for prev. inputs
    def execute(self,dt,x):
        wac = tan(self.wc*dt/2.0)
        # Make sure this is sufficiently far from zero
        eps = 1.0e-6  # minimum allowable value for wac
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
    """
    PID control object.

    Standard controller is initiated by defining the gains as
    Kp: Proportional gain
    Ki: Integral gain
    Kd: Derivative gain

    General workflow is to instatiate the basic controller object,
    then setup the architctureal and filter options,
    then when using the feedback 
    - change the setpoint (goal) by with the set_setpoint() attribute
    - call the execute() attribute with the sample time and 
       state/process-variable (input to the controller) to generate the 
       control output

    Optional Variants:

    * Rate Sensor *
    In the standard from the derivative of the process variable is 
    estimated based on the input.  If there is a separate sensor for the 
    process-variable and the rate of the process variable (e.g., compass 
    and gyro), this can be included in the call to execute.
    - To use - Call execute with the optional dstate input argument
    The dstate value should be the measure rate of change.
    
    * Angular Input *
    If the input/process-variable has a discontinuity, e.g, and angle
    that wraps at 360 or 2*pi, the controller will unwrap accordingly.
    - To use, call the set_inputisangle(True)

    * Input Filter *
    Puts a low-pass filter in the input/state/process-variable input.
    Filter can...
      - none: order=0
      - first-order: order=1
      - second-order: order=2
    The cut-off frequency (wc) is specified in rad/s
    - To use, call the set_inputfilter() function, specifing order and cut-off

    * Derivative Filter *
    Puts a low-pass filter on the derivative estimate.  Same filters
    as the input filter
    - To use, call the set_derivfilter() function with order and cut-off

    * Derivative Feedback *
    In standard form (derivfeedback=False), 
    the derivative term is calculated based on the 
    derivative of the error (setpoint-state).  
    The alternative (derivfeedback=True) is for the derivative in the 
    feedback path so that the derivative term is the derivative of state alone.
    - To use, call the set_derivfeedback(Truee) function
    
    * Anti-Windup *
    The maximum contribution of the I term in the controller output is set
    by the maxIout parameter.  This is set in units of controller output,
    so the internal integration limit is back calculated based on the value
    of Ki.
    - To use, call the set_maxIout() function.
    
    * Rate Limits TODO *
    
    """
    def __init__(self,Kp,Ki,Kd,
                 maxIout=None,
                 inputIsAngle=False,
                 inputFilterOrder=0,
                 derivFilterOrder=0):
        """
        Initial configuratoin has no options - just vanilla PID
        """
        self.Kp = float(Kp)
        self.Ki = float(Ki)
        self.Kd = float(Kd)
        # Standard options
        # Antiwindup
        self.maxIout = maxIout
        # Angle wrap
        self.inputIsAngle=bool(inputIsAngle)
        self.anglewrap = pi
        # Input filter
        self.inputfilter = Zerolowpass()
        # Input filter
        self.derivfilter = Zerolowpass()
        # Derivative in feedback - it true use deriv of PV, 
        #   otherwise deriv of error
        self.derivfeedback = False
        # Iniitial values
        self.I = 0.0
        self.prevsetpoint = 0.0  # for calculating raw deriv
        self.prevstate = 0.0
        self.setpoint = 0.0


    def initfilter(self,order,wc):
        """
        Returns the appropriate filter - used as common way for setting
        input filters
        """
        filt = None
        assert (order<=3 and order>=0),"Filter order must be 0, 1 or 2"
        if order == 0:
            filt = Zerolowpass()
        elif order == 1:
            filt = Firstlowpass(wc)
        elif order == 2:
            filt = Secondbutter(wc)
        else:
            raise Exception("Filter order must be 0, 1 or 2")
        return filt
    
    def set_inputfilter(self,order,wc):
        """
        Setup input filter 
        wc is in rad/s
        """ 
        self.inputfilter = self.initfilter(order,wc)

    def set_derivfilter(self,order,wc):
        self.derivfilter = self.initfilter(order,wc)

    def set_derivfeedback(self,derivfeedback):
        self.derivfeedback=bool(derivfeedback)
    
    def set_maxIout(self,maxIout):
        self.maxIout=float(maxIout)

    def set_inputisangle(self,inputIsAngle,bound=pi):
        self.inputIsAngle=bool(inputIsAngle)
        self.anglewrap = bound

    def set_setpoint(self,setpoint):
        self.setpoint=float(setpoint)
    
    def execute(self,dt,state,dstate=None):
        """
        Pid implementation call
        
        dt - time step in seconds
        state - process variable, fed back from plant
        dstate - rate of change of process variable
                 If None, then will estimate derivative
        """
        if dt < 1.0e-6:
            return np.array([0,0,0,0])
        # Calc error
        if self.inputIsAngle:
            err=angleError(self.setpoint,state,self.anglewrap)
        else:
            err=self.setpoint-state
        # Proportional
        #print err
        P = self.Kp*self.inputfilter.execute(dt,err)
        # Derivative
        if dstate is None:
            # calculate raw derivative
            if self.inputIsAngle:
                dest = angleError(state,self.prevstate,self.anglewrap)/dt
            else:
                dest = (state-self.prevstate)/dt
        else:
            dest = dstate
        self.prevstate = state
        # In feedback or forward path?
        if self.derivfeedback:
            # Derivative in feedback
            deriv = -1*dest
        else:
            # Derivative in forward loop
            if self.inputIsAngle:
                dsetpoint = angleError(self.setpoint,self.prevsetpoint,
                                       self.anglewrap)/dt
            else:
                dsetpoint = (self.setpoint-self.prevsetpoint)/dt
            deriv = dsetpoint-dest
        self.prevsetpoint = self.setpoint
        D = self.Kd*self.derivfilter.execute(dt,deriv)
        # Integral 
        self.I = self.I + err*dt
        # Anti-Windup - simple limit on the size of the I contribution
        if (self.Ki > 0.0) and (not self.maxIout is None):
            maxI = self.maxIout/self.Ki
            self.I = saturate(self.I,maxI)
        I = self.Ki*self.I
        
        # Sum
        Out = P+I+D

        return np.array([Out,P,I,D])

