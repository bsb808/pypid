from std_msgs.msg import Float32
msg = Float32()

import importlib
msgPkg='std_msgs.msg'
msgType='Float64'
msgAttrList=['data']
mod = __import__(msgPkg,fromlist=[msgType])
msg = getattr(mod,msgType)
tmp = msg()
tmp.data = 3.14
val = getattr(tmp,msgAttrList[0])
print val

msgPkg = 'geometry_msgs.msg'
msgType='Twist'
msgAttrList=['linear','x']
mod = __import__(msgPkg,fromlist=[msgType])
msg = getattr(mod,msgType)
tmp = msg()
tmp.linear.x = 1.0

val = tmp
for attr in msgAttrList:
    val = getattr(val,attr)
print val
