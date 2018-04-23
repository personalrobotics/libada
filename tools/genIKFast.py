import openravepy 
from openravepy import ikfast
from openravepy import RaveCreateModule
env = openravepy.Environment()

urdf_address = '/home/kayke/dev/my-workspace/src/ada_description/robots/ada.urdf'
srdf_address = '/home/kayke/dev/my-workspace/src/ada_description/robots/ada.srdf'

with env:
	module = RaveCreateModule(env,'urdf')
	name = module.SendCommand('load ' + urdf_address + ' ' + srdf_address)
	kinbody = env.GetRobot(name)
solver = ikfast.IKFastSolver(kinbody=kinbody)

_baselink = 0
_eelink = 1
print("Please verify that the links are correct.")
print("1. Current links")
print("  baselink " + kinbody.GetLinks()[_baselink].GetName())
print("  eelink " + kinbody.GetLinks()[_eelink].GetName())
print("2. If things are wrong")
print("  call kinbody.GetLinks()")
print("  call _baselink=123456, _eelink=1234, etc")
print("  verify by calling kinbody.GetLinks()[_baselink].GetName()")
print("3. exit to exit")
import IPython; IPython.embed()

chaintree = solver.generateIkSolver(baselink=_baselink,eelink=_eelink, freeindices=[-1],solvefn=ikfast.IKFastSolver.solveFullIK_6D)
code = solver.writeIkSolver(chaintree)
open('ik.cpp','w').write(code)
