import math
import numpy as np
import scipy.optimize

import ArmMC


arm = ArmMC.LinkedArm3()

print(arm.get_endpoint())
# print(arm.inverse_kinematics([100,200]))
