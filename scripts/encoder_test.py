import os
import sys
import numpy as np

sys.path.append(os.path.abspath('.'))

from pyglonax.robot import MockEncoder

encoder_slew = MockEncoder(0x6A, 1000, -np.inf, np.inf)
encoder_boom = MockEncoder(0x6B, 1000, np.deg2rad(0), np.deg2rad(60+45))
encoder_arm = MockEncoder(0x6C, 1000, np.deg2rad(0), np.deg2rad(120-40))
encoder_attachment = MockEncoder(0x6D, 1000, np.deg2rad(0), np.deg2rad(120))

encoder_boom.value = np.deg2rad(45)

print("Boom:", encoder_boom.value)
print("Boom:", encoder_boom.decode())
