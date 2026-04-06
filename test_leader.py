from pyAgxArm import create_agx_arm_config, AgxArmFactory
import time

config = create_agx_arm_config(robot='piper', comm='can', channel='can0')
arm = AgxArmFactory.create_arm(config)
arm.connect()

print("Setting arm to leader mode (drag mode)...")
arm.set_leader_mode()
time.sleep(1)
arm.disconnect()
print("Done")
