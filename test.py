import yarp

yarp.Network.init()

# Create a remote control board client
options = yarp.Property()
options.put("device", "remote_controlboard")
options.put("remote", "/ergocubSim/left_arm")
options.put("local", "/my_client/left_arm")
arm = yarp.PolyDriver(options)

if not arm.isValid():
    print("Cannot connect to device")
    exit()

# Get position control
pos = arm.viewIPositionControl()
pos.positionMove(0, 20.0)  # Move joint 0 to position 20.0 degrees
arm.close()
