import yarp

# Initialize YARP
yarp.Network.init()

# Create a YARP BufferedPort to read IMU data
imu_port = yarp.BufferedPortBottle()
imu_port.open("/imu_reader")  # Open local port

# Connect to the IMU output (change the source port if necessary)
yarp.Network.connect("/ergocubSim/head/inertials/measures:o", "/imu_reader")

print("Waiting for IMU data...")

try:
    while True:
        bottle = imu_port.read()  # Read data from YARP
        if bottle is not None and bottle.size() >= 4:  # Ensure data is available
            # Parse each component
            acc = [bottle.get(0).asList().get(i).asFloat64() for i in range(3)]
            gyro = [bottle.get(1).asList().get(i).asFloat64() for i in range(3)]
            mag = [bottle.get(2).asList().get(i).asFloat64() for i in range(3)]
            orient = [bottle.get(3).asList().get(i).asFloat64() for i in range(3)]

            # Print formatted IMU data
            print(f"Accelerometer: {acc} m/s²")
            print(f"Gyroscope: {gyro} rad/s")
            print(f"Magnetometer: {mag} µT")
            print(f"Orientation (Roll, Pitch, Yaw): {orient} degrees")
            print("-" * 50)

except KeyboardInterrupt:
    print("\nStopping IMU reader...")
    imu_port.close()
    yarp.Network.fini()
