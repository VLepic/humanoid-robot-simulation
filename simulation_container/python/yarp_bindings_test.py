import yarp

# Initialize YARP
yarp.Network.init()

# Create a YARP BufferedPort to read IMU data
imu_port = yarp.BufferedPortBottle()
if not imu_port.open("/imu_reader"):
    print("Failed to open YARP port")

# Connect to the IMU output (change the source port if necessary)
yarp.Network.connect("/ergocubSim/head/inertials/measures:o", "/imu_reader")

print("Waiting for IMU data...")

try:
    while True:
        bottle = imu_port.read()  # Read data from YARP
        if bottle is None:
            print("No data received")
        else:
            #print(f"Received data: {bottle.toString()}")
            acc_list = bottle.get(0)
            if acc_list.isList():
                acc_list = acc_list.asList()  # Convert to Bottle

                # The first element of this list should be another list containing x, y, z
                inner_list = acc_list.get(0)

                if inner_list.isList():
                    timestamp = inner_list.asList().get(1).asFloat64()
                    values = inner_list.asList().get(0).asList()
                    acc_values = [values.get(i).asFloat64() for i in range(values.size())]
                    print(f"Accelerometer: {acc_values} m/s² timestamped: {timestamp}")
                else:
                    print("Unexpected format for accelerometer data!")







except KeyboardInterrupt:
    print("\nStopping IMU reader...")
    imu_port.close()
    yarp.Network.fini()
