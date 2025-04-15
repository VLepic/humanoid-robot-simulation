import yarp

# Initialize YARP
yarp.Network.init()

# Create a YARP BufferedPort to read IMU data
imu_port = yarp.BufferedPortBottle()
if not imu_port.open("/imu_reader"):
    print("Failed to open YARP port")

# Create a YARP BufferedPort to read LIDAR data
lidar_port = yarp.BufferedPortBottle()
if not lidar_port.open("/lidar_reader"):
    print("Failed to open YARP port")

# Connect to the IMU output (change the source port if necessary)
yarp.Network.connect("/ergocubSim/head/inertials/measures:o", "/imu_reader")

# Connect to the LIDAR output (change the source port if necessary)
yarp.Network.connect("/ergocubSim/laser:o", "/lidar_reader")

print("Waiting for data...")

acc = []
gyro = []
mag = []
orient = []

def extract(acc_list):
    if acc_list.isList():
        acc_list = acc_list.asList()  # Convert to Bottle

        # The first element of this list should be another list containing x, y, z
        inner_list = acc_list.get(0)

        if inner_list.isList():
            timestamp = inner_list.asList().get(1).asFloat64()
            values = inner_list.asList().get(0).asList()
            acc_values = [values.get(i).asFloat64() for i in range(values.size())]
            return acc_values, timestamp
        else:
            print("Unexpected format for accelerometer data!")

try:
    while True:
        imu_bottle = imu_port.read()  # Read data from YARP
        lidar_bottle = lidar_port.read()

        if imu_bottle is None:
            print("No IMU data received")
        if lidar_bottle is None:
            print("No LIDAR data received")
        else:
            print(f"Received lidar data: {lidar_bottle.toString()}")

            # Parse each component
            starting_angle = lidar_bottle.get(0).asFloat64()
            end_angle = lidar_bottle.get(1).asFloat64()
            min_range = lidar_bottle.get(2).asFloat64()
            max_range = lidar_bottle.get(3).asFloat64()
            lidar_measurements = [lidar_bottle.get(4).asList().get(i).asFloat64() for i in range(lidar_bottle.get(4).asList().size())]


            # Parse each component
            gyro, gyro_timestamp = extract(imu_bottle.get(0))
            acc, acc_timestamp = extract(imu_bottle.get(1))
            mag, mag_timestamp = extract(imu_bottle.get(2))
            orient, orient_timestamp = extract(imu_bottle.get(3))

            # Print formatted IMU data
            print("*" * 50)
            print("-" * 20 + "LIDAR DATA" + "-" * 20)
            print(f"Start angle: {starting_angle} ")
            print(f"End angle: {end_angle} ")
            print(f"Min range: {min_range} ")
            print(f"Max range: {max_range} ")
            print(f"LIDAR measured ranges: {lidar_measurements} ")
            print(f"Number of LIDAR measured ranges: {len(lidar_measurements)} ")
            print("*" * 50)
            print("-" * 21 + "IMU DATA" + "-" * 21)
            print(f"Accelerometer: {acc} m/s² timestamp: {acc_timestamp}")
            print(f"Gyroscope: {gyro} rad/s timestamp: {gyro_timestamp}")
            print(f"Magnetometer: {mag} µT timestamp: {mag_timestamp}")
            print(f"Orientation (Roll, Pitch, Yaw): {orient} degrees timestamp: {orient_timestamp}")
            print("*" * 50)






except KeyboardInterrupt:
    print("\nStopping IMU reader...")
    imu_port.close()
    lidar_port.close()
    yarp.Network.fini()
