import serial
import time
import math

# Constants for the complementary filter
alpha = 0.98
dt = 0.05  # Time step (100 Hz)

# Initialize variables for angles
roll = 0.0
pitch = 0.0
yaw = 0.0

# Serial update function
def update_serial():
    global AccX, AccY, AccZ
    global GyroX, GyroY, GyroZ
    global currentTime, previousTime, ET
    global roll, pitch, yaw

    try:
        while True:
            if ser.in_waiting > 0:
                try:
                    previousTime = currentTime
                    currentTime = round(time.time() * 1000)
                    ET = currentTime - previousTime

                    dataStreaming = ser.readline().decode('utf-8').rstrip()
                    dataStreamingArray = dataStreaming.split(',')

                    AccX = float(dataStreamingArray[0])
                    AccY = float(dataStreamingArray[1])
                    AccZ = float(dataStreamingArray[2])

                    GyroX = float(dataStreamingArray[3])
                    GyroY = float(dataStreamingArray[4])
                    GyroZ = float(dataStreamingArray[5])

                    # Calculate angles from accelerometer data
                    acc_pitch = math.atan2(AccY, AccZ) * 180 / math.pi
                    acc_roll = math.atan2(-AccX, math.sqrt(AccY**2 + AccZ**2)) * 180 / math.pi

                    # Integrate gyroscope data to get angles
                    gyro_pitch = pitch + GyroX * dt
                    gyro_roll = roll + GyroY * dt
                    gyro_yaw = yaw + GyroZ * dt

                    # Apply complementary filter
                    pitch = alpha * gyro_pitch + (1 - alpha) * acc_pitch
                    roll = alpha * gyro_roll + (1 - alpha) * acc_roll
                    yaw = gyro_yaw  # Yaw is typically not corrected with accelerometer data

                    print(f'AccX:{AccX}\tAccY:{AccY}\tAccZ:{AccZ}\tGyroX:{GyroX}\tGyroY:{GyroY}\tGyroZ:{GyroZ}\tRoll:{round(roll,2)}\tPitch:{round(pitch,2)}\tYaw:{round(yaw,2)}')
                except Exception as e:
                    print("Error occurred: ", e)
    except KeyboardInterrupt:
        print("Exiting the program.")
    finally:
        ser.close()

if __name__ == '__main__':
    # Open the serial port
    ser = serial.Serial('COM13', 115200, timeout=1)
    
    # Allow some time for the serial connection to initialize
    time.sleep(1)
    currentTime = round(time.time() * 1000)

    while True:
        update_serial()
