import serial
import time
import math
import sys
import subprocess

# Constants for the complementary filter
alpha = 0.75
dt = 0.02  # Time step (50 Hz)
velocity_threshold = 0.05  # Threshold for zero-velocity update (m/s^2)

# Initialize variables for angles
roll = 0.0
pitch = 0.0
yaw = 0.0

# Initialize velocity and distance variables
VX = 0.0
VY = 0.0
VZ = 0.0
DX = 0.0
DY = 0.0
DZ = 0.0

g_force = 9.8
g_unit = 1  # Gravity constant (m/s^2)

# Serial update function
def update_variable():
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
                    dt = ET / 1000

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

                    remove_g()
                    calculate_distance()
                except Exception as e:
                    print("Error occurred: ", e)
    except KeyboardInterrupt:
        print("Exiting the program.")
    finally:
        ser.close()

def acc_invert(acc):
    if acc > 0:
        return -1
    else:
        return 1

def clear_screen():
    operating_system = sys.platform

    if operating_system == "win32":
        subprocess.run('cls', shell=True)
    elif operating_system == "linux" or operating_system == "darwin":
        subprocess.run('clear', shell=True)

def remove_g():
    global AccX, AccY, AccZ
    global roll, pitch
    global AccX_noG, AccY_noG, AccZ_noG

    # Calculate the gravity components on each axis
    gX = g_unit * math.sin(math.radians(roll))
    gY = -g_unit * math.sin(math.radians(pitch))
    gZ = g_unit * math.cos(math.radians(roll)) * math.cos(math.radians(pitch))

    # Remove the gravity component from the accelerometer data
    AccX_noG = round((AccX + gX) * g_force, 1)
    AccY_noG = round((AccY + gY) * g_force, 1)
    AccZ_noG = round((AccZ - gZ) * g_force, 1)

    # Print the accelerometer data without gravity
    # print(f'Acc_noG:\t{AccX_noG}\t{AccY_noG}\t{AccZ_noG}\tRoll/Pitch/Yaw\t{roll:.2f}\t{pitch:.2f}\t{yaw:.2f}\tRawAcc\t\t{gX:.2f}\t{gY:.2f}\t{gZ:.2f}\tET:{ET}', end="\r")

def calculate_distance():
    global VX, VY, VZ
    global DX, DY, DZ

    # Low-pass filter for acceleration to reduce noise
    AccX_filtered = alpha * (AccX_noG / 100) + (1 - alpha) * (VX / dt)
    AccY_filtered = alpha * (AccY_noG / 100) + (1 - alpha) * (VY / dt)
    AccZ_filtered = alpha * (AccZ_noG / 100) + (1 - alpha) * (VZ / dt)

    # Zero-velocity update (ZUPT)
    if abs(AccX_filtered) < velocity_threshold:
        VX = 0
    else:
        VX = VX + AccX_filtered * dt

    if abs(AccY_filtered) < velocity_threshold:
        VY = 0
    else:
        VY = VY + AccY_filtered * dt

    if abs(AccZ_filtered) < velocity_threshold:
        VZ = 0
    else:
        VZ = VZ + AccZ_filtered * dt

    # Integrate velocity to get distance
    DX += VX * dt
    DY += VY * dt
    DZ += VZ * dt

    print(f'Acc_noG:\t{AccX_noG}\t{AccY_noG}\t{AccZ_noG}\tVX:{VX:.2f}\tVY:{VY:.2f}\tVZ:{VZ:.2f}\tDX:{DX:.2f}\tDY:{DY:.2f}\tDZ:{DZ:.2f}')

if __name__ == '__main__':
    # Open the serial port
    ser = serial.Serial('COM13', 115200, timeout=1)
    
    # Allow some time for the serial connection to initialize
    time.sleep(1)
    currentTime = round(time.time() * 1000)

    while True:
        update_variable()
