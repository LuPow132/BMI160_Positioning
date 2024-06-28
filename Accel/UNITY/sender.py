import serial
import time
import math
import sys, subprocess
import keyboard
import socket

# Constants for the complementary filter
alpha = 0.8
dt = 0  # Time step (100 Hz)

# Initialize variables for angles
roll = 0.0
pitch = 0.0
yaw = 0.0

g_force = 9.8
g_unit = 1  # Gravity constant (m/s^2)

VX = 0.0
VY = 0.0
VZ = 0.0

DX = 0.0
DY = 0.0
DZ = 0.0

# Serial update function
def update_variable():
    global AccX, AccY, AccZ
    global GyroX, GyroY, GyroZ
    global currentTime, previousTime, ET, dt
    global roll, pitch, yaw
    global VX, VY, VZ
    global DX, DY, DZ

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
                    acc_roll = math.atan2(-AccX, math.sqrt(AccY ** 2 + AccZ ** 2)) * 180 / math.pi

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

                    # Send data to Unity
                    send_data_to_unity(DX, DY, DZ, roll, pitch, yaw)

                    if keyboard.is_pressed("space"):
                        VX = 0.0
                        VY = 0.0
                        VZ = 0.0

                        DX = 0.0
                        DY = 0.0
                        DZ = 0.0
                        clear_screen()
                        print("RESET")
                except Exception as e:
                    print("Error occurred: ", e)
    except KeyboardInterrupt:
        print("Exiting the program.")
    finally:
        ser.close()

def acc_invert(acc):
    return -1 if acc > 0 else 1

def clear_screen():
    operating_system = sys.platform

    if operating_system == "win32":
        subprocess.run('cls', shell=True)
    elif operating_system in ["linux", "darwin"]:
        subprocess.run('clear', shell=True)

def remove_g():
    global AccX, AccY, AccZ
    global roll, pitch
    global AccX_noG, AccY_noG, AccZ_noG

    global AccX, AccY, AccZ
    global roll, pitch

    # Calculate the gravity components on each axis
    gX = g_unit * math.sin(math.radians(roll))
    gY = -g_unit * math.sin(math.radians(pitch))
    gZ = g_unit * math.cos(math.radians(roll)) * math.cos(math.radians(pitch))

    # Remove the gravity component from the accelerometer data
    AccX_noG = (AccX + gX) * g_force
    AccY_noG = (AccY + gY) * g_force
    AccZ_noG = (AccZ - gZ) * g_force

def calculate_distance():
    global VX, VY, VZ
    global DX, DY, DZ

    if abs(AccX_noG) < 0.8:
        VX = 0
    else:
        VX = round(VX + AccX_noG * dt * 1000)

    if abs(AccY_noG) < 0.8:
        VY = 0
    else:
        VY = round(VY + AccY_noG * dt * 1000)

    if abs(AccZ_noG) < 0.8:
        VZ = 0
    else:
        VZ = round(VZ + AccZ_noG * dt * 1000)

    DX += round(round(VX * dt) / 100, 2)
    DY += round(round(VY * dt) / 100, 2)
    DZ += round(round(VZ * dt) / 100, 2)

    print(f'\t{AccX_noG:.2f}\t{AccY_noG:.2f}\t{AccZ_noG:.2f}\t{VX}\t{VY}\t{VZ}\t{roll:.2f}\t{pitch:.2f}\t{yaw:.2f}\t{DX:.2f}\t{DY:.2f}\t{DZ:.2f}\t{dt}')

def send_data_to_unity(dx, dy, dz, roll, pitch, yaw):
    try:
        message = f"{dx},{dy},{dz},{roll},{pitch},{yaw}"
        client_socket.sendall(message.encode('utf-8'))
    except Exception as e:
        print(f"Failed to send data: {e}")

if __name__ == '__main__':
    # Open the serial port
    ser = serial.Serial('COM5', 115200, timeout=1)

    # Allow some time for the serial connection to initialize
    time.sleep(1)
    currentTime = round(time.time() * 1000)

    # Set up the TCP connection to Unity
    HOST = '127.0.0.1'  # Unity is running on the same machine
    PORT = 25001        # Port should match Unity script

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((HOST, PORT))

    try:
        while True:
            update_variable()
    except KeyboardInterrupt:
        print("Exiting the program.")
    finally:
        ser.close()
        client_socket.close()
