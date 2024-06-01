import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
import time
import threading

# Initialize data lists
times = []
values = []

# Shared variable to hold the latest AccX value
shared_data = {"AccX": 0}

# Set up the figure and axis
fig, ax = plt.subplots()
fig.suptitle('AccX Plot')

def animate(i):
    current_time = time.time()
    
    # Read the latest AccX value from the shared variable
    value = shared_data["AccX"]
    
    # Append new data
    times.append(current_time)
    values.append(value)
    
    # Limit lists to a fixed number of items (e.g., last 100 items)
    times_to_plot = times[-100:]
    values_to_plot = values[-100:]
    
    # Clear the previous plot
    ax.clear()
    
    # Update plot limits
    if times_to_plot:
        ax.set_xlim(times_to_plot[0], times_to_plot[-1])
    
    # Plot new data
    ax.plot(times_to_plot, values_to_plot, label='AccX Value')
    ax.set_ylabel('Value')
    ax.set_xlabel('Time (s)')
    ax.legend(loc='upper left')

def readSerial(port='COM13', baudrate=115200, timeout=1):
    # Open the serial port
    ser = serial.Serial(port, baudrate, timeout=timeout)
    
    # Allow some time for the serial connection to initialize
    time.sleep(1)
    try:
        while True:
            if ser.in_waiting > 0:
                dataStreaming = ser.readline().decode('utf-8').rstrip()
                dataStreamingArray = dataStreaming.split(',')

                AccX = float(dataStreamingArray[0])
                AccY = float(dataStreamingArray[1])
                AccZ = float(dataStreamingArray[2])
                GyroX = float(dataStreamingArray[3])
                GyroY = float(dataStreamingArray[4])
                GyroZ = float(dataStreamingArray[5])
                
                # Update the shared variable
                shared_data["AccX"] = AccX
                
                print(f"AccX: {AccX}")
    except KeyboardInterrupt:
        print("Exiting the program.")
    finally:
        ser.close()

# Function to start the serial reading in a separate thread
def start_reading():
    read_thread = threading.Thread(target=readSerial)
    read_thread.daemon = True
    read_thread.start()

if __name__ == '__main__':
    # Start the serial reading thread
    start_reading()
    
    # Create an animation
    ani = animation.FuncAnimation(fig, animate, interval=0.02)
    
    # Show the plot
    plt.show()
