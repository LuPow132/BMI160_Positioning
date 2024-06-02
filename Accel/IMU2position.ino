import serial
import time

class KalmanFilter:
    def __init__(self, process_variance, measurement_variance, estimated_measurement_variance):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimated_measurement_variance = estimated_measurement_variance
        self.posteri_estimate = 0.0
        self.posteri_error_estimate = 1.0

    def update(self, measurement):
        priori_estimate = self.posteri_estimate
        priori_error_estimate = self.posteri_error_estimate + self.process_variance

        blending_factor = priori_error_estimate / (priori_error_estimate + self.measurement_variance)
        self.posteri_estimate = priori_estimate + blending_factor * (measurement - priori_estimate)
        self.posteri_error_estimate = (1 - blending_factor) * priori_error_estimate

        return self.posteri_estimate

def updateSerial():
    global AccX, AccY, AccZ, GyroX, GyroY, GyroZ, currentTime, ET, velocityX, distanceX, velocityY, distanceY, velocityZ, distanceZ
    previousTime = currentTime
    currentTime = round(time.time() * 1000)
    ET = (currentTime - previousTime) / 1000.0  # Convert to seconds

    dataStreaming = ser.readline().decode('utf-8').rstrip()
    dataStreamingArray = dataStreaming.split(',')

    raw_AccX = float(dataStreamingArray[0])
    raw_AccY = float(dataStreamingArray[1])
    raw_AccZ = float(dataStreamingArray[2])
    
    AccX = kf_accX.update(raw_AccX)
    AccY = kf_accY.update(raw_AccY)
    AccZ = kf_accZ.update(raw_AccZ)
    GyroX = kf_gyroX.update(float(dataStreamingArray[3]))
    GyroY = kf_gyroY.update(float(dataStreamingArray[4]))
    GyroZ = kf_gyroZ.update(float(dataStreamingArray[5]))

    # Use only acceleration values greater than 0.1 for calculation
    if abs(AccX) > 0.1:
        velocityX += AccX * ET
    else:
        if velocityX > 0:
            velocityX -= 0.1 * ET
            if velocityX < 0:
                velocityX = 0
        elif velocityX < 0:
            velocityX += 0.1 * ET
            if velocityX > 0:
                velocityX = 0

    distanceX += velocityX * ET

    if abs(AccY) > 0.1:
        velocityY += AccY * ET
    else:
        if velocityY > 0:
            velocityY -= 0.1 * ET
            if velocityY < 0:
                velocityY = 0
        elif velocityY < 0:
            velocityY += 0.1 * ET
            if velocityY > 0:
                velocityY = 0

    distanceY += velocityY * ET

    if abs(AccZ) > 0.1:
        velocityZ += AccZ * ET
    else:
        if velocityZ > 0:
            velocityZ -= 0.1 * ET
            if velocityZ < 0:
                velocityZ = 0
        elif velocityZ < 0:
            velocityZ += 0.1 * ET
            if velocityZ > 0:
                velocityZ = 0

    distanceZ += velocityZ * ET

def main():
    global ser, previousTime, currentTime, kf_accX, kf_accY, kf_accZ, kf_gyroX, kf_gyroY, kf_gyroZ
    global velocityX, distanceX, velocityY, distanceY, velocityZ, distanceZ

    ser = serial.Serial("COM13", 115200, timeout=1)
    currentTime = round(time.time() * 1000)
    
    # Initialize Kalman Filters for each sensor
    kf_accX = KalmanFilter(0.01, 0.1, 0.01)
    kf_accY = KalmanFilter(0.01, 0.1, 0.01)
    kf_accZ = KalmanFilter(0.01, 0.1, 0.01)
    kf_gyroX = KalmanFilter(0.01, 0.1, 0.01)
    kf_gyroY = KalmanFilter(0.01, 0.1, 0.01)
    kf_gyroZ = KalmanFilter(0.01, 0.1, 0.01)

    # Initialize velocity and distance for all axes
    velocityX = 0.0
    distanceX = 0.0
    velocityY = 0.0
    distanceY = 0.0
    velocityZ = 0.0
    distanceZ = 0.0

    try:
        while True:
            if ser.in_waiting > 0:
                try:
                    updateSerial()
                except Exception as e:
                    print(f"Error found: {e}")
                # print(f"AccX: {AccX:.2f}\tAccY: {AccY:.2f}\tAccZ: {AccZ:.2f}\tGyroX: {GyroX:.2f}\tGyroY: {GyroY:.2f}\tGyroZ: {GyroZ:.2f}\tET: {ET:.2f}")
                print(f"VX: {velocityX:.2f}\tDX: {distanceX:.2f}\tVY: {velocityY:.2f}\tDY: {distanceY:.2f}\tVZ: {velocityZ:.2f}\tDZ: {distanceZ:.2f}")

    except KeyboardInterrupt:
        print("Exiting the program.")
    finally:
        ser.close()

if __name__ == '__main__':
    main()
