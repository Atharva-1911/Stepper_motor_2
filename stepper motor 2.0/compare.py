import serial
import threading
import matplotlib.pyplot as plt

# initialize serial at com9 and baudrate
# create empty lists to get data from serialmonitor in it 

ser = serial.Serial('COM9', 115200, timeout=1)
angle_data, time_data, setpoint_data = [], [], []

plt.ion()
fig, ax = plt.subplots()

#function to read from serial
def read():
    global angle_data, time_data, setpoint_data
    
    while True:
        try:
            line = ser.readline().decode('utf-8').strip()
            angle, error, time_val, sp = map(float, line.split('\t'))
            
            angle_data.append(angle)
            time_data.append(time_val)
            setpoint_data.append(sp)
        except:
            pass

#function to write setpoint on serial 
def write():
    while True:
        try:
            value = float(input("Enter setpoint: "))
            ser.write((str(value) + '\n').encode())
        except:
            pass

# function for plotting the graph
def plot():
    global angle_data, time_data, setpoint_data
    
    while True:
        if len(angle_data) > 0:
            ax.clear()
            ax.plot(time_data, angle_data, label="Angle")
            ax.plot(time_data, setpoint_data, '--', label="Setpoint")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Angle")
            ax.set_title("Motor Control")
            ax.legend()
        plt.pause(0.05)


threading.Thread(target=read, daemon=True).start()
threading.Thread(target=write, daemon=True).start()
plot()