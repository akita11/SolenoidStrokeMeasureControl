# pip install pytk
# brew install python-tk & brew install tcl-tk
# https://wp.tekapo.com/2021/07/17/when-no-module-named-_tkinter/
import sys
import tkinter as tk
from tkinter import ttk
import threading
import serial
import time
import numpy as np
import tensorflow as tf
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import math

data = []
dataT = []

COM_PORT = "/dev/tty.usbserial-AL01FTOB"
#COM_PORT = "COM9"

ser = serial.Serial(COM_PORT, 115200, timeout=0.1)
def sendCmd(cmd):
    cmd = cmd + '\n'
    s = cmd.encode()
    ser.write(s)

#0.5ms -> 6250
def setPWM(Ton): # in [ms]
    val = int(Ton * 625 * 2)
    sendCmd("D{0:d}".format(val))

def predict(Ton, v0, v1):
    #ref https://qiita.com/yohachi/items/434f0da356161e82c242
    #Ton, v0, v1 = 0.5, 93.9, 175.7
    input_data = np.array([[[(Ton - scaler_i_mean[0]) / scaler_i_scale[0]],
                            [(v0  - scaler_i_mean[1]) / scaler_i_scale[1]],
                            [(v1  - scaler_i_mean[2]) / scaler_i_scale[2]]]],
                          dtype=np.float32) # Ton[ms]/v0/v1
    interpreter.set_tensor(input_details[0]['index'], input_data)
    interpreter.invoke()
    output_data = interpreter.get_tensor(output_details[0]['index'])
    pos_conv = output_data[0][0]
    temp_conv = output_data[0][1]
    pos  = pos_conv * scaler_o_scale[0] + scaler_o_mean[0]
    temp = temp_conv * scaler_o_scale[1] + scaler_o_mean[1]
    return(pos, temp)

# setup TensorFlow
interpreter = tf.lite.Interpreter(model_path="model_SSBH_200Hz.tflite")
interpreter.allocate_tensors()

scaler_i_mean = [ 2.5, 266.79358974358973, 309.56004273504266 ]
scaler_i_scale = [ 1.2909944487358056, 133.12755355096166, 120.38504041075464 ]
scaler_o_mean = [ 3.0, 32.5 ]
scaler_o_scale = [ 1.8708286933869707, 5.5901699437494745 ]

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

Kp = 0.1
posT = 0.5
Kp_min, Kp_max = 0.01, 1.0
posT_min, posT_max = 0.0, 5.0

#root = tk.Tk()
#root.title("Solenoid Position Control")
#root.geometry("400x150")
# variables for scale

def setTargetPosition(p):
    global posT
    posT = float(p)
    
def setKp(k):
    global Kp
    Kp = float(k)

# initialize SolenoidStrokeUNIT
sendCmd("D6250")
time.sleep(0.1);
sendCmd("T6250"); # PWM cycle=500us(200Hz)
time.sleep(0.1);
sendCmd("062");   # t0=49.6us(200Hz)
time.sleep(0.1);
sendCmd("1432");  # t1=345.6us(200Hz)

Ton = 1.5
setPWM(Ton)

sendCmd('S')

Ton_MAX = 4.95 # [ms]
Ton_MIN = 0.5  # [ms]

dTon = 0

def position_control(pos, posT):
    global Ton
    dTon = (posT - pos) * Kp
    Ton_t = Ton + dTon
    if Ton_t > Ton_MAX:
        Ton = Ton_MAX
    elif Ton_t < Ton_MIN:
        Ton = Ton_MIN
    else:
        Ton = Ton_t
    setPWM(Ton)

root = tk.Tk()
root.title("Solenoid Position Control")
#root.geometry("600x400")

fig, ax = plt.subplots(figsize=(6,2))
ax.set_xlabel("Time")
ax.set_ylabel("Position[mm]")
line,  = ax.plot([], [], "r-", color="red")
lineT, = ax.plot([], [], "r-", color="green")

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

# Scale for PosT
scale_frame_pos = ttk.Frame(root)
scale_frame_pos.pack(side=tk.LEFT, fill=tk.Y, padx=10)

v_posT = tk.DoubleVar(value=posT_max - posT)
labelPosT = ttk.Label(scale_frame_pos, text=f"Target Position[mm]: {posT_max - v_posT.get():.2f}")
labelPosT.pack()
def update_label_posT(*args):
    labelPosT.config(text=f"Target Position[mm]: {posT_max - v_posT.get():.2f}")
v_posT.trace_add("write", update_label_posT)
scalePosT = ttk.Scale(scale_frame_pos, from_=posT_min, to=posT_max,
                      orient="vertical", variable=v_posT,
                      )
scalePosT.pack(fill=tk.Y, expand=True)

# Scale for Kp
v_Kp = tk.DoubleVar(value=Kp)
labelKp = ttk.Label(scale_frame_pos, text=f"Kp: {v_Kp.get():.2f}")
labelKp.pack(padx=10)
def update_label_Kp(*args):
    global Kp
    labelKp.config(text=f"Kp: {v_Kp.get():.2f}")
    Kp = v_Kp.get()
v_Kp.trace_add("write", update_label_Kp)
scaleKp = ttk.Scale(scale_frame_pos, from_=Kp_min, to=Kp_max,
                    orient="vertical", variable=v_Kp,
                    )
scaleKp.pack(fill=tk.Y, expand=True, padx=10)

fSin = False
Tsin = 0
Tsin_max = 30 # cycle = Tsin_max x100ms
#Asin = 1 # amplitude
#Asin0 = 1.5 # center of sin
#Asin = 1.25 # amplitude
#Asin0 = 1.25 # center of sin
Asin = 1.3 # amplitude
Asin0 = 1.3 # center of sin

bottom_frame = tk.Frame(scale_frame_pos)
bottom_frame.pack(side=tk.BOTTOM, pady=10)

#status_label = ttk.Label(bottom_frame, text="Sin Curve")
#status_label.pack(side=tk.LEFT, padx=5)

v_fSin = tk.BooleanVar(value=False)

def checkbox_toggle():
    global fSin
    if v_fSin.get():
        fSin = True
    else:
        fSin = False

# チェックボックス追加
checkbox = ttk.Checkbutton(bottom_frame, text="Sine Curve", variable=v_fSin, command=checkbox_toggle)
checkbox.pack(side=tk.LEFT)

# draw graph
def update_plot():
    global Tsin, posT
    if data:
        ax.set_xlim(max(0, len(data) - 50), len(data))  # 表示範囲を調整
        ax.set_ylim(0, 6)
        line.set_data(range(len(data)), data)
        lineT.set_data(range(len(dataT)), dataT)
        canvas.draw()
    root.after(100, update_plot) # redraw every 100ms
    if (fSin == True):
        Tsin = (Tsin + 1) % Tsin_max
        posT = Asin0 + Asin * math.sin(float(Tsin) / Tsin_max * 2 * 3.14)

# read data from serial
def read_serial():
    global ser, posT
    while True:
        line0 = ser.readline()
        serial_data = line0.decode()
        #serial_data = "01230456\r\n"
        if (len(serial_data) == 10):
            v0 = float(serial_data[0:4])
            v1 = float(serial_data[4:8])
            pos, temp = predict(Ton, v0, v1)
            if (fSin == False):
                posT = posT_max - v_posT.get()
            position_control(pos, posT)
            #print(Kp, posT, pos, Ton)
            data.append(pos)
            dataT.append(posT)
            if len(data) > 100: # maximum data limit
                data.pop(0)
            if len(dataT) > 100: # maximum data limit
                dataT.pop(0)

# start serial-receive thread
thread = threading.Thread(target=read_serial, daemon=True)
thread.start()

# start graph update
update_plot()

root.mainloop()
