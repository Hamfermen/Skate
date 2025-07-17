import customtkinter as ctk
import serial
import re  # Import regex for parsing

import time
import board
import busio
from enum import Enum, auto
from adafruit_ina219 import INA219


class ProfileState(Enum):
    START_CHECK = auto()
    FINDING_START = auto()
    FINDING_LEFT_PLATO = auto()
    FINDING_RIGHT_PLATO = auto()
    FINDING_FINISH = auto()
    PROFILE_STOP = auto()
    MOVING_ZERO = auto()
    END_PROFILE = auto()


stateProfile = ProfileState.END_PROFILE

# Set up I2C bus and INA219 instance
i2c = busio.I2C(board.SCL, board.SDA)
ina = INA219(i2c)

# Optional: choose calibration (depends on your expected max current/voltage)
# ina.set_calibration_32V_2A()  # default
# ina.set_calibration_32V_1A()
# ina.set_calibration_16V_400mA()


profile_arr_x = [0, 0, 0, 0]
profile_arr_z = [0, 0, 0, 0]
voltage = 0


right_z = 0; left_z = 0; right_x = 0; left_x = 0; left_voltage = 10; right_voltage = 10
x_value = 0

# Function to average current over N samples


def send_volt():
   global voltage
   voltage = ina.bus_voltage  # in volts
   send_command("l1")
   send_command(f"{voltage}")

   app.after(100, send_volt)


def check_prof():
   global left_x, right_x, left_z, right_z, left_voltage, right_voltage, voltage, x_value
   update_laser()
   #print(left_voltage, right_voltage, left_x, right_x, left_z, right_z)
   
   check_profile_fun()
   app.after(500, check_prof)


def update_laser():
   global left_x, right_x, left_z, right_z, left_voltage, right_voltage, voltage, x_value

   left_x = right_x
   left_z = right_z
   left_voltage = right_voltage
   right_voltage = voltage
   right_z = voltage * 6.1648049166
   right_x = x_value


def get_x_from_laser(sign_x, sign_z):
    
    global right_x, right_z, left_x, left_z
    r = 50
    hy = right_z - left_z
    hx = right_x - left_x
    cy = right_z - left_z
    cx = 0
    l = hy * cy / (hy**2 + hx**2)
    vx = (left_x + hx * l) - left_x
    vy = (left_z + hy * l) - right_z
    
    x = -sign_x * vx * r / (vx**2 + vy**2)**0.5 + right_x
    y = -sign_z * vy * r / (vx**2 + vy**2)**0.5 + right_z

    return [x, y]

def check_profile_fun():
	global stateProfile, left_x, right_x, left_z, right_z, left_voltage, right_voltage, voltage, profile_arr_x, profile_arr_z

	if stateProfile == ProfileState.START_CHECK:
		print("start")
		stateProfile = ProfileState.FINDING_START

	elif stateProfile == ProfileState.FINDING_START:

		if left_voltage <= 5:
			profile_arr_x[0], profile_arr_z[0] = get_x_from_laser(1, 1)
			print_debug(0, left_voltage, left_x, left_z, right_z, profile_arr_x[0], profile_arr_z[0])
			stateProfile = ProfileState.FINDING_LEFT_PLATO

	elif stateProfile == ProfileState.FINDING_LEFT_PLATO:

		if abs(left_z - right_z) < 0.1:
			profile_arr_x[1], profile_arr_z[1] = get_x_from_laser(1, 1)
			profile_arr_z[1] = right_z
			print_debug(1, left_voltage, left_x, left_z, right_z, profile_arr_x[1], profile_arr_z[1])
			stateProfile = ProfileState.FINDING_RIGHT_PLATO

	elif stateProfile == ProfileState.FINDING_RIGHT_PLATO:
     
		if abs(left_z - right_z) > 0.2:
			profile_arr_x[2], profile_arr_z[2] = get_x_from_laser(1, 1)
			profile_arr_z[2] = left_z
			print_debug(2, left_voltage, left_x, left_z, right_z, profile_arr_x[2], profile_arr_z[2])
			stateProfile = ProfileState.FINDING_FINISH

	elif stateProfile == ProfileState.FINDING_FINISH:
		if right_voltage >= 5:
			profile_arr_x[3], profile_arr_z[3] = get_x_from_laser(-1, 1)
			print_debug(3, left_voltage, left_x, left_z, right_z, profile_arr_x[3], profile_arr_z[3])
			stateProfile = ProfileState.PROFILE_STOP
   
	elif stateProfile == ProfileState.PROFILE_STOP:
		send_command("e1")
        send_prof()
		stateProfile = ProfileState.END_PROFILE

		return stateProfile == ProfileState.END_PROFILE

def send_prof():
    send_command("q1")
    [send_command(f"{profile_arr_x[i]}") for i in range(0, 3)]
    [send_command(f"{profile_arr_z[i]}") for i in range(0, 3)]

def print_debug(idx, left_v, lx, lz, rz, pos_x, pos_z):
    print("----------------------------")
    print(f"Stage {idx}:")
    print(f"Left Voltage: {left_v}")
    print(f"Left X: {lx}")
    print(f"Left Z: {lz}")
    print(f"Right Z: {rz}")
    print(f"X{idx}: {pos_x}")
    print(f"Z{idx}: {pos_z}")

# UART Configuration
SERIAL_PORT = "/dev/ttyAMA0"
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)
except Exception as e:
    print(f"Error opening port: {e}")
    ser = None

# Function to send data via UART
def send_command(command):
    if ser and ser.is_open:
        ser.write(f"{command}\n".encode())
        #print(f"Sent: {command}")
    else:
        print("UART not available")

# Function to handle button press and release
def on_press(command):
	global stateProfile
	if command == "r1":
		stateProfile = ProfileState.START_CHECK
		app.after(1, check_prof)
		print(stateProfile == ProfileState.START_CHECK)
	send_command(f"{command}")
	print(f"Pressed: {command}")

def on_release(command):
    send_command(f"{command}")
    print(f"Released: {command}")

# Function to read data from UART and update the label
def read_uart():
    global x_value
    if ser and ser.is_open:
        data = ser.readline()  # Read a line from UART
        if data:
            try:
                # Attempt to decode only if the data is not empty
                if isinstance(data, bytes):
                    data = data.decode().strip()  # Decode the bytes to a string and remove extra spaces
                    #print(f"Received data: {data}")  # Debugging line to see the raw data

                    if data:  # Ensure data is not empty
                        pattern = re.compile(r"x:([-+]?\d*\.\d+|\d+),y:([-+]?\d*\.\d+|\d+)")  # Regex pattern for float numbers
                        match = pattern.search(data)  # Try to match the expected format

                        if match:
                            x_value, y_value = match.groups()  # Extract x and y values
                            
                            x_value = float(x_value)  # Convert to float
                            y_value = float(y_value)  # Convert to float
                            output_text = f"X: {x_value}, Y: {y_value}"

                           # print(f"Parsed data: {output_text}")  # Debugging line to see parsed data

                            # Update label in GUI safely in the main thread
                            app.after(0, update_label, output_text)

            except Exception as e:
                print(f"Error decoding data: {e}")

#        else:
            #print("No data received.")
    else:
        print("Serial port not open or unavailable.")

    # Schedule the next read in 100ms
    app.after(100, read_uart)


def update_label(output_text):
    uart_label.configure(text=output_text)  # Update the label in the GUI
    #print(f"Updated label with: {output_text}")  # Debugging label update

# Create GUI
app = ctk.CTk()
app.title("Machine Control")
app.geometry("1024x600")

button_size = (150, 60)  # Increased button size

# Label to display received UART data
uart_label = ctk.CTkLabel(app, text="Waiting for UART data...", font=("Arial", 18))
uart_label.pack(pady=10)

# Profile Selection Buttons
profile_frame = ctk.CTkFrame(app)
profile_frame.pack(pady=10)
ctk.CTkLabel(profile_frame, text="Profile Selection").pack()
for i in range(1, 7):
    btn = ctk.CTkButton(profile_frame, text=str(i), width=button_size[0], height=button_size[1])
    btn.bind("<Button-1>", lambda event, i=i: on_press(f"d{i}"))  # Send profile selection
    btn.bind("<ButtonRelease-1>", lambda event: on_release("d0"))  # Reset on release
    btn.pack(side="left", padx=5)

# Skate Size Selection Buttons
size_frame = ctk.CTkFrame(app)
size_frame.pack(pady=10)

ctk.CTkLabel(size_frame, text="Skate Size Selection").pack()

for i in range(1, 6):
    btn = ctk.CTkButton(size_frame, text=str(i), width=button_size[0], height=button_size[1])
    btn.bind("<Button-1>", lambda event, i=i: on_press(f"p{i}"))  # Send profile selection
    btn.bind("<ButtonRelease-1>", lambda event: on_release("p0"))  # Reset on release
    btn.pack(side="left", padx=5)

# Control Buttons
control_frame = ctk.CTkFrame(app)
control_frame.pack(pady=20)

start_button = ctk.CTkButton(control_frame, text="Start", fg_color="green", width=button_size[0], height=button_size[1])
start_button.bind("<Button-1>", lambda event: on_press("t1"))
start_button.bind("<ButtonRelease-1>", lambda event: on_release("t0"))
start_button.pack(side="left", padx=10)

stop_button = ctk.CTkButton(control_frame, text="Stop", fg_color="red", width=button_size[0], height=button_size[1])
stop_button.bind("<Button-1>", lambda event: on_press("a1"))
stop_button.bind("<ButtonRelease-1>", lambda event: on_release("a0"))
stop_button.pack(side="left", padx=10)

stpr_button = ctk.CTkButton(control_frame, text="Start Profile", fg_color="purple", width=button_size[0], height=button_size[1])
stpr_button.bind("<Button-1>", lambda event: on_press("r1"))
stpr_button.bind("<ButtonRelease-1>", lambda event: on_release("r0"))
stpr_button.pack(side="left", padx=10)


# Positioning Buttons
position_frame = ctk.CTkFrame(app)
position_frame.pack(pady=10)
clamp_stop_button = ctk.CTkButton(position_frame, text="Clamp Stop", width=button_size[0], height=button_size[1])
clamp_stop_button.bind("<Button-1>", lambda event: on_press("c1"))
clamp_stop_button.bind("<ButtonRelease-1>", lambda event: on_release("c0"))
clamp_stop_button.pack(side="left", padx=10)

clamp_open_button = ctk.CTkButton(position_frame, text="Clamp Open", fg_color="orange", width=button_size[0], height=button_size[1])
clamp_open_button.bind("<Button-1>", lambda event: on_press("c2"))
clamp_open_button.bind("<ButtonRelease-1>", lambda event: on_release("c0"))
clamp_open_button.pack(side="left", padx=10)

gozero_button = ctk.CTkButton(position_frame, text="Go to Zero", fg_color="blue", width=button_size[0], height=button_size[1])
gozero_button.bind("<Button-1>", lambda event: on_press("z1"))
gozero_button.bind("<ButtonRelease-1>", lambda event: on_release("z0"))
gozero_button.pack(side="left", padx=10)

sharp_button = ctk.CTkButton(position_frame, text="Sharpen", fg_color="blue", width=button_size[0], height=button_size[1])
sharp_button.bind("<Button-1>", lambda event: on_press("s1"))
sharp_button.bind("<ButtonRelease-1>", lambda event: on_release("s0"))
sharp_button.pack(side="left", padx=10)

dim_button = ctk.CTkButton(position_frame, text="Sharpen Disk", fg_color="blue", width=button_size[0], height=button_size[1])
dim_button.bind("<Button-1>", lambda event: on_press("n1"))
dim_button.bind("<ButtonRelease-1>", lambda event: on_release("n0"))
dim_button.pack(side="left", padx=10)

# Movement Buttons
movement_frame = ctk.CTkFrame(app)
movement_frame.pack(pady=10)
# X -0.2mm
x_neg_button = ctk.CTkButton(movement_frame, text="X -0.2mm", width=button_size[0], height=button_size[1])
x_neg_button.bind("<Button-1>", lambda event: on_press("x1"))
x_neg_button.bind("<ButtonRelease-1>", lambda event: on_release("x0"))
x_neg_button.pack(side="left", padx=10)

# X +0.2mm
x_pos_button = ctk.CTkButton(movement_frame, text="X +0.2mm", width=button_size[0], height=button_size[1])
x_pos_button.bind("<Button-1>", lambda event: on_press("x2"))
x_pos_button.bind("<ButtonRelease-1>", lambda event: on_release("x0"))
x_pos_button.pack(side="left", padx=10)

# Y -0.2mm
y_neg_button = ctk.CTkButton(movement_frame, text="Y -0.2mm", width=button_size[0], height=button_size[1])
y_neg_button.bind("<Button-1>", lambda event: on_press("y1"))
y_neg_button.bind("<ButtonRelease-1>", lambda event: on_release("y0"))
y_neg_button.pack(side="left", padx=10)

# Y +0.2mm
y_pos_button = ctk.CTkButton(movement_frame, text="Y +0.2mm", width=button_size[0], height=button_size[1])
y_pos_button.bind("<Button-1>", lambda event: on_press("y2"))
y_pos_button.bind("<ButtonRelease-1>", lambda event: on_release("y0"))
y_pos_button.pack(side="left", padx=10)

# Start reading UART data every 100ms
app.after(100, read_uart)
app.after(1000, send_volt)


# Start main loop
app.mainloop()

# Close UART on exit
if ser:
    ser.close()
