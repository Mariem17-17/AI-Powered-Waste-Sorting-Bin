import time
import random
import math
import json
import threading
from collections import deque
import serial.tools.list_ports  # Make sure pyserial is installed
import serial
from PIL import Image, ImageDraw, ImageFont
import customtkinter
from ultralytics import YOLO


class CTkPieChart(customtkinter.CTkLabel):
    def __init__(self, master, command=None, values={}, **kwargs):
        self.arc = None
        self.im = Image.new('RGBA', (1000, 1000))
        self.size = kwargs.get('radius') or 200
        self.background = kwargs.get('bg_color') or master.cget("fg_color")
        self.border_width = kwargs.get('border_width') or 0
        self.border_color = kwargs.get('border_color') or customtkinter.ThemeManager.theme["CTkButton"]["border_color"]
        self.width = kwargs.get('line_width') or 20
        self.text_color = kwargs.get('text_color') or None
        self.widget = master
        self.values = {}
        self.command = command
        self.bg = master.cget("fg_color")

        for i in values:
            self.add(tag=i, draw=False, **values[i])

        super().__init__(master, image=self.arc, fg_color=self.background, compound='center', text="")
        self.draw_pie_chart()

    def _set_scaling(self, *args, **kwargs):
        super()._set_scaling(*args, **kwargs)
        self.size = int(self._apply_widget_scaling(self.size))
        self.width = int(self._apply_widget_scaling(self.width))
        self.border_width = int(self._apply_widget_scaling(self.border_width))

    def _set_appearance_mode(self, mode_string):
        super()._set_appearance_mode(mode_string)

    def draw_pie_chart(self, *args):
        width = self.width * 10
        del self.im
        self.im = Image.new('RGBA', (1000, 1000))
        draw = ImageDraw.Draw(self.im)
        draw.arc((0, 0, 990, 990), 0, 360, self.widget._apply_appearance_mode(self.border_color), self.border_width)
        new_angle = -90
        sum_ = sum(value["value"] for value in self.values.values())

        for value in self.values.values():
            old_angle = new_angle
            new_angle = old_angle + (value['value'] / sum_) * 360
            draw.arc((self.border_width, self.border_width, 990 - self.border_width, 990 - self.border_width), old_angle, new_angle, value['color'], width)
            
            if value['color'] == (0, 0, 0, 0):
                draw.arc((self.border_width, self.border_width, 990 - self.border_width, 990 - self.border_width), old_angle, new_angle, "black", 2)
                draw.arc((self.border_width + width, self.border_width + width, 990 - self.border_width - width, 990 - self.border_width - width), old_angle, new_angle, "black", 2)
            
            midpoint_angle = (old_angle + new_angle) / 2
            xn = yn = (900 - self.border_width) / 2
            radians = (990 - self.border_width) / 2
            arc_pos = radians / 3
            textpos = arc_pos / 1.5
            perc = int(value['value'] / sum_ * 100)
            midpoint1_x = xn + (radians - textpos) * math.cos(math.radians(midpoint_angle))
            midpoint1_y = yn + (radians - textpos) * math.sin(math.radians(midpoint_angle))
            draw.text((midpoint1_x, midpoint1_y), text=str(perc) + "%", fill=value['text_color'], font=ImageFont.load_default(size=70))
        
        x0 = width + self.border_width
        x1 = 990 - width - self.border_width
        if x0 > x1:
            x1 = x0
        draw.arc((x0, x0, x1, x1), 0, 360, self.widget._apply_appearance_mode(self.border_color), self.border_width)
        self.arc = customtkinter.CTkImage(self.im.resize((self.size, self.size), Image.LANCZOS), size=(self.size, self.size))
        super().configure(image=self.arc)

    def add(self, tag, value, color=None, text_color=None, draw=True):
        if tag in self.values:
            self.update(tag, value, color, text_color)
            return

        color = color or "#" + ''.join([random.choice('ABCDEF0123456789') for _ in range(6)])
        text_color = text_color or ("black" if self.is_color_too_bright(color) else "white")
        if self.text_color:
            text_color = self.text_color

        self.values.update({tag: {'color': color, 'value': value, 'text_color': text_color}})
        if draw:
            self.draw_pie_chart()

    def delete(self, tag):
        if tag in self.values:
            del self.values[tag]
        self.draw_pie_chart()

    def update(self, tag, value=None, color=None, text_color=None):
        if tag in self.values:
            if value:
                self.values[tag]['value'] = value
            if color:
                self.values[tag]['color'] = color
            if text_color:
                self.values[tag]['text_color'] = text_color
            self.draw_pie_chart()
        super().update()

    def cget(self, param):
        if param == "bg_color":
            return self.background
        if param == "border_color":
            return self.border_color
        if param == "border_width":
            return self.border_width
        if param == "line_width":
            return self.width
        if param == "radius":
            return self.size
        if param == "width":
            return super().winfo_width()
        if param == "height":
            return super().winfo_height()
        if param == "values":
            return self.values
        if param == "text":
            raise ValueError(f"No such parameter: {param}")
        if param == "justify":
            raise ValueError(f"No such parameter: {param}")
        if param == "text_color":
            raise ValueError(f"No such parameter: {param}")
        if param == "text_color_disabled":
            raise ValueError(f"No such parameter: {param}")
        if param == "corner_radius":
            raise ValueError(f"No such parameter: {param}")
        if param == "font":
            raise ValueError(f"No such parameter: {param}")
        if param == "image":
            raise ValueError(f"No such parameter: {param}")

        return super().cget(param)

    def configure(self, **kwargs):
        if "bg_color" in kwargs:
            self.background = kwargs["bg_color"]
            kwargs.update({"fg_color": self.background})
        if "border_color" in kwargs:
            self.border_color = kwargs.pop("border_color")
        if "border_width" in kwargs:
            self.border_width = kwargs.pop("border_width")
        if "radius" in kwargs:
            self.size = kwargs.pop("radius")
        if "values" in kwargs:
            self.values = kwargs.pop("values")
            for i in self.values:
                self.add(tag=i, draw=False, **self.values[i])
        if "line_width" in kwargs:
            self.width = kwargs.pop("line_width")

        super().configure(**kwargs)
        self.draw_pie_chart()

    def is_color_too_bright(self, hex_color, threshold=100):
        if not hex_color.startswith("#"):
            return False

        hex_color = hex_color.lstrip("#")
        r, g, b = tuple(int(hex_color[i:i + 2], 16) for i in (0, 2, 4))
        total = (r + g + b) / 3

        return True if total > threshold else False

    def get(self, tag=None):
        if tag:
            return self.values[tag]
        return self.values


# Load a model
model = YOLO("train20.pt")


# Assuming these are your class names in the same order as your model's output
class_names = ["metal", "plastic", "other"]  # Modify this list as per your classes

# Parameters
stable_threshold = 5  # Number of consecutive frames to be stored in the queue
detection_queue = deque(maxlen=stable_threshold)
typeOfWaste = None

# Function to send the result over the serial port
def send_serial_result(command):
    print("SENDING COMMAND TO PORT ==========================")
    print(f"Detected stable waste COMMAND ******************************** : {command}")
    ports = serial.tools.list_ports.comports()
    serialInst = serial.Serial()
    portsList = [str(onePort) for onePort in ports]

    for onePort in ports:
        print(str(onePort))

    portVar = "COM" + str('14')
    print(portVar)

    serialInst = serial.Serial(portVar, 115200)
    serialInst.write(command.encode('utf-8'))

# Function to receive the JSON string from the serial port
def receive_serial_json():
    print("RECEIVING JSON FROM PORT ==========================")
    ports = serial.tools.list_ports.comports()
    portVar = "COM" + str('14')
    print(portVar)

    serialInst = serial.Serial(portVar, 115200)
    while True:
        if serialInst.in_waiting:
            json_string = serialInst.readline().decode('utf-8').strip()
            print(f"Received JSON string: {json_string}")
            return json.loads(json_string)

# Function to update the charts with the data from the JSON string
def update_charts(data):
    print("Updating charts with data:", data)
    # data = '{}'
    charts[0].update("EMPTY", value=100 - data["plastic"])
    charts[0].update("FULL", value=data["plastic"])

    charts[1].update("EMPTY", value=100 - data["metal"])
    charts[1].update("FULL", value=data["metal"])

    charts[2].update("EMPTY", value=100 - data["other"])
    charts[2].update("FULL", value=data["other"])

# Create the GUI
root = customtkinter.CTk()
frame = customtkinter.CTkFrame(root)
frame.pack(pady=20, padx=20, fill="both", expand=True)

charts = []

# First pie chart
pie_chart = CTkPieChart(frame)
pie_chart.pack(side="left", padx=20)
pie_chart.add("EMPTY", 100, color=(0, 0, 0, 0), text_color="white")
pie_chart.add("FULL", 0.100, color="black", text_color="black")
charts.append(pie_chart)

# Second pie chart
pie_chart1 = CTkPieChart(frame)
pie_chart1.pack(side="left", padx=20)
pie_chart1.add("EMPTY", 100, color=(0, 0, 0, 0), text_color="white")
pie_chart1.add("FULL", 0.100, color="black", text_color="black")
charts.append(pie_chart1)

# Third pie chart
pie_chart2 = CTkPieChart(frame)
pie_chart2.pack(side="left", padx=20)
pie_chart2.add("EMPTY", 100, color=(0, 0, 0, 0), text_color="white")
pie_chart2.add("FULL", 0.100, color="black", text_color="black")
charts.append(pie_chart2)


import time
from collections import deque

def main_process():
    global typeOfWaste

    detection_queue = deque(maxlen=7)  # Create a deque with a maximum length of 7

    # Run inference on the camera feed
    for result in model.predict(source=0, stream=True):
        detected_wastes = None

        # Extract the class name of the detected object
        if result.boxes:
            class_ids = [int(box.cls.item()) for box in result.boxes]  # Convert class IDs to integers
            detected_wastes = [class_names[class_id] for class_id in class_ids]  # Map class IDs to class names
        else:
            detected_wastes = ["no detection"]

        # Add the detections to the queue
        detection_queue.extend(detected_wastes)
        print(f"Current detections: {detected_wastes}")
        print(f"Detection queue: {list(detection_queue)}")

        # Count occurrences of each waste type in the detection queue
        waste_counts = {waste: detection_queue.count(waste) for waste in class_names}
        print(f"Waste counts: {waste_counts}")

        # Check if any waste type has been detected 4 or more times in the last 7 detections
        for waste, count in waste_counts.items():
            if count >= 4:
                final_prediction = waste
                typeOfWaste = final_prediction  # Always update typeOfWaste

                if typeOfWaste == 'plastic':
                    command = '0'
                elif typeOfWaste == 'metal':
                    command = '1'
                elif typeOfWaste == 'other':
                    command = '2'

                send_serial_result(command)
                print(f"Detected stable waste type: {typeOfWaste}")

                # Receive the JSON string from the serial port
                json_data = receive_serial_json()

                # Update the charts with the received data
                update_charts(json_data)

                # Delay to avoid repeated commands
                time.sleep(7)

                # Clear the detection queue
                detection_queue.clear()

                break  # Exit the loop once the command is sent

        # Optionally, display the results
        plot = result.plot()
        print("Plotting results...")
        # Add a delay if necessary to control the frame rate
        time.sleep(0.7)  # Adjust the delay as needed


# Start the main process in a separate thread
main_thread = threading.Thread(target=main_process, daemon=True)
main_thread.start()

# Start the GUI event loop
root.mainloop()