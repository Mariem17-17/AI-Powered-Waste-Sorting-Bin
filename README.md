# AI-Powered Waste Sorting Bin

## Overview
This project aims to develop an automated waste sorting system using computer vision and machine learning. The system classifies waste items into different compartments of a smart bin, facilitating more efficient waste management. A YOLOv8 model is trained on a custom dataset and integrated with an STM32 microcontroller to perform real-time sorting decisions. A Python interface is also provided to monitor the fullness of each bin compartment.

### Key Technologies:
- **YOLOv8**: Custom object detection model for waste item classification.
- **Roboflow**: Used for dataset annotation
- **STM32 Microcontroller**: For real-time decision-making and control of bin compartments.
- **Serial Communication**: Facilitates communication between the computer vision model and the STM32.
- **Python Interface**: To visualize the current status of the bin compartments (e.g., fullness percentage).

---

## Workflow

### 1. Dataset Collection and Preparation
- Custom waste sorting datasets were collected and annotated using Roboflow.
- The dataset includes common waste items like metal, plastic, and others.
- The dataset was augmented to improve model accuracy and trained using the YOLOv8 architecture.

### 2. Training the YOLOv8 Model
- The YOLOv8 model was fine-tuned on the custom dataset using Roboflow.
- Trained the model with parameters optimized for object detection in real-time conditions.
  
### 3. Model Deployment
- The trained YOLOv8 model was deployed on a local machine for real-time inference.
- Python scripts manage the object detection pipeline, capturing waste item images and running inferences.

### 4. Integration with STM32 Microcontroller
- **Serial Communication**: The STM32 microcontroller communicates with the Python object detection pipeline via serial communication. After a waste item is detected and classified, the decision is transmitted to the STM32, which controls the sorting mechanism of the bin.
- **Real-Time Decision Making**: Based on the classification results (paper, plastic, or organic), the STM32 decides which compartment the waste item should be allocated to.

### 5. Monitoring Bin Status
- A Python interface was developed to display the current status of the bin compartments. It shows the fullness percentage of each compartment, which is updated in real-time as waste is sorted.
- The interface also provides alerts when a compartment is full and needs to be emptied.

---

## Installation and Setup

### Hardware Setup:
- STM32 Development Board: Manages the waste sorting system and handles motor control through PWM signals.
- Servo Motors: Two servo motors control the bin rotation and lid movement.
- Distance Sensor: Measures the distance to detect waste and calculates bin fullness.
- UART Communication: The STM32 communicates with a PC or other devices via UART to receive waste classification commands.

### Software Requirements:
- Python 3.9
- YOLOv8 (install via `pip install ultralytics`)
- STM32 firmware configured to control the sorting mechanism based on serial input.
- Python libraries for serial communication: `pyserial`
- Python GUI libraries (optional): `tkinter` or `PyQt`

### Steps:
1. Train the YOLOv8 model.
2. Deploy the model on the local machine and set up serial communication with the STM32 microcontroller.
3. Run the Python interface to monitor bin status.

---

## Usage

### Model Inference:
1. Capture images of waste items using the camera connected to the local system.
2. The YOLOv8 model performs object detection and classifies the waste.
3. The classification results are sent via serial communication to the STM32 microcontroller, which allocates the waste to the correct bin compartment.

### Monitoring:
- The Python interface continuously updates the fullness percentage of each bin compartment, providing real-time status updates.

---

## Future Improvements
- Add more waste categories for improved sorting accuracy.
- Implement cloud-based monitoring for remote status updates.
- Optimize the system to handle larger-scale waste sorting operations."# AI-Powered-Waste-Sorting-Bin" 
"# AI-Powered-Waste-Sorting-Bin" 
