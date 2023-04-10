# Raspberry-Pi-based-Face-Recognition-Car-Lock-System
Unlocking your car with your face is now a reality thanks to the power of the Raspberry Pi, Python, and facial recognition technology. With the help of some easy-to-install packages and a few lines of code, you can use your Raspberry Pi to recognize your face and unlock your car without the need for any physical keys.

In this article, we will guide you step-by-step on how to set up your Raspberry Pi to recognize your face and unlock your car.


# Prerequisites:

Before we get started, there are a few things you will need to have:

* A Raspberry Pi 3B+ or higher.

* A camera module for your Raspberry Pi.

* A physical relay module for your Raspberry Pi.

* A USB keyboard and mouse.

* An HDMI display.

* A microSD card with at least 8GB of storage capacity.

* A power supply for your Raspberry Pi.

# Step 1: Update and Install Dependencies

The first step is to update your Raspberry Pi and install the necessary dependencies. Open up the terminal on your Raspberry Pi and type the following commands:
```python
sudo apt-get update

sudo apt-get install python3-opencv

sudo pip3 install face_recognition

sudo apt-get install python3-picamera

sudo apt-get install rpi.gpio
```

The first command updates your Raspberry Pi's package list, while the second command installs OpenCV, a popular computer vision library for Python. The third command installs face_recognition, a Python library that simplifies face recognition. The fourth command installs the Python interface for the Raspberry Pi Camera module, and the final command installs the GPIO library for controlling the relay module.

# Step 2: Prepare the Raspberry Pi Camera

Next, we need to set up the Raspberry Pi camera. To do this, follow these steps:

* Connect the camera module to the Raspberry Pi's CSI port.

* Open up the terminal and type sudo raspi-config.

* Select Interfacing Options and then Camera.

* Choose Yes to enable the camera module.

* Reboot your Raspberry Pi by typing sudo reboot.

# Step 3: Create a Face Model

Before we can use facial recognition to unlock our car, we need to create a face model of the car owner. To do this, follow these steps:

* Take a picture of the car owner's face with the Raspberry Pi camera and save it as "owner.jpg" in the same directory as your Python script.

* Open up the Python script in your text editor of choice and add the following lines of code to load and encode the owner's face:

```python
# Load the image of the car owner
owner_image = face_recognition.load_image_file("owner.jpg")
owner_encoding = face_recognition.face_encodings(owner_image)[0]
```

# Step 4: Set Up the Relay Module

Next, we need to set up the relay module that will unlock the car. To do this, follow these steps:

* Connect the relay module to your Raspberry Pi's GPIO pins. The VCC pin on the relay module should be connected to the 5V pin on the Raspberry Pi, the GND pin on the relay module should be connected to a GND pin on the Raspberry Pi, and the IN pin on the relay module should be connected to GPIO pin 17 on the Raspberry Pi.

* Open up the Python script in your text editor of choice and add the following lines of code to initialize the relay module:

```python
# Set the GPIO mode and output pin number
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
relay_pin = 17
GPIO.setup(relay_pin, GPIO.OUT)
```
# Step 5: Facial Recognition and Car Unlocking

Now that you have all the necessary libraries installed, you can start writing the code to control the car lock.

First, you need to import the required libraries:

```python
import cv2
import face_recognition
import RPi.GPIO as GPIO
import time
from picamera.array import PiRGBArray
from picamera import PiCamera
```
cv2 is the OpenCV library that will be used to capture and process images from the camera. face_recognition is a popular library for face recognition and will be used to compare faces. RPi.GPIO is a Python module that allows control of the GPIO pins on the Raspberry Pi. time is a standard Python library that will be used for waiting. PiRGBArray and PiCamera are the Raspberry Pi Camera libraries.

Next, you will set up the GPIO pin and load the image of the car owner:

```python
# Set the GPIO mode and output pin number
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
relay_pin = 17
GPIO.setup(relay_pin, GPIO.OUT)

# Load the image of the car owner
owner_image = face_recognition.load_image_file("owner.jpg")
owner_encoding = face_recognition.face_encodings(owner_image)[0]
```

Here, you're setting up the GPIO pin to output and assigning it to relay_pin. Then, you're loading the image of the car owner and encoding it using face_recognition.

Now, you will initialize the PiCamera:

```python
# Initialize the PiCamera
camera = PiCamera()

# Set the camera resolution
camera.resolution = (640, 480)

# Set the camera framerate
camera.framerate = 30
```

This initializes the camera, sets the resolution to 640x480, and sets the framerate to 30 frames per second.

You will also initialize the OpenCV video stream:

```python
# Initialize the OpenCV video stream
video_stream = PiRGBArray(camera, size=camera.resolution)
time.sleep(0.1)
```

This initializes the OpenCV video stream using the PiRGBArray class, which allows for fast and efficient processing of the camera stream.

Now, you will loop through the video stream, capture frames, and perform face recognition:

```python
for frame in camera.capture_continuous(video_stream, format="bgr", use_video_port=True):
    # Convert the frame from BGR (OpenCV default) to RGB (face_recognition default)
    rgb_frame = cv2.cvtColor(frame.array, cv2.COLOR_BGR2RGB)

    # Detect faces in the frame
    face_locations = face_recognition.face_locations(rgb_frame)
    face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)

    # Loop through each face in the frame
    for face_encoding, face_location in zip(face_encodings, face_locations):
        # Compare the face encoding with the owner's face encoding
        match = face_recognition.compare_faces([owner_encoding], face_encoding, tolerance=0.5)

        # If the face matches the owner's face, unlock the car
        if match[0]:
            # Trigger the relay to unlock the car
            GPIO.output(relay_pin, GPIO.HIGH)
            print("Car unlocked")
            # Wait for a few seconds
            time.sleep(5)
            # Stop the relay
            GPIO.output(relay_pin, GPIO.LOW)

    # Display the frame
    cv2.imshow("Frame", frame.array)

    # Clear the stream in preparation for the next frame
    video_stream.truncate(0)

    # Wait for a key press
    key = cv2.waitKey(1)
```

# Final Step Code:

```python
import cv2
import face_recognition
import RPi.GPIO as GPIO
import time
from picamera.array import PiRGBArray
from picamera import PiCamera

# Set the GPIO mode and output pin number
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
relay_pin = 17
GPIO.setup(relay_pin, GPIO.OUT)

# Load the image of the car owner
owner_image = face_recognition.load_image_file("owner.jpg")
owner_encoding = face_recognition.face_encodings(owner_image)[0]

# Initialize the PiCamera
camera = PiCamera()

# Set the camera resolution
camera.resolution = (640, 480)

# Set the camera framerate
camera.framerate = 30

# Initialize the OpenCV video stream
video_stream = PiRGBArray(camera, size=camera.resolution)
time.sleep(0.1)

for frame in camera.capture_continuous(video_stream, format="bgr", use_video_port=True):
    # Convert the frame from BGR (OpenCV default) to RGB (face_recognition default)
    rgb_frame = cv2.cvtColor(frame.array, cv2.COLOR_BGR2RGB)

    # Detect faces in the frame
    face_locations = face_recognition.face_locations(rgb_frame)
    face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)

    # Loop through each face in the frame
    for face_encoding, face_location in zip(face_encodings, face_locations):
        # Compare the face encoding with the owner's face encoding
        match = face_recognition.compare_faces([owner_encoding], face_encoding, tolerance=0.5)


        # If the face matches the owner's face, unlock the car
        if match[0]:
            # Trigger the relay to unlock the car
            GPIO.output(relay_pin, GPIO.HIGH)
            print("Car unlocked")
            # Wait for a few seconds
            time.sleep(5)
            # Stop the relay
            GPIO.output(relay_pin, GPIO.LOW)

    # Display the frame
    cv2.imshow("Frame", frame.array)

    # Clear the stream in preparation for the next frame
    video_stream.truncate(0)

    # Wait for a key press
    key = cv2.waitKey(1)

    # If the 'q' key is pressed, exit the loop
    if key == ord('q'):
        break

# Release the video stream and close the window
cv2.destroyAllWindows()

# Clean up the GPIO
GPIO.cleanup()

```

In this tutorial, we've shown you how to use face recognition on a Raspberry Pi to unlock a car. By combining the power of OpenCV, face_recognition library, and a Raspberry Pi with a PiCamera, we've built a simple yet effective system that can detect faces in a video stream and compare them against an owner's face to unlock a car.

While this tutorial focused on unlocking a car, you can use the same principles to develop other applications that use face recognition on a Raspberry Pi, such as smart doorbells, security cameras, and more. The possibilities are endless!
