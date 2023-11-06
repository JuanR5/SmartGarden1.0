import cv2
import numpy as np
from tensorflow.keras.models import load_model
from tensorflow.keras.preprocessing.image import img_to_array
from tensorflow.keras.applications.mobilenet_v2 import preprocess_input
import os

# Load the pre-trained model
leaf_disease_model = load_model('/home/sofia/Proyecto_final/modeloClasificacion/leaf-diseases-detect/leaf-diseases-detect-main/Leaf Deases(96,88).h5')

# Define class labels
label_names = ['Apple scab', 'Apple Black rot', 'Apple Cedar apple rust', 'Apple healthy', 'Cherry Powdery mildew',
              'Cherry healthy', 'Corn Cercospora leaf spot Gray leaf spot', 'Corn Common rust', 'Corn Northern Leaf Blight',
              'Corn healthy', 'Grape Black rot', 'Grape Esca', 'Grape Leaf blight', 'Grape healthy', 'Peach Bacterial spot',
              'Peach healthy', 'Pepper bell Bacterial spot', 'Pepper bell healthy', 'Potato Early blight', 'Potato Late blight',
              'Potato healthy', 'Strawberry Leaf scorch', 'Strawberry healthy', 'Tomato Bacterial spot', 'Tomato Early blight',
              'Tomato Late blight', 'Tomato Leaf Mold', 'Tomato Septoria leaf spot', 'Tomato Spider mites', 'Tomato Target Spot',
              'Tomato Yellow Leaf Curl Virus', 'Tomato mosaic virus', 'Tomato healthy']

# Open a connection to the webcam (0 represents the default camera, you can change it if needed)
cap = cv2.VideoCapture(0)

# Set the window size (you can change these values to make the window larger)
window_width = 500
window_height = 500

# Create the window
cv2.namedWindow('Leaf Disease Classification', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Leaf Disease Classification', window_width, window_height)

# Set the font properties for the text
font = cv2.FONT_HERSHEY_SIMPLEX
font_scale = 0.3  # Adjust the font size (you can change this value)
font_color = (0, 0, 0)  # Green color
font_thickness = 1  # Adjust the font thickness (you can change this value)

while True:
    # Read a frame from the webcam
    ret, frame = cap.read()

    # Ensure the frame is successfully read
    if not ret:
        break

    # Preprocess the frame (resize to 150x150 and convert to array)
    frame = cv2.resize(frame, (150, 150))
    frame = img_to_array(frame)
    frame = preprocess_input(frame)

    # Make a prediction using the model
    prediction = leaf_disease_model.predict(np.expand_dims(frame, axis=0))

    # Get the predicted class label and confidence
    predicted_class = label_names[np.argmax(prediction)]
    confidence = prediction[0][np.argmax(prediction)] * 100

    # Display the predicted label and confidence on the frame
    label_text = f"{predicted_class} ({confidence:.2f}%)"
    cv2.putText(frame, label_text, (10, 30), font, font_scale, font_color, font_thickness)

    # Display the frame with the label
    cv2.imshow('Leaf Disease Classification', frame)

    # Exit the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()