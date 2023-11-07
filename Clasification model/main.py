#desde el repositorio https://github.com/shukur-alom/leaf-diseases-detect con algunas modificaciones
from tensorflow.keras.models import load_model
from tensorflow.keras.preprocessing.image import load_img,img_to_array
import numpy as np
import cv2
import os

leaf_deases_model =load_model(os.path.abspath('/home/sofia/Proyecto_final/modeloClasificacion/leaf-diseases-detect/leaf-diseases-detect-main/Leaf Deases(96,88).h5'))
#leaf_deases_model = load_model('/home/sofia/Proyecto_final/modeloClasificacion/leaf-diseases-detect/leaf-diseases-detect-main/Leaf_Deases(95,88).h5')

label_name = ['Apple scab','Apple Black rot', 'Apple Cedar apple rust', 'Apple healthy', 'Cherry Powdery mildew',
'Cherry healthy','Corn Cercospora leaf spot Gray leaf spot', 'Corn Common rust', 'Corn Northern Leaf Blight','Corn healthy', 
'Grape Black rot', 'Grape Esca', 'Grape Leaf blight', 'Grape healthy','Peach Bacterial spot','Peach healthy', 'Pepper bell Bacterial spot', 
'Pepper bell healthy', 'Potato Early blight', 'Potato Late blight', 'Potato healthy', 'Strawberry Leaf scorch', 'Strawberry healthy',
'Tomato Bacterial spot', 'Tomato Early blight', 'Tomato Late blight', 'Tomato Leaf Mold', 'Tomato Septoria leaf spot',
'Tomato Spider mites', 'Tomato Target Spot', 'Tomato Yellow Leaf Curl Virus', 'Tomato mosaic virus', 'Tomato healthy']
                   

#path = input('Imag Path')

path= 'maiz.JPG'

img = img_to_array(load_img(path,target_size=(150,150,3)))

pridict_image = leaf_deases_model.predict( img.reshape((1,) + img.shape ))

print(f"{label_name[np.argmax(pridict_image)]} {pridict_image[0][np.argmax(pridict_image)]*100}%")
predicted_label = f"{label_name[np.argmax(pridict_image)]}" # Replace this with your predicted label

# Load and preprocess the input image (replace this with your own image loading code)
input_image_path = path
input_image = cv2.imread(input_image_path)
input_image = cv2.resize(input_image, (400, 400))  # Resize to a suitable size

# Create a blank image to display the label
label_image = np.zeros((100, 400, 3), dtype=np.uint8)
label_image.fill(255)  # White background

# Add text to the label image
font = cv2.FONT_HERSHEY_SIMPLEX
font_scale = 0.4
font_color = (0, 0, 0)  # Black color
font_thickness = 1
text = f'Predicted Label: {predicted_label}'
text_size = cv2.getTextSize(text, font, font_scale, font_thickness)[0]
text_x = (label_image.shape[1] - text_size[0]) // 2
text_y = (label_image.shape[0] + text_size[1]) // 2
cv2.putText(label_image, text, (text_x, text_y), font, font_scale, font_color, font_thickness)

# Create a composite image by stacking the input image and label image vertically
composite_image = np.vstack((input_image, label_image))

# Create a window and display the composite image
cv2.imshow('Input Image and Predicted Label', composite_image)

# Wait for a key press and then close the window
cv2.waitKey(0)