from __future__ import absolute_import, division, print_function, unicode_literals

# TensorFlow and tf.keras
import tensorflow as tf
from tensorflow import keras

# Helper libraries
import numpy as np
import matplotlib.pyplot as plt


# load images
def load_image(filename):
    #load the image
    img = keras.preprocessing.image.load_img(filename, color_mode = "grayscale", target_size=(28,28))
    img = keras.preprocessing.image.img_to_array(img)
    img = img.reshape(1,28,28,1)
    img = img.astype('float32') / 255.0
    return img

def load_model(filename):
    new_model = keras.models.load_model(filename)
    new_model.summary()
    return new_model

path = input("Enter the model path: ")
#path = 'saved_model/my_model'
model = load_model(path)

file = input("Enter the test image name: ")
#file = 'sample_image.png'
img = load_image(file)
predict = model.predict_classes(img)
print(predict)
