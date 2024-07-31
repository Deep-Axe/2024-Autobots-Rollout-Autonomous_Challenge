# Import necessary libraries
import torch
from yolov5 import YOLOv5
from PIL import Image
import matplotlib.pyplot as plt

# Function to run inference
def run_inference(image, model_path='best.pt', img_size=640):
    # Load the model
    model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path, force_reload=True)

    # Perform inference
    results = model(image, size=img_size)

    # Print the results
    results.print()

# Example usage
run_inference('path/to/your/image.jpg', model_path='best.pt')

'''
from roboflow import Roboflow
rf = Roboflow(api_key="API_KEY")
project = rf.workspace().project("MODEL_ENDPOINT")
model = project.version(VERSION).model

# infer on a local image
print(model.predict("your_image.jpg", confidence=40, overlap=30).json())
'''