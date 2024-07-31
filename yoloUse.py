# Import necessary packages
from ultralytics import YOLO
from PIL import Image

# Load the trained model
model = YOLO('yolov5_stop_sign.pt')

# Define a function to detect stop signs
def detect_stop_sign(image_path):
    # Load image
    image = Image.open(image_path)

    # Perform inference
    results = model(image)

    # Print results
    results.print()  # Print the results (e.g., bounding boxes and confidence scores)

    # Plot results
    #results.show()  # Show the image with detected objects

    # Save results
    #results.save(save_dir='path/to/save/detections')  # Directory to save annotated images

# Example usage
detect_stop_sign('path/to/image.jpg')
