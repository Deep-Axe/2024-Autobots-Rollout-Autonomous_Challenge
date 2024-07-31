
# Install necessary packages
# !pip install torch torchvision torchaudio
# !pip install ultralytics

from ultralytics import YOLO

# Create the YOLO model
model = YOLO('yolov5s.pt')  # Load a pretrained model (YOLOv5s)

# Define the data and configuration for training
# You should have a 'data.yaml' file prepared that specifies the path to your dataset and class names.
data_yaml = 'path/to/data.yaml'  # Replace with the path to your data.yaml file

# Train the model
model.train(data=data_yaml, epochs=100, batch_size=16, img_size=640, device=0)

# Save the model
model.save('yolov5_stop_sign.pt')

'''
/dataset
    /images
        /train
            image1.jpg
            image2.jpg
            ...
        /val
            image1.jpg
            image2.jpg
            ...
    /labels
        /train
            image1.txt
            image2.txt
            ...
        /val
            image1.txt
            image2.txt
            ...
Each .txt file should have the corresponding labels for the images in YOLO format (class, x_center, y_center, width, height).
'''