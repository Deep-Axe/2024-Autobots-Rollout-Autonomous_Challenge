import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv2D, MaxPooling2D, Flatten, Dense, Dropout

import numpy as np

# Example dataset dimensions
num_samples = 240
image_height = 240
image_width = 320
channels = 3

# Generate dummy data for illustration
#X_train = np.random.rand(num_samples, image_height, image_width, channels)
#y_train = np.random.randint(0, 2, size=num_samples)  # Random labels (0 or 1)

# Save x and y from camera -> if yellow colour seen label as 1 else 0 - should work


# Define the input shape
input_shape = (image_height, image_width, channels)

# Create a Sequential model
model = Sequential()

# First convolutional layer with ReLU activation
model.add(Conv2D(32, kernel_size=(3, 3), activation='relu', input_shape=input_shape))
# Max pooling layer
model.add(MaxPooling2D(pool_size=(2, 2)))

# Second convolutional layer with ReLU activation
model.add(Conv2D(64, kernel_size=(3, 3), activation='relu'))
# Max pooling layer
model.add(MaxPooling2D(pool_size=(2, 2)))

# Flatten the 2D feature maps to 1D feature vectors
model.add(Flatten())

# Fully connected layer with 128 neurons and ReLU activation
model.add(Dense(128, activation='relu'))

# Dropout layer to reduce overfitting
model.add(Dropout(0.5))

# Output layer with sigmoid activation (binary classification: ramp or not ramp)
model.add(Dense(1, activation='sigmoid'))

# Compile the model
model.compile(optimizer='adam', loss='binary_crossentropy', metrics=['accuracy'])

# Print model summary
model.summary()


# Train the model
model.fit(X_train, y_train, batch_size=32, epochs=10, validation_split=0.2)

# Evaluate on test data (assuming you have separate test data)
# Replace X_test and y_test with your actual test data
#loss, accuracy = model.evaluate(X_test, y_test)
#print(f'Test accuracy: {accuracy}')

# Make predictions
#predictions = model.predict(X_test)
