import cv2
import numpy as np
import pickle

def save_model(image_path, model_path):
    """
    Extract and save ORB keypoints and descriptors from the query image.

    :param image_path: Path to the image to be processed.
    :param model_path: Path where the model will be saved.
    """
    # Load the image
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

    # Initialize the ORB detector
    orb = cv2.ORB_create()

    # Find the keypoints and descriptors with ORB
    keypoints, descriptors = orb.detectAndCompute(image, None)

    # Serialize and save keypoints and descriptors to a file
    with open(model_path, 'wb') as model_file:
        pickle.dump((keypoints, descriptors), model_file)

    print(f"Model saved to {model_path}")

def load_model(model_path):
    """
    Load ORB keypoints and descriptors from a file.

    :param model_path: Path to the saved model file.
    :return: Tuple of keypoints and descriptors.
    """
    # Load keypoints and descriptors from the model file
    with open(model_path, 'rb') as model_file:
        keypoints, descriptors = pickle.load(model_file)
    return keypoints, descriptors

def detect_object_with_model(model_path, train_image_path):
    """
    Detect an object in a scene using saved ORB keypoints and descriptors.

    :param model_path: Path to the saved model file.
    :param train_image_path: Path to the scene image where the object is to be detected.
    """
    # Load the model
    keypoints_query, descriptors_query = load_model(model_path)

    # Load the train image
    train_image = cv2.imread(train_image_path, cv2.IMREAD_GRAYSCALE)

    # Initialize the ORB detector
    orb = cv2.ORB_create()

    # Find the keypoints and descriptors with ORB in the train image
    keypoints_train, descriptors_train = orb.detectAndCompute(train_image, None)

    # Create a BFMatcher object with Hamming distance and cross-checking
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    # Match descriptors
    matches = bf.match(descriptors_query, descriptors_train)

    # Sort them in the order of their distance
    matches = sorted(matches, key=lambda x: x.distance)

    # Draw the first 10 matches
    result_image = cv2.drawMatches(
        None, keypoints_query, train_image, keypoints_train, matches[:10], None,
        flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

    # Show the result
    cv2.imshow("Matches", result_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Example usage
# Save model from the query image
save_model('query_object.jpg', 'object_model.pkl')

# Detect the object in a new scene
detect_object_with_model('object_model.pkl', 'scene.jpg')
