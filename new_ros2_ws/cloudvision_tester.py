import os
import io
from google.cloud import vision

# Set up Google Cloud credentials
os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = '/home/rosam/Downloads/causal-relic-430206-p6-ec3ade6f63da.json'

# Initialize Google Cloud Vision client
vision_client = vision.ImageAnnotatorClient()

def detect_text(image_path):
    """Detects text in the file."""
    with open(image_path, "rb") as image_file:
        content = image_file.read()

    image = vision.Image(content=content)
    response = vision_client.text_detection(image=image)
    texts = response.text_annotations

    if response.error.message:
        raise Exception(
            "{}\nFor more info on error messages, check: "
            "https://cloud.google.com/apis/design/errors".format(response.error.message)
        )

    detected_texts = []
    for text in texts:
        vertices = [(vertex.x, vertex.y) for vertex in text.bounding_poly.vertices]
        detected_texts.append({
            'description': text.description,
            'vertices': vertices
        })

    return detected_texts

def extract_coordinates_by_color(detected_texts, color):
    """Extracts the coordinates for the specified color block."""
    for text in detected_texts:
        if color.lower() in text['description'].lower():
            description = text['description']
            # Parse the coordinates from the description
            import re
            match = re.search(r"\((\d+),\s*(\d+)\)", description)
            if match:
                pixel_x = int(match.group(1))
                pixel_y = int(match.group(2))
                return pixel_x, pixel_y
    return None, None

def main():
    image_path = '/home/rosam/Downloads/Image Window_screenshot_31.07.2024.png'
    detected_texts = detect_text(image_path)

    if not detected_texts:
        print("No text detected or an error occurred.")
        return

    color = "Red"  # Example: Change to the desired color (e.g., "Red", "Blue", "Green")
    pixel_x, pixel_y = extract_coordinates_by_color(detected_texts, color)

    if pixel_x is not None and pixel_y is not None:
        print(f"The coordinates for the {color} block are: ({pixel_x}, {pixel_y})")
    else:
        print(f"No coordinates found for the {color} block.")

if __name__ == '__main__':
    main()
