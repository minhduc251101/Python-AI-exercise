"""
from rembg import remove
import cv2

input = cv2.imread('input.png')
output = remove(input)
cv2.imwrite('output.png', output)
"""
from loguru import logger
from rembg import remove
import cv2


def background_removal(input_image_path: str, output_image_path: str) -> bool:

    assert input_image_path.endswith(".png"), "Input image must be a png file"
    image = cv2.imread(input_image_path)
    output = remove(image)
    cv2.imwrite(output_image_path, output)
    return True

