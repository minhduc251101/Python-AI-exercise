

from core.remove import background_removal
from loguru import logger


if __name__ == "__main__":
    logger.info("Background removal started")
    input_image_path = "test3.png"
    output_image_path = "output.png"
    background_removal(input_image_path, output_image_path)
    logger.info(output_image_path)
