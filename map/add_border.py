# Script used to add a 1 pixel border to the map image

from PIL import Image


with Image.open("map.png") as img:
    pixels = img.load()

    for i in range(img.size[0]):
        for j in range(img.size[1]):
            if i == 0 or j == 0 or j == img.size[1] - 1 or i == img.size[0] - 1:
                pixels[i, j] = 0
    img.show()
