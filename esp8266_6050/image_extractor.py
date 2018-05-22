from PIL import Image
import numpy as np


with Image.open('./Noisebridge_Reboot_logo.png') as im:
    smaller_image = im.resize((75, 75), resample=Image.NEAREST)
    smaller_image.colors = smaller_image.getcolors(256)
    data = np.array(smaller_image.convert('RGBA'))

    red, green, blue, alpha = data.T
    light_areas = (red > 250) & (blue > 250) & (green > 250) | (alpha < .2)
    data[..., :-1][light_areas.T] = (0, 0, 0)

    color_corrected = Image.fromarray(data)
    color_corrected.show()
