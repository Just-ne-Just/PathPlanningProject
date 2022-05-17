import random

import matplotlib.pyplot as plt
import numpy as np
import time
from matplotlib.pyplot import figure


with open("out", "r") as f:
    kek = list(map(lambda x: list(map(int, x.strip().split())), f.readlines()))

visibility = int(kek[0][0])
kek.pop(0)

image = []
while len(kek[0]) != 0:
    image.append(kek[0])
    kek.pop(0)

kek.pop(0)

image_with_path = [[3] * len(image[0]) for i in range(len(image))]
print(image_with_path)

for x in kek:
    if len(x) == 0:
        break
    for i in range(-visibility, visibility + 1):
        for j in range(-visibility, visibility + 1):
            if 0 <= x[0] + i < len(image) and\
                    0 <= x[1] + j < len(image[0]) and\
                    image_with_path[x[0] + i][x[1] + j] != 2:
                image_with_path[x[0] + i][x[1] + j] = image[x[0] + i][x[1] + j]
    image_with_path[x[0]][x[1]] = 2
    figure(figsize=(25, 25), dpi=10)
    plt.imshow(image_with_path)
    plt.show()
    time.sleep(2)
