from PIL import Image
import numpy as np

img = Image.open('./BALL.png')
width, height = img.size
width, height = 18, 18
print("old "+str(width)+"  "+str(height))
img = img.resize((width,height))
print("new "+str(width)+"  "+str(height))
img.save('./BALL.png')
