import numpy as np
import sys
import random
import tf_classify_sensed_image as emc
from PIL import Image









#############################################################################
#############################################################################
#############################################################################
#############################################################################
def predict(input_x):
	x = ""
	array = []
	array.append([])
	if len(input_x) != (28*28):
		print "The input image or input array is shaped incorrectly. Expecting a 28x28 image."
	for i in xrange(0,28):
		x = x+"\n"
		for j in xrange(0,28):
			if input_x[(i*28)+j]>0.5:
				x = x + "1"
				array[0].append(1)
			else:
				x = x + "0"
				array[0].append(0)
	print "Input image array: \n", x
	#Fill in this function to correctly predict the class the image array corresponds to
	#Fill in this function to correctly predict the class the image array corresponds to
	#Fill in this function to correctly predict the class the image array corresponds to
	#Fill in this function to correctly predict the class the image array corresponds to
	prediction = emc.evaluate_model()
	pre_result = prediction.evaluate_model(1,array)
	return pre_result
#############################################################################
#############################################################################
#############################################################################
#############################################################################















if len(sys.argv) < 1:
	print "The script should be passed the full path to the image location"
filename = sys.argv[1]
#filename = '_0.jpg'
#full_image = Image.open('/home/irfan/prx_core_ws/src/prx_output/images/_0.jpg')
full_image = Image.open(filename)
size = 28,28
image = full_image.resize(size, Image.ANTIALIAS)
width, height = image.size
pixels = image.load()
print width, height
fill = 1
array = [[fill for x in range(width)] for y in range(height)]

for y in range(height):
    for x in range(width):
        r, g, b = pixels[x,y]
        lum = 255-((r+g+b)/3)
	array[y][x] = float(lum/255)

image_array = []
for arr in array:
    for ar in arr:
    	image_array.append(ar)
im_array = np.array(image_array)
im_array.reshape(-1,1)
print image_array
print im_array
out = predict(im_array)
out = str(out)
outlis = list(out)
outlis.remove('[')
outlis.remove(']')
out = ''.join(outlis)
print(out)

outfile = "/".join(filename.split("/")[:-1])+"/predict.ion"
outfile = "/home/ziad/prx_core_ws/src/prx_output/images/predict.ion"
outf = open(outfile, 'w')
outf.write(str(out)) 
