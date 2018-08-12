import numpy as np
import sys
import random
from PIL import Image
import tf_evaluate_model_code as classify









#############################################################################
#############################################################################
#############################################################################
#############################################################################
def predict(input_x):
	out = ""
	if len(input_x) != (28*28):
		print "The input image or input array is shaped incorrectly. Expecting a 28x28 image."
	for i in xrange(0,28):
		out = out+"\n"
		for j in xrange(0,28):
			if input_x[(i*28)+j]>0.5:
				out = out + "1"
			else:
				out = out + "0"
	print "Input image array: \n", out
	#Fill in this function to correctly predict the class the image array corresponds to
	#Fill in this function to correctly predict the class the image array corresponds to
	#Fill in this function to correctly predict the class the image array corresponds to
	#Fill in this function to correctly predict the class the image array corresponds to
	#print "*!*!*!*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
	#print filename
	#print "*!*!*!*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"

	evaluator = classify.evaluate_model()
	prediction = evaluator.evaluate_model(8, input_x)
	# prediction = int(random.random()*9.9) #Current prediction is random
	return prediction
#############################################################################
#############################################################################
#############################################################################
#############################################################################















if len(sys.argv) < 1:
	print "The script should be passed the full path to the image location"
filename = sys.argv[1]
# full_image = Image.open('$PRACSYS_PATH/prx_output/images/_0.jpg')
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
print image_array
print im_array
out = predict(im_array)

outfile = "/".join(filename.split("/")[:-1])+"/predict.ion"
outf = open(outfile, 'w')
outf.write(str(out))