from PIL import Image
import sys

if len(sys.argv) < 1:
	print "The script should be passed the full path to the image location"
filename = sys.argv[1]
# full_image = Image.open('/home/pracsys/repositories/summer_18_ai/cs_440_assignment/src/prx_output/images/_0.jpg')
full_image = Image.open(filename)
size = 28,28
image = full_image.resize(size, Image.ANTIALIAS)
width, height = image.size
pixels = image.load()
print width, height
# Check if has alpha, to avoid "too many values to unpack" error
has_alpha = len(pixels[0,0]) == 4
print has_alpha
# Create empty 2D list
fill = 1
array = [[fill for x in range(width)] for y in range(height)]

for y in range(height):
    for x in range(width):
        r, g, b = pixels[x,y]
        lum = 255-((r+g+b)/3) # Reversed luminosity
        array[y][x] = float(lum/(float(255))) # Map values from range 0-255 to 0-1
for arr in array:
    string = ""
    for ar in arr:
        string = string+str(ar)
    print string

