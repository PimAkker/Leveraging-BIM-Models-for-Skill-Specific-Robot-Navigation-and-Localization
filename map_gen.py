import cv2
import os
import xml.etree.ElementTree as ET
import ntpath
import subprocess
import zipfile
import numpy as np
from wand.image import Image
from pathlib import Path

subprocess.run(["PrusaSlicer --sla --load config.ini atlas8_rescaled.stl"],shell=True,check=True,cwd= os.getcwd())

p_sl = Path('atlas8_rescaled.sl1')
p_sl.rename(p_sl.with_suffix('.zip'))

if not os.path.exists("atlas8_rescaled"):
    os.makedirs("atlas8_rescaled")

with zipfile.ZipFile("atlas8_rescaled.zip","r") as zip_ref:
    zip_ref.extractall("atlas8_rescaled")

### Iterate through images to match desired Height
# https://www.geeksforgeeks.org/how-to-iterate-through-images-in-a-folder-python/
dir = 'atlas8_rescaled'
img_path = Path(dir).glob('*.png')

c = 0
c_ar = []
imgs = []
img_paths = []
for img_ps in img_path:
    c += 1
    c_ar.append(c)
    img_se = ntpath.basename(img_ps)
    imgs.append(img_se)
    img_paths.append(img_ps)
imgs.sort()
img_paths.sort()

### (Find Desired height) ---> might be placed after image iteration 
# read config.ini and model.sdf

# full-scale height in proportion to Prusa
h = 8.15 #total height of model [m]
h_f = 0.15 #height of the floor inside model [m]
fs_i = h/c
c_ar = np.asarray(c_ar)
fs_ar = fs_i*c_ar
# fs_ar = [[x*fs_ar for x in y] for y in c_ar]

print(c_ar[-1])
print(fs_ar[0])

tree = ET.parse('model.sdf') # ---> location of tree # can provide path to catkin ws alternatively
for node in tree.iter():
     if node.tag == 'sensor':
        for child in node:
            if child.tag == 'pose':
                pose = child.text
                pose = list(map(float, pose.split()))
                h_r = pose[2]

h_r = h_r + h_f
print(type(h_r))
print(h_r)

d_ar = np.absolute(fs_ar-h_r)
i_i = d_ar.argmin()
print(i_i)
i = i_i
### Invert Image
fldr_nam = 'atlas8_rescaled'
num_dgt = len(str(i))
if num_dgt == 1:
    prfx = fldr_nam + '0000'
elif num_dgt == 2:
    prfx = fldr_nam + '000'
elif num_dgt == 3:
    prfx = fldr_nam + '00'
elif num_dgt == 4:
    prfx = fldr_nam + '0'
else:
    prfx = fldr_nam

#i = 17 # choose i
# prfx = 'atlas8_rescaled000'
img = cv2.imread(os.path.join(img_paths[i]))
img_neg = cv2.bitwise_not(img)
# cv2.imshow('negative',img_neg)
img_inv = prfx +str(i)+'_inverted.png'
cv2.imwrite(img_inv, img_neg)
cv2.waitKey(0)

### Convert file into binary
imgi = cv2.imread(img_inv)
ret, bw_img = cv2.threshold(imgi,220, 255, cv2.THRESH_BINARY)

bw = cv2.threshold(imgi,240,255, cv2.THRESH_BINARY)

# cv2.imshow("Binary", bw_img)
img_inv_bi = prfx +str(i)+'_inverted_binary.png'
cv2.imwrite(img_inv_bi, bw_img)
cv2.waitKey(0)
cv2.destroyAllWindows()

### (Find Pixel-to-Meter ratio/ Define Origin in Map) 
# https://stackoverflow.com/questions/56441769/iterate-over-all-pixels-to-check-which-pixels-are-white-and-which-are-black
img_ar = cv2.imread(img_inv_bi)
pix_ar = np.asarray(img_ar) # -> white [255 255 255] -> black [0 0 0]

ht = pix_ar.shape[0]
wh = pix_ar.shape[1]

m = round(wh/2)
k = 0
ext = 0
while ext == 0:
    if all(pix_ar[k][m] == [0,0,0]):
        if all(pix_ar[k+1][m] == [255,255,255]):
            ext = 1
        else:
            m -= 1 
    else:
        k += 1
hd = k
print("hd = "+str(hd))

k = 0
pc = 0
for it in range(wh):
    if all(pix_ar[hd][it] == [0,0,0]):
        pc += 1
        k +=1
    else:
        k+=1
print("pc = "+str(pc))

print(m)

print(pix_ar[0][0])
print(pix_ar.shape[0])
print(pix_ar.shape[1])
### Write .yaml file
wh_bldg = 19.54 # width of the builing
res = wh_bldg/pc
res = round(res,4)
print("res = "+ str(res))

x_orig = -53.000000
y_orig = -175.000000
z_orig = 0.000000

mapName = "map"
mapLocation = Path.home()
# completeFileNameYaml = os.path.join(mapLocation, mapName +".yaml")
completeFileNameYaml = os.path.join(os.getcwd(),mapName+".yaml")
yaml = open(completeFileNameYaml, "w")

yaml.write("image: " + str(mapLocation) + "/" + mapName + ".pgm\n")
# yaml.write("resolution: 0.0726\n")
yaml.write("resolution: "+str(res)+"\n")
yaml.write("origin: [" + str(x_orig) + "," + str(y_orig) + "," + str(z_orig) + "]\n")
yaml.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196")
yaml.close()

### Flop Image
with Image(filename= prfx + str(i) +'_inverted_binary.png') as image:
    with image.clone() as flopped:
        flopped.flop()
        flopped.save(filename='map.pgm')

### Cleans Up Directory
for png in os.listdir(os.getcwd()):
    if png.endswith(".png"):
        os.remove(os.path.join(os.getcwd(),png))
exit()

