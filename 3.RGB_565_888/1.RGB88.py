from PIL import Image
import numpy as np
#np.set_printoptions(threshold=np.inf)
#np.set_printoptions(threshold = np.nan) 
icontr = np.array(Image.open(r"C:\Users\test1.bmp").getdata()).reshape(80, 129, 3)
image = np.zeros((80,129)) 
for i in range(80):
    for j in range(129):
        for k in range(3):
         image[i][j] = icontr[i][j][k]  
im_1 = Image.fromarray(image.astype('uint8'), 'L')
im_1.show()
