from PIL import Image
import numpy as np
import numpy

def get_rgb565_bytes_from_image():
    rgb888 = numpy.asarray(Image.open(r"C:\Users\IldaRon\Desktop\OpenMV-master\OpenMV-master\3.RGB565\test1.bmp")) #.getdata()).reshape(80, 129, 3)
    # check that image have 3 color components, each of 8 bits
    assert rgb888.shape[-1] == 3 and rgb888.dtype == numpy.uint8
    r5 = (rgb888[..., 0] >> 3 & 0x1f).astype(numpy.uint16)
    g6 = (rgb888[..., 1] >> 2 & 0x3f).astype(numpy.uint16)
    b5 = (rgb888[..., 2] >> 3 & 0x1f).astype(numpy.uint16)
    rgb565 = r5 << 11 | g6 << 5 | b5
    print(len(rgb565))
    return (rgb565)

z_image = get_rgb565_bytes_from_image ()
print ("image", len(z_image))

z_image = z_image.reshape(80, 129, 1)

image = np.zeros((80,129))

for i in range(80):
   # print (i)
    for j in range(129):
    #    print (j)
        for k in range(1):
            #if k == 2:
        # print (k)   
         image[i][j] = z_image[i][j][k]
print ("image", len(image))
im_1 = Image.fromarray(image.astype('uint8'), 'L')
im_1.show()
