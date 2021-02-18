from cython.parallel import parallel, prange
import numpy as np
cimport numpy as np
import cython
# DTYPE = np.int
# ctypedef np.int_t DTYPE_t


from libc.math cimport sqrt,pow
# cimport numpy as np
# import cython
# import numpy as np
# from cython.parallel import parallel, prange
# from libc.math cimport sqrt,pow

# DTYPE = np.int
# ctypedef np.int_t DTYPE_t

@cython.boundscheck(False)
def euclidean_detect(unsigned char[:, :, :] img, int b, int g, int r, int eucl_dist):
    cdef int height = img.shape[0]
    cdef int width = img.shape[1]
    cdef int pix = img.shape[2]
    cdef unsigned char[:,:,:] res = np.zeros((height,width,pix), dtype=np.uint8)
    cdef int j = 0
    cdef int i = 0
    cdef int num_threads
    cdef int blue,green,red
    cdef int root = 0

    with nogil,parallel(num_threads= 4):
        for j in prange(height):
            for i in prange(width):
                blue =  int(pow(img[j][i][0] - b,2))
                green = int(pow(img[j][i][1] - g,2))
                red = int(pow(img[j][i][2] - r,2))
                # if sqrt(blue+green+red) - eucl_dist < 0:
                root = int(sqrt(blue+green+red)/(eucl_dist+1))
                img[j][i][0] *= root
                img[j][i][1] *= root
                img[j][i][2] *= root
                res[j][i] = img[j][i] #, green* int(root/eucl_dist), red* int(root/eucl_dist)
                # if sqrt(blue+green+red) < eucl_dist:
                #     res[j][i] = img[j][i]
    return res

