#!/usr/bin/env python

class point() :
    def __init__(self, x, y, z) :
        self.x = x
        self.y = y
        self.z = z


def dilate( array , mask_size : tuple ) :
    Dilated_image = array.copy()

    mask = [(i,j) for i in range(-mask_size[0] , mask_size[0]) for j in range(-mask_size[1] , mask_size[1])]

    a,b = array.shape
    for i in range(a) :
        for j in range(b) :
            Dilated_image[i][j] = array[i][j]
            for k in mask :
                if (0 <= (i+k[0]) < a) and (0<= (j+k[1]) < b) :
                    if array[i+k[0]][j+k[1]] == 100 :
                        Dilated_image[i][j] = 100
                        break
    return Dilated_image

def dilate_subset( array , mask_size , subarray ) :
    row_max, col_max = array.shape
    new_array = array.copy()
    mask = [(i,j) for i in range(-mask_size[0] , mask_size[0]) for j in range(-mask_size[1] , mask_size[1])]

    for a,b in subarray :
        for i,j in mask :
            if (0 <= (a+i) < row_max) and (0<= (b+j) < col_max) :
                new_array[a+i][b+j] = 100
    return new_array