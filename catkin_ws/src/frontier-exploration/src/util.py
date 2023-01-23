#!/usr/bin/env python

import numpy as np


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

def informed_dilate( array , mask_size , subarray ) :
    row_max, col_max = array.shape
    new_array = array.copy()
    mask = [(i,j) for i in range(-mask_size[0] , mask_size[0]) for j in range(-mask_size[1] , mask_size[1])]

    for a,b in subarray :
        for i,j in mask :
            if (0 <= (a+i) < row_max) and (0<= (b+j) < col_max) :
                new_array[a+i][b+j] = 100
    return new_array


def naive_target_filter(Grid, target=100) :
        a,b = Grid.shape
        points = [(c,r,0) for r in range(a) for c in range(b) if Grid[c][r]== target]
        return points

def tf_occuGrid_to_map( array , width = 384, height = 384, offset = 10, map_width = 19.2) :
    '''
        FIXED MAP VALUES
    '''
    return [point(((c/width)*map_width) - offset,((r/height)*map_width) - offset,z) for r,c,z in array]

def edge_detection(Grid, mask_size : tuple = (1,1), filtered_Grid = None) :
    a,b = Grid.shape
    mask = [(i,j) for i in range(-mask_size[0] , mask_size[0]) for j in range(-mask_size[1] , mask_size[1])]
    if filtered_Grid == None : filtered_Grid = np.zeros(Grid.shape,int)

    for i in range(a) :
        for j in range(b) :
            conv = [Grid[i+k[0]][j+k[1]] for k in mask if (0<= (i+k[0]) < a) and (0<= (j+k[1]) < b)]
            if -1 in conv and 0 in conv and Grid[i][j] != 100 :
                filtered_Grid[i][j] = 100
    return filtered_Grid


def informed_edge_detection(Grid, array, mask_size : tuple = (1,1) , filtered_Grid = None) :
    width,height = Grid.shape
    mask = [(i,j) for i in range(-mask_size[0] , mask_size[0]) for j in range(-mask_size[1] , mask_size[1])]
    
    if len(filtered_Grid) == 0 : filtered_Grid = np.zeros(Grid.shape,int)

    for a,b in array :
        conv = [Grid[i+a][j+b] for i,j in mask if (0<= (i+a) < width) and (0<= (b+j) < height)]
        if -1 in conv and 0 in conv and Grid[a][b] != 100 :
            filtered_Grid[a][b] = 100
    return filtered_Grid
    

