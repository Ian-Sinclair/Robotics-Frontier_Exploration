#!/usr/bin/env python

import numpy as np


class point() :
    def __init__(self, x, y, z) :
        self.x = x
        self.y = y
        self.z = z


def dilate( array , mask_size : tuple ) :
    Dilated_image = array.copy()

    mask = [(i,j) for i in range(-mask_size[0] , mask_size[0]+1) for j in range(-mask_size[1] , mask_size[1]+1)]

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

def informed_dilate( grid , mask_size , array ) :
    row_max, col_max = grid.shape
    new_grid = grid.copy()
    mask = [(i,j) for i in range(-mask_size[0] , mask_size[0]+1) for j in range(-mask_size[1] , mask_size[1]+1)]
    new_array = []
    

    for a,b in array :
        for i,j in mask :
            if (0 <= (a+i) < row_max) and (0<= (b+j) < col_max) :
                new_grid[a+i][b+j] = 100
                new_array += [(a+i,b+j)]
    return new_grid, new_array

def informed_erode( Grid , mask_size, array, key = None, tr = 3 ) :
    row_max, col_max = Grid.shape
    new_grid = Grid.copy()
    if key is None : key = Grid.copy()
    new_points = set(array)

    mask = [(i,j) for i in range(-mask_size[0] , mask_size[0]+1) for j in range(-mask_size[1] , mask_size[1]+1)]
    for a,b in array :
        for i,j in mask :
            conv = [key[i+a][j+b] for i,j in mask if (0<= (i+a) < row_max) and (0<= (b+j) < col_max) and key[i+a][j+b] != 100]
            if sum(conv) > -tr :
                new_grid[a][b] = 0
                try : 
                    new_points.remove((i,j))
                except :
                    continue
    return new_grid, list(new_points)




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
    mask = [(i,j) for i in range(-mask_size[0] , mask_size[0]+1) for j in range(-mask_size[1] , mask_size[1]+1)]
    if filtered_Grid == None : filtered_Grid = np.zeros(Grid.shape,int)
    edges = []

    for i in range(a) :
        for j in range(b) :
            conv = [Grid[i+k[0]][j+k[1]] for k in mask if (0<= (i+k[0]) < a) and (0<= (j+k[1]) < b)]
            if -1 in conv and 0 in conv and Grid[i][j] != 100 :
                filtered_Grid[i][j] = 100
                edges += [(i,j)]
    return filtered_Grid, edges


def informed_edge_detection(Grid, array, mask_size : tuple = (1,1) , filtered_Grid = None) :
    width,height = Grid.shape
    mask = [(i,j) for i in range(-mask_size[0] , mask_size[0]) for j in range(-mask_size[1] , mask_size[1])]
    
    if len(filtered_Grid) == 0 : filtered_Grid = np.zeros(Grid.shape,int)

    for a,b in array :
        conv = [Grid[i+a][j+b] for i,j in mask if (0<= (i+a) < width) and (0<= (b+j) < height)]
        if -1 in conv and 0 in conv and Grid[a][b] != 100 :
            filtered_Grid[a][b] = 100
    return filtered_Grid
    


def get_successors( center , grid , kernel ) :
    x,y = center
    width,height = grid.shape
    return [(x+a,y+b) for a,b in kernel if (0<= (x+a) < width) and (0<= (y+b) < height) and grid[x+a][y+b] == 100] # add edge case


def BFS( anchor , grid , array , kernel_size ) :
    kernel = [(i,j) for i in range(-kernel_size[0] , kernel_size[0]+1) for j in range(-kernel_size[1] , kernel_size[1]+1)]
    kernel.remove((0,0))

    segment = {anchor}

    queue = get_successors( anchor , grid, kernel )
    limit = len(array)
    count = 0

    while len(queue) > 0 and count < limit :
        count += 1
        anchor = queue.pop()
        segment.add(anchor)

        successors = [x for x in get_successors( anchor , grid , kernel ) if x not in segment and x not in queue]

        queue += successors

        try : 
            array.remove(anchor)
        except :
            continue

    return list(segment), array


def connection_component_analysis( grid, array, kernel_size ) :
    '''
        1) convert array to set (or hash table not sure)
        2) pop frontier point from set
        2) initialize a new segment (list)
        3) run BFS from that point
            3.a) neighbors are defined by the size of the kernel
            3.b) after removing from queue add to segment and  remove from original set.
            3.c) continue until all paths are exhausted return segments and updated array set
        4) repeat until array set is empty.
    '''
    novel_points = array.copy()
    frontiers = []
    print(type(novel_points))
    count = 0
    limit = len(array)

    while len(novel_points) > 0 and count < limit+100 :
        count += 1
        anchor = novel_points.pop()
        # run BFS
        segment , novel_points = BFS(anchor , grid , novel_points , kernel_size)
        frontiers += [segment]

    return frontiers



