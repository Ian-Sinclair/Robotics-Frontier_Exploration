#!/usr/bin/env python

import numpy as np
 
'''
    Helper file to frontiers_finder.py
'''

'''
    Here lies the majority of the logic to find, manipulate, and segment frontiers.
        This includes methods for dilation, erosion, edge detection, breadth first search,
        and connected component analysis.
        Along with conversion methods to convert from occupancy grid format to coordinates. 
'''




#  points class serves as wrapper to use instead of Pose when creating markers.
class point() :
    def __init__(self, x, y, z) :
        self.x = x
        self.y = y
        self.z = z




def dilate( grid , mask_size : tuple , target = 100 ) :
    """dilate function takes a grid, mask_size and target, 
        iterates a mask through the grid convolving the grid
        values with the mask values (unit mask).
        If at grid index (a,b) the mask contains the target,
        grid index (a,b) is reassigned to equal the target.
        This expands the array in which target pixel occupy
        on the image.

    Args:
        grid (_numpy_array_): A 2D numpy  array.
        mask_size (tuple): mask_size = (n,m) creates an n times m rectangular grid to use as a mask.

    Returns:
        num_array: A 2D numpy array with expanded target regions. 
    """    
    Dilated_image = grid.copy()

    mask = [(i,j) for i in range(-mask_size[0] , mask_size[0]+1) for j in range(-mask_size[1] , mask_size[1]+1)]

    a,b = grid.shape
    for i in range(a) :
        for j in range(b) :
            Dilated_image[i][j] = grid[i][j]
            for k in mask :
                if (0 <= (i+k[0]) < a) and (0<= (j+k[1]) < b) :
                    if grid[i+k[0]][j+k[1]] == target :
                        Dilated_image[i][j] = target
                        break
    return Dilated_image




def informed_dilate( grid , mask_size : tuple , array , target = 100 ) :
    """informed dilate expands target pixel values on a grid 
        to neighboring pixels. It is assumed that target points
        are known a priori (in array variable). And so this function
        just re-assigns the neighbors of points inside 'array' based
        on the mask_size. 

    Args:
        grid (numpy array): 2D matrix
        mask_size (_type_): tuple, mask_size = (n,m) creates an n times m mask to convolve against the grid.
        array (numpy array): 1D list of target points.
        target (int, optional): reassigns neighboring points to target value. Defaults to 100.

    Returns:
        new_grid (numpy array): 2d matrix with expanded targets.
        new_array (numpy array): updated list of target points.
    """    
    row_max, col_max = grid.shape
    new_grid = grid.copy()
    mask = [(i,j) for i in range(-mask_size[0] , mask_size[0]+1) for j in range(-mask_size[1] , mask_size[1]+1)]
    new_array = []
    

    for a,b in array :
        for i,j in mask :
            if (0 <= (a+i) < row_max) and (0<= (b+j) < col_max) :
                new_grid[a+i][b+j] = target
                new_array += [(a+i,b+j)]
    return new_grid, new_array




def informed_erode( Grid , array , mask_size : tuple , key = None, tr = 3, mask = None ) :
    """Takes a list of points (array) on a grid (grid), determines the entropy of
        each point based on the uncertainty of its neighbors.
        If to much information is known about the neighbors of a point, it is removed
        as a target point (frontier candidate). This is determined by a hard threshold (tf).
        Neighbors are determined by an n times m mask.

    conventions: 
        -1 : unknown space
        0 : known unoccupied space
        100 : known occupied space

    Args:
        Grid (numpy array): 2D numpy matrix
        array (numpy array): 1D list of target points
        mask_size (tuple): mask_size = (n,m) creates an n times m mask.
        key (numpy array, optional): fill if disparity between grid and targets. Defaults to None.
        tr (int, optional): entropy threshold (lower results in more aggressive pruning). Defaults to 3.
        mask (_type_, optional): fill if a special mask is needed, overwrites mask_size. Defaults to None.

    Returns:
        new_grid (numpy array): 2D matrix with re-assigned targets.
        new_points (numpy array): 1D array of all remaining target points ( frontiers )
    """    
    row_max, col_max = Grid.shape
    new_grid = Grid.copy()
    if key is None : key = Grid.copy()
    new_points = set(array)
    if mask == None :
        mask = [(i,j) for i in range(-mask_size[0] , mask_size[0]+1) for j in range(-mask_size[1] , mask_size[1]+1)]
    for a,b in array :
        conv = [key[i+a][j+b] for i,j in mask if (0<= (i+a) < row_max) and (0<= (b+j) < col_max) and key[i+a][j+b] != 100]
        if sum(conv) > -tr :
            new_grid[a][b] = 0
            try : 
                new_points.remove((a,b))
            except :
                continue
    return new_grid, list(new_points)




def naive_target_filter(Grid, target=100) :
    """takes a grid and a target,
        returns a list of the indices 
        coinciding with each occurrence of
        the target

    Args:
        Grid (numpy array): 2D matrix
        target (int, optional): targets to extract from grid. Defaults to 100.

    Returns:
        points (numpy array): list of indices coinciding with each occurrence of target in grid.
    """    
    a,b = Grid.shape
    points = [(c,r,0) for r in range(a) for c in range(b) if Grid[c][r]== target]
    return points





def tf_occuGrid_to_map( array , width = 384, height = 384, offset = 10, map_width = 19.2) :
    """Coordinate transform from 2D matrix to map coordinates.

    Args:
        array (numpy array): list of matrix indices as tuples
        width (int, optional): width of 2d numpy array. Defaults to 384.
        height (int, optional): height of 2d numpy array. Defaults to 384.
        offset (int, optional): distance between origins (expected square). Defaults to 10.
        map_width (float, optional): the total size of the map. Defaults to 19.2.

    Returns:
        numpy array: modified list of tuples in map coordinate form.
    """    
    return [point(((c/width)*map_width) - offset , ((r/height)*map_width) - offset , 0) for r,c in array]





def edge_detection(Grid, mask_size : tuple = (1,1), filtered_Grid = None) :
    """Finds edges in a 2D grid based on disparity between pixel intensity 
        values. Takes a 2D grid, convolves an n times m mask at each index.
        If both a 0 and -1 are with a mask at an index, that index is marked
        as an edge (frontier)
    
    conventions:
        100 : known occupied space
        0 : known empty space
        -1 : unknown space

    Args:
        Grid (_type_): _description_
        mask_size (tuple, optional): _description_. Defaults to (1,1).
        filtered_Grid (_type_, optional): _description_. Defaults to None.

    Returns:
        filtered_Grid: binary grid with 100 as edges and 0 as non-edges.
        edges : list of the index locations of all edge pixels.
    """    
    a,b = Grid.shape
    mask = [(i,j) for i in range(-mask_size[0] , mask_size[0]+1) for j in range(-mask_size[1] , mask_size[1]+1)]
    if filtered_Grid == None : filtered_Grid = np.zeros(Grid.shape,int)
    edges = []

    for i in range(a) :
        for j in range(b) :
            conv = [Grid[i+k[0]][j+k[1]] for k in mask if (0<= (i+k[0]) < a) and (0<= (j+k[1]) < b)]
            if -1 in conv and 0 in conv and Grid[i][j] != 100 and Grid[i][j] == -1:
                filtered_Grid[i][j] = 100
                edges += [(i,j)]
    return filtered_Grid, edges





def informed_edge_detection(Grid, array, mask_size : tuple = (1,1) , filtered_Grid = None) :
    """Edge detection based on a priori knowledge about pixels that are likely to be edges.
        determines which pixels are actually edges. (less exhaustive than regular edge detection 
        but very fast.)

    Args:
        Grid (numpy array): 2D numpy matrix
        array (numpy array): list of indices that could be edges.
        mask_size (tuple, optional): tuple: (n,m) --> creates an n times m mask. Defaults to (1,1).
        filtered_Grid (_type_, optional): fill if need to preserve static edges in the return grid. Defaults to None.

    Returns:
        filtered_Grid: binary grid with 100 as edges and 0 as non-edges.
        edges : list of the index locations of all edge pixels.
    """    
    width,height = Grid.shape
    mask = [(i,j) for i in range(-mask_size[0] , mask_size[0]) for j in range(-mask_size[1] , mask_size[1])]
    
    if len(filtered_Grid) == 0 : filtered_Grid = np.zeros(Grid.shape,int)
    edges = []

    for a,b in array :
        conv = [Grid[i+a][j+b] for i,j in mask if (0<= (i+a) < width) and (0<= (b+j) < height)]
        if -1 in conv and 0 in conv and Grid[a][b] != 100 :
            filtered_Grid[a][b] = 100
            edges += [(a,b)]
    return filtered_Grid, edges
    







def get_successors( center , grid , kernel ) :
    x,y = center
    width,height = grid.shape
    return [(x+a,y+b) for a,b in kernel if (0<= (x+a) < width) and (0<= (y+b) < height) and grid[x+a][y+b] == 100]


def BFS( anchor , grid , array , kernel_size ) :
    kernel = [(i,j) for i in range(-kernel_size[0] , kernel_size[0]+1) for j in range(-kernel_size[1] , kernel_size[1]+1)]

    segment = {anchor}

    queue = get_successors( anchor , grid, kernel )

    while len(queue) > 0 :
        anchor = queue.pop()
        segment.add(anchor)
        a,b = anchor
        grid[a][b] = 0
        successors = [x for x in get_successors( anchor , grid , kernel ) if x not in segment and x not in queue]
        queue = successors + queue

        try : 
            array.remove(anchor)
        except :
            continue

    return list(segment), array, grid


def connection_component_analysis( grid, array, kernel_size : tuple ) :
    """Segment each frontier as semi-continuous topological regions. 
                By semi-continuous we mean that the region is either fully connected or any gaps between 
                components are 'small'.
            This is done by running breadth first search (BFS) sequentially on each frontier point
            candidate and connecting the visited points.
            Successors in the BFS for each point are found by the neighbors of that point on a grid.
            Neighbors are determined by overlaying an (n X n) kernel. 

    Args:
        grid (numpy array): 2D matrix
        array (numpy array): 1D list of frontier points
        kernel_size (tuple): (n,m) --> size n times m mask

    Returns:
        frontiers (numpy array): returns a list of frontier clusters.
    """    
    '''
        1) convert array to set
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
    updated_grid = grid.copy()

    while len(novel_points) > 0 :
        anchor = novel_points.pop()
        x,y = anchor
        if updated_grid[x][y] == 100 :
            segment , novel_points , updated_grid = BFS(anchor , updated_grid , novel_points , kernel_size)
            frontiers += [segment]

    return frontiers



def get_centroid(points_array) :
    return point(sum([a.x for a in points_array])/len(points_array), sum([b.y for b in points_array])/len(points_array),1)

