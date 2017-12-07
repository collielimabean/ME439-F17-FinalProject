#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import scipy as sp
from matplotlib import pyplot as plt


#svg_file = 'C:\Users\pgadamczyk\Documents\Personal\UWisc Employment\Teaching\ME439\Hardware\Robot Arm\Letters\PeterStraightLines.svg'
svg_file = 'SVGtest.svg'
# how much room do you want the robot to move in? Measure its space!
x_space = 2.0
y_space = 3.0

def parse_svg_for_paths(svg_file):
    # Find lines with 'd="' at the beginning:
    d_lines = []
    with open(svg_file,'r') as svgfile:
        for line in svgfile:
            l = line.strip()
            if l[0:3]=='d="' :
                d_lines.append(line.strip().strip('d="'))

    # parse such lines to separate out anything delimited by space
    coords = np.ndarray((0,2)).astype(float)
    current_point = np.array([0.,0.])
    #first_absolute = 1
    for d_line in d_lines:
        chunks = d_line.split(' ')  # Split the string at spaces

        # Step through the chunks and parse out coordinates
        ii = 0
        while ii < len(chunks) :
            chunk = chunks[ii]

            if chunk.isalpha():  # If chunk is alphabetic, it's a setting telling us what's coming.

                # Case of mode command determines absolute (upper) or relative (lower)
                if chunk.isupper():
                    absolute = 1
                else :
                    absolute = 0

                # use the lowercase version to settle what mode we are in.
                mode = chunk.lower()
                # Set the number of chunks to increment
                if mode == "m":
                    incr = 1
                    # first M or m is always absolute, but this is a separate setting from Mode.
                    first_absolute = 1
                elif mode == "l":
                    incr = 1
                elif mode == "c":  # skip the control points
                    incr = 3
                elif mode == "s":  # skip the control point
                    incr = 2

                ii += incr
                continue

            else:
                xy = chunk.split(',')
                xy = [float(s) for s in xy]
                if first_absolute:

                    coords = np.append(coords, [[xy[0],xy[1]]],axis=0)
                    # set first_absolute to zero (not anymore).
                    first_absolute = 0
                else:
                    if absolute:
                        coords = np.append(coords, [[xy[0],xy[1]]],axis=0)
                    else:
                        coords = np.append(coords, [current_point + np.array([xy[0],xy[1]])],axis=0)

                current_point = coords[-1]

                if ii < len(chunks)-1:
                    if chunks[ii+1].isalpha():
                        incr = 1

                ii += incr


    xsvg = coords[:,0]
    ysvg = coords[:,1]  # Note that Y is Down in SVG!

    xmin = np.min(xsvg)
    xmax = np.max(xsvg)
    xrng = xmax - xmin
    xctr = xmin + xrng/2
    ymin = np.min(ysvg)
    ymax = np.max(ysvg)
    yrng = ymax - ymin
    yctr = ymin + yrng/2




    # Shift and scale.
    k = min([x_space/xrng, y_space/yrng])
    ycsv = -1*(ysvg-ymax)*k   # Note that Y is Down in SVG!
    xcsv = (xsvg-xmin)*k

    csvcoords = np.vstack((xcsv,ycsv)).transpose()

    return csvcoords


def save_coords_to_txt(svgfile, csvcoords):
    np.savetxt(svg_file[:-4] + '.csv', csvcoords,delimiter=',')


def convert_coords_to_path_specs(csvcoords):
## path_specs are:  [Xorigin, Yorigin, Theta_init, Rsigned, Length]

    coords = np.append([[0.,0.]],csvcoords,axis=0)  # start at 0
    coords = np.append(coords,[[0.,0.]],axis=0)  # end at 0

    path_specs = np.ndarray((0,5)).astype(float)
    for ii in range(1,len(coords)):
        xorigin = coords[ii-1][0]
        yorigin = coords[ii-1][1]
        displacement = coords[ii]-coords[ii-1]
        angle = np.arctan2(-displacement[0], displacement[1])   # Remember the angle is measured from the +y axis.
        rsigned = np.inf
        length = np.linalg.norm(displacement)

        path_specs = np.append(path_specs, [[xorigin, yorigin, angle, rsigned, length]], axis=0)

    return path_specs


def convert_svg_to_path_specs(svg_file):
    csvcoords = parse_svg_for_paths(svg_file)
    path_specs = convert_coords_to_path_specs(csvcoords)

    return path_specs


if __name__=='__main__':
    csvcoords = parse_svg_for_paths(svg_file)
    save_coords_to_txt(svgfile, csvcoords)
    path_specs = convert_coords_to_path_specs(csvcoords)

    print(path_specs)

    # Plot it if you want to:
    fig = plt.figure();
    plt.plot(csvcoords[:,0],csvcoords[:,1],'o-')
    plt.axis('equal')
    plt.xlabel('x'); plt.ylabel('y');