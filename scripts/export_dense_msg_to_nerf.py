#!/usr/bin/env python3

import sys
import os
import numpy as np
import cv2
from numba import jit
from pathlib import Path
import argparse

import json

from scipy.spatial.transform import Rotation as R

import msgpack
import math

@jit(nopython=True)
def clip(value, min, max):
    if value > min and value < max:
        return value
    elif value < min:
        return min
    else:
        return max

# get x,y,z coords from out image pixels coords
# i,j are pixel coords
# face is face number
# edge is edge length
@jit(nopython=True)
def outImgToXYZ(i,j,face,edge):
    a = 2.0*float(i)/edge
    b = 2.0*float(j)/edge
    if face==0: # back
        (x,y,z) = (-1.0, 1.0-a, 3.0 - b)
    elif face==1: # left
        (x,y,z) = (a-3.0, -1.0, 3.0 - b)
    elif face==2: # front
        (x,y,z) = (1.0, a - 5.0, 3.0 - b)
    elif face==3: # right
        (x,y,z) = (7.0-a, 1.0, 3.0 - b)
    elif face==4: # top
        (x,y,z) = (b-1.0, a -5.0, 1.0)
    elif face==5: # bottom
        (x,y,z) = (5.0-b, a-5.0, -1.0)
    return (x,y,z)

# convert using an inverse transformation
@jit(nopython=True)
def convertBack(inSize, outSize, inPix, outPix):

    edge = int(inSize[1]/4)   # the length of each edge in pixels
    for i in range(outSize[1]):
        face = int(i/edge) # 0 - back, 1 - left 2 - front, 3 - right
        if face==2:
            rng = range(0,edge*3)
        else:
            rng = range(edge,edge*2)

        for j in rng:
            if j<edge:
                face2 = 4 # top
            elif j>=2*edge:
                face2 = 5 # bottom
            else:
                face2 = face

            (x,y,z) = outImgToXYZ(i,j,face2,edge)
            theta = math.atan2(y,x) # range -pi to pi
            r = math.hypot(x,y)
            phi = math.atan2(z,r) # range -pi/2 to pi/2
            # source img coords
            uf = ( 2.0*edge*(theta + math.pi)/math.pi )
            vf = ( 2.0*edge * (math.pi/2 - phi)/math.pi)
            # Use bilinear interpolation between the four surrounding pixels
            ui = math.floor(uf)  # coord of pixel to bottom left
            vi = math.floor(vf)
            u2 = ui+1       # coords of pixel to top right
            v2 = vi+1
            mu = uf-ui      # fraction of way across pixel
            nu = vf-vi
            # Pixel values of four corners
            A = inPix[int(clip(vi,0,inSize[0]-1)),ui % inSize[1]]
            B = inPix[int(clip(vi,0,inSize[0]-1)),u2 % inSize[1]]
            C = inPix[int(clip(v2,0,inSize[0]-1)),ui % inSize[1]]
            D = inPix[int(clip(v2,0,inSize[0]-1)),u2 % inSize[1]]
            # interpolate
            (r,g,b) = (
              A[0]*(1-mu)*(1-nu) + B[0]*(mu)*(1-nu) + C[0]*(1-mu)*nu+D[0]*mu*nu,
              A[1]*(1-mu)*(1-nu) + B[1]*(mu)*(1-nu) + C[1]*(1-mu)*nu+D[1]*mu*nu,
              A[2]*(1-mu)*(1-nu) + B[2]*(mu)*(1-nu) + C[2]*(1-mu)*nu+D[2]*mu*nu )

            outPix[j,i] = (int(round(r)),int(round(g)),int(round(b)))



def main():
    parser = argparse.ArgumentParser()
    # Path to map.msg
    parser.add_argument('map_msg', type=str, help='Path to map.msg')
    # Path to video
    parser.add_argument('video_path', type=str, help='Path to video.mp4')
    # Path to output folder
    parser.add_argument('output_folder', type=str, help='Path to output folder')

    # Skip frames
    parser.add_argument('--frame-skip', type=int, help='Only every nth frame, same as openvslam parameter', default=1)

    # Path to mask
    parser.add_argument('--mask', type=str, help='Path to mask', default='None')

    args = parser.parse_args()

    # Read msgpack file
    with open(args.map_msg, "rb") as data_file:
        byte_data = data_file.read()

    data_loaded = msgpack.unpackb(byte_data)



    print(data_loaded.keys())


    keyframe_ids = {}
    for i, (k, v) in enumerate(data_loaded['keyframes'].items()):
        keyframe_ids[v['src_frm_id']] = k

    print(keyframe_ids)

    # Opening JSON file
    f = open('/example/dense/base.json')

    # returns JSON object as
    # a dictionary
    json_data = json.load(f)

    print(json_data)

    # Create a VideoCapture object and read from input file
    # If the input is the camera, pass 0 instead of the video file name
    cap = cv2.VideoCapture(args.video_path)

    # Check if camera opened successfully
    if cap.isOpened() == False:
        print("Error opening video stream or file")



    # Read until video is completed
    frame_nr = 0

    with_mask = args.mask != 'None'
    if with_mask:
        mask = cv2.imread(args.mask)
        mask = cv2.bitwise_not(mask)
        mask_inSize = mask.shape
        mask_inPix = mask
        mask_outPix = np.zeros((int(mask_inSize[1] * 3 / 4), mask_inSize[1], 3), np.uint8)
        mask_outSize = mask_outPix.shape

        convertBack(mask_inSize, mask_outSize, mask_inPix, mask_outPix)

    frame_infos = []
    edge = 0

    if True:
        Path(os.path.join(args.output_folder, 'images/')).mkdir(parents=True, exist_ok=True)
        while (cap.isOpened()):
            # Capture frame-by-frame
            ret, frame = cap.read()
            if ret == True:
                # if frame_nr > 100:
                #     break
                if frame_nr in keyframe_ids :
                    print("Extracting frame: " + str(frame_nr))
                    r = R.from_quat(data_loaded['keyframes'][keyframe_ids[frame_nr]]['rot_cw'])
                    t = data_loaded['keyframes'][keyframe_ids[frame_nr]]['trans_cw']
                    t = np.matrix([t])  # * 0.2
                    #t[0][0] = -t[0][0]
                    t = t.transpose()
                    #print(t)
                    r_ = r.as_matrix().T
                    #print(r_)
                    t = - r_ * t
                    #print(t)
                    t = np.asarray([[t[0, 0]], [t[1, 0]], [t[2, 0]]])
                    #print(t)

                    r = R.from_quat(data_loaded['keyframes'][keyframe_ids[frame_nr]]['rot_cw'])
                    r = r.as_euler('xyz')
                    r[1] *= -1
                    r = R.from_euler('xyz', r)


                    inSize = frame.shape
                    inPix = frame
                    outPix = np.zeros((int(inSize[1] * 3 / 4), inSize[1], 3), np.uint8)
                    outSize = outPix.shape

                    convertBack(inSize, outSize, inPix, outPix)

                    edge = int(inSize[1] / 4)

                    # top
                    r_r = r * R.from_euler('Z', 180, degrees=True)
                    r_r = r_r * R.from_euler('Y', 180, degrees=True)
                    r_r = r_r * R.from_euler('X', 90, degrees=True)
                    mat = np.append(r_r.as_matrix(), t, axis=1)
                    mat = np.append(mat, [[0, 0, 0, 1]], axis=0)

                    T = np.eye(4)
                    T[:3, :3] = R.from_euler('X', -90, degrees=True).as_matrix()
                    mat = np.matmul(T, mat)

                    image_path = os.path.join(sys.argv[3], 'images/', str(frame_nr) + "_top.png")
                    cv2.imwrite(image_path, outPix[0:edge, edge * 2:edge * 3])

                    if with_mask:
                        image_path = os.path.join(sys.argv[3], 'images/', 'dynamic_mask_' + str(frame_nr) + "_top.png")
                        cv2.imwrite(image_path, mask_outPix[0:edge, edge * 2:edge * 3])

                    frame_infos.append((frame_nr, "top", mat))

                    # bottom
                    r_r = r * R.from_euler('Z', 180, degrees=True)
                    r_r = r_r * R.from_euler('Y', 180, degrees=True)
                    r_r = r_r * R.from_euler('X', -90, degrees=True)
                    mat = np.append(r_r.as_matrix(), t, axis=1)
                    mat = np.append(mat, [[0, 0, 0, 1]], axis=0)

                    T = np.eye(4)
                    T[:3, :3] = R.from_euler('X', -90, degrees=True).as_matrix()
                    mat = np.matmul(T, mat)

                    image_path = os.path.join(sys.argv[3], 'images/', str(frame_nr) + "_bottom.png")
                    cv2.imwrite(image_path, outPix[edge * 2:, edge * 2:edge * 3])

                    if with_mask:
                        image_path = os.path.join(sys.argv[3], 'images/', 'dynamic_mask_' + str(frame_nr) + "_bottom.png")
                        cv2.imwrite(image_path, mask_outPix[edge * 2:, edge * 2:edge * 3])

                    frame_infos.append((frame_nr, "bottom", mat))

                    # back
                    r_r = r * R.from_euler('Z', 180, degrees=True)
                    mat = np.append(r_r.as_matrix(), t, axis=1)
                    mat = np.append(mat, [[0, 0, 0, 1]], axis=0)

                    T = np.eye(4)
                    T[:3, :3] = R.from_euler('X', -90, degrees=True).as_matrix()
                    mat = np.matmul(T, mat)

                    image_path = os.path.join(sys.argv[3], 'images/', str(frame_nr) + "_back.png")
                    cv2.imwrite(image_path, outPix[edge:edge * 2, edge * 0:edge * 1])

                    if with_mask:
                        image_path = os.path.join(sys.argv[3], 'images/', 'dynamic_mask_' + str(frame_nr) + "_back.png")
                        cv2.imwrite(image_path, mask_outPix[edge:edge * 2, edge * 0:edge * 1])

                    frame_infos.append((frame_nr, "back", mat))

                    # left
                    r_r = r * R.from_euler('Z', 180, degrees=True)
                    r_r = r_r * R.from_euler('Y', -90, degrees=True)
                    mat = np.append(r_r.as_matrix(), t, axis=1)
                    mat = np.append(mat, [[0, 0, 0, 1]], axis=0)

                    T = np.eye(4)
                    T[:3, :3] = R.from_euler('X', -90, degrees=True).as_matrix()
                    mat = np.matmul(T, mat)

                    image_path = os.path.join(sys.argv[3], 'images/', str(frame_nr) + "_left.png")
                    cv2.imwrite(image_path, outPix[edge:edge * 2, edge * 1:edge * 2])

                    if with_mask:
                        image_path = os.path.join(sys.argv[3], 'images/', 'dynamic_mask_' + str(frame_nr) + "_left.png")
                        cv2.imwrite(image_path, mask_outPix[edge:edge * 2, edge * 1:edge * 2])

                    frame_infos.append((frame_nr, "left", mat))

                    # front
                    r_r = r * R.from_euler('Z', 180, degrees=True)
                    r_r = r_r * R.from_euler('Y', 180, degrees=True)

                    mat = np.append(r_r.as_matrix(), t, axis=1)
                    mat = np.append(mat, [[0, 0, 0, 1]], axis=0)

                    T = np.eye(4)
                    T[:3, :3] = R.from_euler('X', -90, degrees=True).as_matrix()
                    mat = np.matmul(T, mat)

                    image_path = os.path.join(sys.argv[3], 'images/', str(frame_nr) + "_front.png")
                    cv2.imwrite(image_path, outPix[edge:edge * 2, edge * 2:edge * 3])

                    if with_mask:
                        image_path = os.path.join(sys.argv[3], 'images/', 'dynamic_mask_' + str(frame_nr) + "_front.png")
                        cv2.imwrite(image_path, mask_outPix[edge:edge * 2, edge * 2:edge * 3])

                    frame_infos.append((frame_nr, "front", mat))

                    # right
                    r_r = r * R.from_euler('Z', 180, degrees=True)
                    r_r = r_r * R.from_euler('Y', 90, degrees=True)
                    mat = np.append(r_r.as_matrix(), t, axis=1)
                    mat = np.append(mat, [[0, 0, 0, 1]], axis=0)

                    T = np.eye(4)
                    T[:3, :3] = R.from_euler('X', -90, degrees=True).as_matrix()
                    mat = np.matmul(T, mat)

                    image_path = os.path.join(sys.argv[3], 'images/', str(frame_nr) + "_right.png")
                    cv2.imwrite(image_path, outPix[edge:edge * 2, edge * 3:edge * 4])

                    if with_mask:
                        image_path = os.path.join(sys.argv[3], 'images/', 'dynamic_mask_' + str(frame_nr) + "_right.png")
                        cv2.imwrite(image_path, mask_outPix[edge:edge * 2, edge * 3:edge * 4])

                    frame_infos.append((frame_nr, "right", mat))
                frame_nr += 1
            else:
                break
            if args.frame_skip > 1:
                for i in range(0, args.frame_skip - 1):
                    ret, frame = cap.read()
                    if ret == True:
                        pass
                    else:
                        break

    else:
        for i in range(0, 3000):
            if frame_nr in keyframe_ids and frame_nr > 100:
                r = R.from_quat(data_loaded['keyframes'][keyframe_ids[frame_nr]]['rot_cw'])
                t = data_loaded['keyframes'][keyframe_ids[frame_nr]]['trans_cw']
                t = np.matrix([t])  # * 0.2
                #t[0][0] = -t[0][0]
                t = t.transpose()
                print(t)
                r_ = r.as_matrix().T
                print(r_)
                t = - r_ * t
                print(t)
                t = np.asarray([[t[0, 0] ], [t[1, 0]], [t[2, 0]]])
                print(t)
                # top
                r_r = r * R.from_euler('Z', 180, degrees=True)
                r_r = r_r * R.from_euler('Y', 180, degrees=True)
                r_r = r_r * R.from_euler('X', 90, degrees=True)
                mat = np.append(r_r.as_matrix(), t, axis=1)
                mat = np.append(mat, [[0, 0, 0, 1]], axis=0)
                print(mat)

                frame_infos.append((frame_nr, "top", mat))

                # bottom
                r_r = r * R.from_euler('Z', 180, degrees=True)
                r_r = r_r * R.from_euler('Y', 180, degrees=True)
                r_r = r_r * R.from_euler('X', -90, degrees=True)
                mat = np.append(r_r.as_matrix(), t, axis=1)
                mat = np.append(mat, [[0, 0, 0, 1]], axis=0)

                frame_infos.append((frame_nr, "bottom", mat))

                # back
                r_r = r * R.from_euler('Z', 180, degrees=True)
                mat = np.append(r_r.as_matrix(), t, axis=1)
                mat = np.append(mat, [[0, 0, 0, 1]], axis=0)

                frame_infos.append((frame_nr, "back", mat))

                # left
                r_r = r * R.from_euler('Z', 180, degrees=True)
                r_r = r_r * R.from_euler('Y', -90, degrees=True)
                mat = np.append(r_r.as_matrix(), t, axis=1)
                mat = np.append(mat, [[0, 0, 0, 1]], axis=0)

                frame_infos.append((frame_nr, "left", mat))

                # front
                r_r = r * R.from_euler('Z', 180, degrees=True)
                r_r = r_r * R.from_euler('Y', 180, degrees=True)

                mat = np.append(r_r.as_matrix(), t, axis=1)
                mat = np.append(mat, [[0, 0, 0, 1]], axis=0)

                frame_infos.append((frame_nr, "front", mat))

                # right
                r_r = r * R.from_euler('Z', 180, degrees=True)
                r_r = r_r * R.from_euler('Y', 90, degrees=True)
                mat = np.append(r_r.as_matrix(), t, axis=1)
                mat = np.append(mat, [[0, 0, 0, 1]], axis=0)

                frame_infos.append((frame_nr, "right", mat))
            frame_nr += 1

    max_x = -math.inf
    min_x = math.inf
    max_y = -math.inf
    min_y = math.inf
    max_z = -math.inf
    min_z = math.inf

    for info in frame_infos:
        if info[2][0][3] > max_x:
            max_x = info[2][0][3]
        if info[2][0][3] < min_x:
            min_x = info[2][0][3]
        if info[2][1][3] > max_y:
            max_y = info[2][1][3]
        if info[2][1][3] < min_y:
            min_y = info[2][1][3]
        if info[2][2][3] > max_z:
            max_z = info[2][2][3]
        if info[2][2][3] < min_z:
            min_z = info[2][2][3]

    x_range = max_x - min_x
    y_range = max_y - min_y
    z_range = max_z - min_z

    scale_factor = 1/max(x_range, y_range, z_range)

    for info in frame_infos:
        info[2][0][3] -= max_x - x_range/2
        info[2][1][3] -= max_y - y_range/2
        info[2][2][3] -= max_z - z_range / 2

        info[2][0][3] *= scale_factor * 2
        info[2][1][3] *= scale_factor * 2
        info[2][2][3] *= scale_factor * 2


    json_data["fl_x"] = float(edge / 2)
    json_data["fl_y"] = float(edge / 2)

    json_data["cx"] = float(edge / 2)
    json_data["cy"] = float(edge / 2)

    json_data["w"] = float(edge)
    json_data["h"] = float(edge)

    for info in frame_infos:
        json_data['frames'].append({"file_path": './images/' + str(info[0]) + "_" + info[1] + ".png", "sharpness": 200.0,
                                    "transform_matrix": info[2].tolist()})

    # When everything done, release the video capture object
    cap.release()


    # onlyfiles = [f for f in listdir(sys.argv[1]) if isfile(join(sys.argv[1], f))]
    #
    # for filename in onlyfiles:
    #
    #     if filename.split('.')[1] in ['jpg', 'png']:
    #         print('converting ' + filename)
    #         #imgIn = cv2.imread(os.path.join(sys.argv[1], filename)) #Image.open(sys.argv[1])
    #         #inSize = imgIn.shape
    #         equ = E2P.Equirectangular(os.path.join(sys.argv[1], filename))
    #         #imgOut = Image.new("RGB",(inSize[0],int(inSize[0]*3/4)),"black")
    #         #convertBack(imgIn,imgOut)
    #         #inSize = imgIn.size
    #         #outSize = imgOut.size
    #         #inPix = imgIn
    #         #outPix = np.zeros((int(inSize[1]*3/4), inSize[1], 3), np.uint8)
    #         #outSize = outPix.shape
    #         #print(inSize)
    #         #print(outSize)
    #         #convertBack(inSize, outSize, inPix, outPix)
    #         #imgOut.save(sys.argv[1].split('.')[0]+"Out2.png")
    #         #cv2.imwrite(sys.argv[1].split('.')[0]+"Out2.png", outPix)
    #         #edge = int(inSize[1]/4)
    #
    #         #
    #         # FOV unit is degree
    #         # theta is z-axis angle(right direction is positive, left direction is negative)
    #         # phi is y-axis angle(up direction positive, down direction negative)
    #         # height and width is output image dimension
    #         #
    #         # Specify parameters(FOV, theta, phi, height, width)
    #
    #         cv2.imwrite(os.path.join(sys.argv[2], filename.split('.')[0]+"top.png"), equ.GetPerspective(120, 0, 90, 720, 1080))
    #         cv2.imwrite(os.path.join(sys.argv[2], filename.split('.')[0]+"bottom.png"), equ.GetPerspective(120, 0, -90, 720, 1080))
    #         cv2.imwrite(os.path.join(sys.argv[2], filename.split('.')[0]+"back.png"), equ.GetPerspective(120, 180, 0, 720, 1080))
    #         cv2.imwrite(os.path.join(sys.argv[2], filename.split('.')[0]+"left.png"), equ.GetPerspective(120, -90, 0, 720, 1080))
    #         cv2.imwrite(os.path.join(sys.argv[2], filename.split('.')[0]+"front.png"), equ.GetPerspective(120, 0, 0, 720, 1080))
    #         cv2.imwrite(os.path.join(sys.argv[2], filename.split('.')[0]+"right.png"), equ.GetPerspective(120, 90, 0, 720, 1080))
    #         #imgOut.show()

    json_path = os.path.join(args.output_folder, "transforms.json")

    with open(json_path, 'w') as outfile:
        json.dump(json_data, outfile, indent=4)



if __name__ == "__main__":
    main()
