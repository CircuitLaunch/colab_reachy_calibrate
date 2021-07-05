#!/usr/bin/env python3

import json
import numpy as np
import rospy as ros
from colab_reachy_calibrate.srv import TransformPose, TransformPoseResponse

class ErrorMap:
    def __init__(self, map):
        self.map = map
        self.dims = map.shape

        self.xDivs = dims[0]-1
        self.yDivs = dims[1]-1
        self.zDivs = dims[2]-1

        self.lowerLeftPair = map[0,0,0]
        self.upperRightPair = map[self.xDivs, self.yDevs, self.zDevs]

        self.xMin = lowerLeftPair[0][0]
        self.yMin = lowerLeftPair[0][1]
        self.zMin = lowerLeftPair[0][2]

        self.xMax = upperRightPair[0][0]
        self.yMax = upperRightPair[0][1]
        self.zMax = upperRightPair[0][2]

        self.xRange = self.xMax - self.xMin
        self.yRange = self.yMax - self.yMin
        self.zRange = self.zMax - self.zMin

        self.xStep = self.xRange / self.xDivs
        self.yStep = self.yRange / self.yDivs
        self.zStep = self.zRange / self.zDivs

    def interpolateError(position):
        xIn = position.x
        yIn = position.y
        zIn = position.z

        xIndex = int((xIn - self.xMin) / self.xRange)
        yIndex = int((yIn - self.yMin) / self.yRange)
        zIndex = int((zIn - self.zMin) / self.zRange)

        try:
            errors = [map[xIndex + (i & 1), yIndex + ((i & 2) >> 1), zIndex + ((i & 4) >> 2)][1] for i in range(0, 8)]
        except e:
            return None

        boxMinPair = map[xIndex, yIndex, zIndex]
        boxMaxPair = map[xIndex+1, yIndex+1, zIndex+1]

        boxMinX = boxMinPair[0][0]
        boxMinY = boxMinPair[0][1]
        boxMinZ = boxMinPair[0][2]

        cx = (xIn - boxMinX) / self.xStep
        cy = (yIn - boxMinY) / self.yStep
        cz = (zIn - boxMinZ) / self.zStep

        cx = [1.0 - cx, cx]
        cy = [1.0 - cy, cy]
        cz = [1.0 - cz, cz]

        weights = [ cx[i & 1] * cy[(i & 2) >> 1] * cz[(i & 4) >> 2] for i in range(0, 8) ]

        errorx = 0.0
        errory = 0.0
        errorz = 0.0

        for i in range(0, 8):
            errorx += errors[i][0] * weights[i]
            errory += errors[i][1] * weights[i]
            errorz += errors[i][2] * weights[i]

        return errorx, errory, errorz

class Compensator:
    def __init__(self):
        ros.init_node('compensator')

        rf = open(ros.get_param('/calibrate/right_error_map_file'))
        self.rightMap = np.array(json.load(rf))
        rf.close()

        lr = open(ros.get_param('/calibrate/left_error_map_file'))
        self.leftMap = np.array(json.load(lf))
        lf.close()

        self.ServiceServer('compensate', TransformPose, self.compensate)

    def compensate(self, request):
        side = request.side
        inputPos = request.input_pose.position
        resp = TransformPoseResponse()
        errorTuple = self.interpolateError(inputPos)
        if errorTuple == None:
            resp.in_bounds = False
        else:
            resp.output_pose.position.x = inputPos.x + errorTuple[0]
            resp.output_pose.position.y = inputPos.y + errorTuple[1]
            resp.output_pose.position.z = inputPos.z + errorTuple[2]
            resp.in_bounds = True
        return resp

if __name__ == '__main__':
    Compensator()

    rospy.spin()
