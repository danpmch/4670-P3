#!/bin/bash

./Panorama sphrWarp pano1_0008.tga warp08.tga 595 -0.15 0.0
./Panorama sphrWarp pano1_0009.tga warp09.tga 595 -0.15 0.0

./Features computeFeatures warp08.tga warp08.f
./Features computeFeatures warp09.tga warp09.f

./Features matchFeatures warp08.f warp09.f 0.8 match-08-09.txt 2

./Panorama alignPair warp08.f warp09.f match-08-09.txt 200 1

./Panorama blendPairs pairlist2.txt stitch2.tga 200
