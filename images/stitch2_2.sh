#!/bin/bash

IMG1="pano1_0008.tga"
IMG2="pano1_0009.tga"

NAME1=`echo $IMG1 | cut -d '.' -f 1`
NAME2=`echo $IMG2 | cut -d '.' -f 1`

WARP_NAME1="${NAME1}_warped"
WARP_NAME2="${NAME2}_warped"

./Panorama sphrWarp $IMG1 $WARP_NAME1.tga 595 -0.15 0.0
./Panorama sphrWarp $IMG2 $WARP_NAME2.tga 595 -0.15 0.0

./Features computeFeatures $WARP_NAME1.tga $WARP_NAME1.f
./Features computeFeatures $WARP_NAME2.tga $WARP_NAME2.f

MATCH_FILE="match-$WARP_NAME1-$WARP_NAME2.txt"

./Features matchFeatures $WARP_NAME1.f $WARP_NAME2.f 0.8 $MATCH_FILE 2

./Panorama alignPair $WARP_NAME1.f $WARP_NAME2.f $MATCH_FILE 2000 1
#MATRIX="`./Panorama alignPair $WARP_NAME1.f $WARP_NAME2.f $MATCH_FILE 200 1`"
#echo $MATRIX

# create pairlist file for blendPairs
#PAIRLIST="pairlist-$WARP_NAME1-$WARP_NAME2.txt"
#echo "$WARP_NAME1.tga $WARP_NAME2.tga $MATRIX" > $PAIRLIST

#./Panorama blendPairs $PAIRLIST stitch2.tga 200
#./Panorama blendPairs pairlist2.txt stitch2.tga 200
