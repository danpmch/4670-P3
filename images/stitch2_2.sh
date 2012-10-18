#!/bin/bash

IMG1="pano1_0010.tga"
IMG2="pano1_0011.tga"

NAME1=`echo $IMG1 | cut -d '.' -f 1`
NAME2=`echo $IMG2 | cut -d '.' -f 1`

WARP_NAME1=$NAME1 #"${NAME1}_warped"
WARP_NAME2=$NAME2 #"${NAME2}_warped"

#./Panorama sphrWarp $IMG1 $WARP_NAME1.tga 595 -0.15 0.0
#./Panorama sphrWarp $IMG2 $WARP_NAME2.tga 595 -0.15 0.0

#./Features computeFeatures $WARP_NAME1.tga $WARP_NAME1.f
#./Features computeFeatures $WARP_NAME2.tga $WARP_NAME2.f

MATCH_FILE="match-$NAME1-$NAME2.txt"
./Features matchSIFTFeatures $NAME1.key $NAME2.key 0.8 $MATCH_FILE 2

NRANSAC="10000"
./Panorama alignPairHomography $NAME1.key $NAME2.key $MATCH_FILE $NRANSAC 1 sift
#MATRIX="`./Panorama alignPairHomography $NAME1.key $NAME2.key $MATCH_FILE $NRANSAC 1 sift`"
#echo $MATRIX

# create pairlist file for blendPairs
#PAIRLIST="pairlist-$NAME1-$NAME2.txt"
#echo "$NAME1.tga $NAME2.tga $MATRIX" > $PAIRLIST

#./Panorama blendPairs $PAIRLIST stitch2.tga 200
#feh stitch2.tga

./Panorama blendPairs pairlist2.txt stitch2.tga 200
