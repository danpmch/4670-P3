///////////////////////////////////////////////////////////////////////////
//
// NAME
//  BlendImages.cpp -- blend together a set of overlapping images
//
// DESCRIPTION
//  This routine takes a collection of images aligned more or less horizontally
//  and stitches together a mosaic.
//
//  The images can be blended together any way you like, but I would recommend
//  using a soft halfway blend of the kind Steve presented in the first lecture.
//
//  Once you have blended the images together, you should crop the resulting
//  mosaic at the halfway points of the first and last image.  You should also
//  take out any accumulated vertical drift using an affine warp.
//  Lucas-Kanade Taylor series expansion of the registration error.
//
// SEE ALSO
//  BlendImages.h       longer description of parameters
//
// Copyright ?Richard Szeliski, 2001.  See Copyright.h for more details
// (modified for CSE455 Winter 2003 and CS4670 Fall 2012)
//
///////////////////////////////////////////////////////////////////////////

#include "ImageLib/ImageLib.h"
#include "BlendImages.h"
#include <float.h>
#include <math.h>
#include <assert.h>
#include <cstdio>

#define MAX(x,y) (((x) < (y)) ? (y) : (x))
#define MIN(x,y) (((x) < (y)) ? (x) : (y))

// Return the closest integer to x, rounding up
static int iround(double x) {
    if (x < 0.0) {
		return (int) (x - 0.5);
    } else {
		return (int) (x + 0.5);
    }
}

/******************* TO DO *********************
 * AccumulateBlend:
 *	INPUT:
 *		img: a new image to be added to acc
 *		acc: portion of the accumulated image where img is to be added
 *      M: transformation matrix for computing a bounding box
 *		blendWidth: width of the blending function (horizontal hat function;
 *	    try other blending functions for extra credit)
 *	OUTPUT:
 *		add a weighted copy of img to the subimage specified in acc
 *		the first 3 band of acc records the weighted sum of pixel colors
 *		the fourth band of acc records the sum of weight
 */
static void AccumulateBlend(CByteImage& img, CFloatImage& acc, CTransform3x3 M, float blendWidth)
{
	// BEGIN TODO
	// Fill in this routine

  // convert CByteImage to alpha premultiplied CFloatImage
	CFloatImage img2( img.Shape() );
	for( int row = 0; row < img.Shape().width; row++ )
	{
		for( int col = 0; col < img.Shape().height; col++ )
		{
      // extract and store alpha value in img2
      float alpha = img.Pixel( row, col, 3 ) / 255.0;
      img2.Pixel( row, col, 3 ) = alpha;

      // store alpha premultiplied rgb values in img2
			for( int channel = 0; channel < img.Shape().nBands - 1; channel++ )
			{
        printf( "Input image channel %d: %d\n", channel, img.Pixel( row, col, channel ) );
				img2.Pixel( row, col, channel ) = img.Pixel( row, col, channel ) * alpha / 255.0;
			}
		}
	}

  // store the warped image in a temporary image
  CFloatImage tmp( acc.Shape() );
	CTransform3x3 M_inv = M.Inverse();
	WarpGlobal( img2, tmp, M_inv, eWarpInterpLinear, 1.0f);

  // add result into acc
  for( int row = 0; row < acc.Shape().width; row++ )
  {
    for( int col = 0; col < acc.Shape().height; col++ )
    {
      for( int channel = 0; channel < acc.Shape().nBands; channel++ )
      {
        acc.Pixel( row, col, channel ) += tmp.Pixel( row, col, channel );
      }
    }
  }

	// END TODO
}


/******************* TO DO *********************
 * NormalizeBlend:
 *	INPUT:
 *		acc: input image whose alpha channel (4th channel) contains
 *		     normalizing weight values
 *		img: where output image will be stored
 *	OUTPUT:
 *		normalize r,g,b values (first 3 channels) of acc and store it into img
 */
static void NormalizeBlend(CFloatImage& acc, CByteImage& img)
{
  // BEGIN TODO
  // fill in this routine..

  int width = acc.Shape().width;
  int height = acc.Shape().height;
  int acc_bands = acc.Shape().nBands;
  int img_bands = img.Shape().nBands;

  printf( "acc bands: %d,  img bands: %d\n", acc_bands, img_bands );

  // for each pixel in the input image
  for( int row = 0; row < width; row++ )
  {
    for( int col = 0; col < height; col++ )
    {
      // get alpha value of pixel
      float alpha = acc.Pixel( row, col, 3 );

      // normalize rgb channels of pixel
      for( int channel = 0; channel < 3; channel++ )
      {
        float val = acc.Pixel( row, col, channel );
        img.Pixel( row, col, channel ) = ( alpha != 0 ) ? val / alpha * 255.0 : 0.0;

        printf( "channel %d: %f --> %d\n", channel, val, img.Pixel( row, col, channel ) );
      }

      // set pixel to be opaque
      img.Pixel( row, col, 3 ) = 255;
    }
  }

  // END TODO
}



/******************* TO DO *********************
 * BlendImages:
 *	INPUT:
 *		ipv: list of input images and their relative positions in the mosaic
 *		blendWidth: width of the blending function
 *	OUTPUT:
 *		create & return final mosaic by blending all images
 *		and correcting for any vertical drift
 */
CByteImage BlendImages(CImagePositionV& ipv, float blendWidth)
{
    // Assume all the images are of the same shape (for now)
    CByteImage& img0 = ipv[0].img;
    CShape sh        = img0.Shape();
    int width        = sh.width;
    int height       = sh.height;
    int nBands       = sh.nBands;

    int n = ipv.size();
	if (n == 0) return CByteImage(0,0,1);

	bool is360 = false;

	if (ipv[0].imgName == ipv[n-1].imgName)
		is360 = true;

	// Compute the bounding box for the mosaic
    float min_x = FLT_MAX, min_y = FLT_MAX;
    float max_x = 0, max_y = 0;
    int i;
    for (i = 0; i < n; i++)
    {
        CTransform3x3 &T = ipv[i].position;

        // BEGIN TODO
		// add some code here to update min_x, ..., max_y

        // iterate over the set of image corners (0,0), (width - 1, 0), (0, height - 1), (width - 1, height - 1)
        for( int row = 0; row < width; row += width - 1 )
        {
          for( int col = 0; col < height; col += height - 1 )
          {
            // project corner into panorama space
            CVector3 corner = T * CVector3( row, col, 1 );

            // update x coords
            if( corner[ 0 ] < min_x )
              min_x = corner[ 0 ];
            else if( corner[ 0 ] > max_x )
              max_x = corner[ 0 ];

            // update y coords
            if( corner[ 1 ] < min_y )
              min_y = corner[ 1 ];
            else if( corner[ 1 ] > max_y )
              max_y = corner[ 1 ];

          }
        }

		// END TODO
    }

    // Create a floating point accumulation image
    CShape mShape((int)(ceil(max_x) - floor(min_x)),
                  (int)(ceil(max_y) - floor(min_y)), nBands + 1);
    CFloatImage accumulator(mShape);
    accumulator.ClearPixels();

    printf( "x range: (%f, %f),  y range: (%f, %f),  dimensions( %d, %d )\n", min_x, max_x, min_y, max_y, mShape.width, mShape.height );

    double x_init, x_final;
    double y_init, y_final;

    // Add in all of the images
    for (i = 0; i < n; i++) {
        // Compute the sub-image involved
        CTransform3x3 &M = ipv[i].position;
        CTransform3x3 M_t = CTransform3x3::Translation(-min_x, -min_y) * M;
        CByteImage& img = ipv[i].img;

        // Perform the accumulation
        AccumulateBlend(img, accumulator, M_t, blendWidth);

        if (i == 0) {
            CVector3 p;
            p[0] = 0.5 * width;
            p[1] = 0.0;
            p[2] = 1.0;

            p = M_t * p;
            x_init = p[0];
            y_init = p[1];
        } else if (i == n - 1) {
            CVector3 p;
            p[0] = 0.5 * width;
            p[1] = 0.0;
            p[2] = 1.0;

            p = M_t * p;
            x_final = p[0];
            y_final = p[1];
        }
    }

    // Normalize the results
    mShape = CShape((int)(ceil(max_x) - floor(min_x)),
		    (int)(ceil(max_y) - floor(min_y)), nBands);

    CByteImage compImage(mShape);
    NormalizeBlend(accumulator, compImage);
    bool debug_comp = false;
    if (debug_comp)
        WriteFile(compImage, "tmp_comp.tga");

    // Allocate the final image shape
	int outputWidth;

	bool crop = false;  // set to true to crop
	if (crop) {
		outputWidth = mShape.width - width;
	} else {
		outputWidth = mShape.width;
	}
	
	CShape cShape(outputWidth, mShape.height, nBands);

    CByteImage croppedImage(cShape);

    // Compute the affine translations
    CTransform3x3 A;

	// BEGIN TODO
    // fill in appropriate entries in A to trim the left edge and
    // to take out the vertical drift if this is a 360 panorama
	// (i.e. is360 is true)

	// END TODO

    // Warp and crop the composite
    WarpGlobal(compImage, croppedImage, A, eWarpInterpLinear);

    return croppedImage;
}
