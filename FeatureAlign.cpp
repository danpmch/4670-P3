///////////////////////////////////////////////////////////////////////////
//
// NAME
//  FeatureAlign.h -- image registration using feature matching
//
// SEE ALSO
//  FeatureAlign.h      longer description
//
// Copyright Richard Szeliski, 2001.
// (modified for CSE576 Spring 2005, and for CS4670, Fall 2012)
//
///////////////////////////////////////////////////////////////////////////

#include "ImageLib/ImageLib.h"
#include "FeatureAlign.h"
#include <math.h>
#include <assert.h>
#include <iostream>
#include <set>
#include <time.h>

#include "Eigen/Core"
#include "Eigen/SVD"

using namespace Eigen;

/******************* TO DO *********************
 * ComputeHomography:
 *
 * Given two feature sets and a set of matches between them, compute
 * a homography using the direct linear transformation method.  This
 * function will be used in other functions you write
 *
 * INPUT: 
 *      f1, f2: source feature sets
 *      matches: correspondences between f1 and f2
 *               *!!IMPORTANT NOTE!!* Each match in 'matches' contains two feature ids of 
 *               matching features, id1 (in f1) and id2 (in f2).
 *               These ids are 1-based indices into the feature arrays,
 *               so you access the appropriate features as f1[id1-1] and f2[id2-1].
 *      m: motion model
 *
 * OUTPUT: homography (returned as a CTransform3x3 object)
 */

CTransform3x3 ComputeHomography(const FeatureSet &f1, const FeatureSet &f2,
								const vector<FeatureMatch> &matches)
{
	cout << "ComputeHomography" << endl;
	int numMatches = (int) matches.size();

	// first, we will compute the A matrix in the homogeneous linear equations Ah = 0
	int numRows = 2 * numMatches; // number of rows of A
	const int numCols = 9;        // number of cols of A

	// this allocates space for the matrix
	typedef Matrix<double, Dynamic, 9, RowMajor> MatrixType;
	MatrixType A = MatrixType::Zero(numRows, numCols);

	for (int i = 0; i < numMatches; i++) {
		const FeatureMatch &m = matches[i];
		const Feature &a = f1[m.id1 - 1];
		const Feature &b = f2[m.id2 - 1];


		// BEGIN TODO
		// fill in the matrix A in this loop.
		// To access an element of A, use parentheses, e.g. A(0,0)

    double x = a.x;
    double y = a.y;

    double xp = b.x;
    double yp = b.y;
    
    int A_row = 2 * i;
    A.row( A_row )     << x, y, 1, 0, 0, 0, -xp * x, -xp * y, -xp;
    A.row( A_row + 1 ) << 0, 0, 0, x, y, 1, -yp * x, -yp * y, -yp;

		// END TODO
	}

	// compute the svd of A using the Eigen package
	JacobiSVD<MatrixType> svd(A, ComputeFullV);

	// BEGIN TODO
	// fill the homography H with the appropriate elements of the SVD
	// To extract, for instance, the V matrix, use svd.matrixV()
  
  const double *H_vector = svd.matrixV().col( svd.matrixV().cols() - 1 ).data();
	CTransform3x3 H;
  for( int row = 0; row < 3; row++ )
    for( int col = 0; col < 3; col++ )
      H[ row ][ col ] = H_vector[ row * 3 + col ];

	// END TODO
	return H;
}

void print_transform( CTransform3x3 &t )
{
  printf( "[ " );
  for( int row = 0; row < 3; row++ )
    for( int col = 0; col < 3; col++ )
      printf( "%f, ", t[ row ][ col ] );
  printf( "] " );
}

CTransform3x3 ComputeTranslation(const FeatureSet &f1, const FeatureSet &f2,
								const vector<FeatureMatch> &matches)
{
	cout << "ComputeTranslation" << endl;
  assert( matches.size() == 1 );
	cout << "matches.size == 1" << endl;

  int fid1 = matches[ 0 ].id1;
  int fid2 = matches[ 0 ].id2;
  Feature feature1 = f1[ fid1 - 1 ];
  Feature feature2 = f2[ fid2 - 1 ];
  cout << "here" << endl;
  int tx = feature2.x - feature1.x;
  int ty = feature2.y - feature1.y;
  CTransform3x3 trans = CTransform3x3::Translation( tx, ty );

  return trans;
}

// returns a randomly selected subset of the input matches, with size determined by the MotionModel
vector< FeatureMatch > get_random_matches( const vector< FeatureMatch > &matches, int num_matches )
{
	cout << "get_random_matches" << endl;
  set< int > match_indices;

  // generate set of random indices
  cout << "num matches = "<< num_matches << endl;
  while( match_indices.size() < num_matches )
  {
	cout << "match_indices.size() = " <<  match_indices.size() << endl;
	cout << "matches.size() = " <<  matches.size() << endl;
    int r = rand() % matches.size();
	cout << "r = " << r << endl;
	match_indices.insert( r );
  }

  vector< FeatureMatch > match_subset;
  for( set< int >::iterator i = match_indices.begin(); i != match_indices.end(); i++ )
    match_subset.push_back( matches[ *i ] );

  return match_subset;
}

/******************* TO DO *********************
 * alignPair:
 *	INPUT:
 *		f1, f2: source feature sets
 *		matches: correspondences between f1 and f2
 *               *!!IMPORTANT NOTE!!* Each match in 'matches' contains two feature ids of 
 *               matching features, id1 (in f1) and id2 (in f2).
 *               These ids are 1-based indices into the feature arrays,
 *               so you access the appropriate features as f1[id1-1] and f2[id2-1].
 *		m: motion model
 *		nRANSAC: number of RANSAC iterations
 *		RANSACthresh: RANSAC distance threshold
 *		M: transformation matrix (output)
 *
 *	OUTPUT:
 *		repeat for nRANSAC iterations:
 *			choose a minimal set of feature matches
 *			estimate the transformation implied by these matches
 *			count the number of inliers
 *		for the transformation with the maximum number of inliers,
 *		compute the least squares motion estimate using the inliers,
 *		and store it in M
 */
int alignPair(const FeatureSet &f1, const FeatureSet &f2,
	      const vector<FeatureMatch> &matches, MotionModel m, 
	      int nRANSAC, double RANSACthresh, CTransform3x3& M)
{
    // BEGIN TODO
    // Write this entire method.  You need to handle two types of 
	//  motion models, pure translations (m == eTranslation) and 
	//  full homographies (m == eHomography).  However, you should
	//  only have one outer loop to perform the RANSAC code, as 
	//  the use of RANSAC is almost identical for both cases.
	//
	//  Your homography handling code should call ComputeHomography.
    //  This function should also call countInliers and, at the end,
	//  leastSquaresFit.

  int num_matches = -1;
  CTransform3x3 ( *transform_func )( const FeatureSet &, const FeatureSet &, const vector< FeatureMatch > & ) = NULL;
  
  switch( m )
  {
    case eTranslate:
		cout << "eTranslate" << endl;
      num_matches = 1;
      transform_func = ComputeTranslation;
      break;
    case eHomography:
		cout << "ehomography" << endl;
      num_matches = 4;
      transform_func = ComputeHomography;
      break;
    default:
      // should never get here
      assert( false );
  }

  vector< int > max;
  srand( time( NULL ) );
  for( int n = 0; n < nRANSAC; n++ )
  {
	cout << "transforming" << endl;
    CTransform3x3 current = transform_func( f1, f2, get_random_matches( matches, num_matches ) );

    vector< int > inlier_ids;
    int num_inliers = countInliers( f1, f2, matches, m, current, RANSACthresh, inlier_ids );
//    printf( "inliers for current: %d\n", num_inliers );

    if( num_inliers > max.size() )
    {
      max = inlier_ids;
//      printf( "New Transform: " ); print_transform( current ); puts( "" );
//      printf( "Max inliers: %d\n", max.size() );
    }
  }
  printf( "Total matches: %d\n", matches.size() );
  printf( "Maximum inliers: %d\n", max.size() );

  leastSquaresFit(f1, f2, matches, m, max, M);

    // END TODO

	return 0;
}

void perspectiveDivide( CVector3 v )
{
  if( v[ 2 ] != 1.0 )
  {
    v[ 0 ] /= v[ 2 ];
    v[ 1 ] /= v[ 2 ];
    v[ 2 ] /= v[ 2 ];
  }
}

float distance2d( CVector3 v1, CVector3 v2 )
{
  perspectiveDivide( v1 );
  perspectiveDivide( v2 );

  float dx = v1[ 0 ] - v2[ 0 ];
  float dy = v1[ 1 ] - v2[ 1 ];

  return sqrt( dx * dx + dy * dy );
}

/******************* TO DO *********************
 * countInliers:
 *	INPUT:
 *		f1, f2: source feature sets
 *		matches: correspondences between f1 and f2
 *               *!!IMPORTANT NOTE!!* Each match in 'matches' contains two feature ids of 
 *               matching features, id1 (in f1) and id2 (in f2).
 *               These ids are 1-based indices into the feature arrays,
 *               so you access the appropriate features as f1[id1-1] and f2[id2-1].
 *		m: motion model
 *		M: transformation matrix
 *		RANSACthresh: RANSAC distance threshold
 *		inliers: inlier feature IDs
 *	OUTPUT:
 *		transform the features in f1 by M
 *
 *		count the number of features in f1 for which the transformed
 *		feature is within Euclidean distance RANSACthresh of its match
 *		in f2
 *
 *		store these features IDs in inliers
 *
 */
int countInliers(const FeatureSet &f1, const FeatureSet &f2,
				 const vector<FeatureMatch> &matches, MotionModel m, 
				 CTransform3x3 M, double RANSACthresh, vector<int> &inliers)
{
    inliers.clear();

	for (unsigned int i = 0; i < matches.size(); i++) {
        // BEGIN TODO
        // determine if the ith matched feature f1[id1-1], when transformed by M,
        // is within RANSACthresh of its match in f2
        //
        // if so, append i to inliers
        //
        // *NOTE* Each match contains two feature ids of matching features, id1 and id2.
        //        These ids are 1-based indices into the feature arrays,
        //        so you access the appropriate features as f1[id1-1] and f2[id2-1].
		//
    
    Feature feature1 = f1[ matches[ i ].id1 - 1 ];
    Feature feature2 = f2[ matches[ i ].id2 - 1 ];

    CVector3 v1( feature1.x, feature1.y, 1 );
    CVector3 v1_transformed = M * v1;

    CVector3 v2( feature2.x, feature2.y, 1 );

    printf( "v1: (%f, %f)\n", v1[ 0 ], v1[ 1 ] );
    printf( "M: " ); print_transform( M ); puts("");
    printf( "v1_trans: (%f, %f)\n", v1_transformed[ 0 ], v1_transformed[ 1 ] );
    printf( "v2: (%f, %f)\n", v2[ 0 ], v2[ 1 ] );
    printf( "Distance: %f\n", distance2d( v1_transformed, v2 ) );
    if( distance2d( v1_transformed, v2 ) <= RANSACthresh )
    {
      inliers.push_back( i );
    }

		// END TODO
  }

    return (int) inliers.size();
}

/******************* TO DO *********************
 * leastSquaresFit:
 *	INPUT:
 *		f1, f2: source feature sets
 *		matches: correspondences between f1 and f2
 *		m: motion model
 *      inliers: inlier match indices (indexes into 'matches' array)
 *		M: transformation matrix (output)
 *	OUTPUT:
 *		compute the transformation from f1 to f2 using only the inliers
 *		and return it in M
 */
int leastSquaresFit(const FeatureSet &f1, const FeatureSet &f2,
		    const vector<FeatureMatch> &matches, MotionModel m, 
		    const vector<int> &inliers, CTransform3x3& M)
{
	// This function needs to handle two possible motion models, 
	// pure translations and full homographies.

    switch (m) {
	    case eTranslate: {
        // for spherically warped images, the transformation is a 
        // translation and only has two degrees of freedom
        //
        // therefore, we simply compute the average translation vector
        // between the feature in f1 and its match in f2 for all inliers
        double u = 0;
        double v = 0;

        for (int i=0; i < (int) inliers.size(); i++) {
          // BEGIN TODO
          // use this loop to compute the average translation vector
          // over all inliers
          
          Feature feature1 = f1[ matches[ i ].id1 - 1 ];
          Feature feature2 = f2[ matches[ i ].id2 - 1 ];

          u += feature2.x - feature1.x;
          v += feature2.y - feature1.y;

          // END TODO
        }

        u /= inliers.size();
        v /= inliers.size();

        M = CTransform3x3::Translation((float) u, (float) v);

        break;
      } 

      case eHomography: {
        // BEGIN TODO
        // Compute a homography M using all inliers.
        // This should call ComputeHomography.
        
        vector< FeatureMatch > match_subset;
        for( int i = 0; i < inliers.size(); i++ )
        {
          match_subset.push_back( matches[ inliers[ i ] ] );
        }

        M = ComputeHomography( f1, f2, match_subset );
        // END TODO

        break;
      }
    }

    return 0;
}
