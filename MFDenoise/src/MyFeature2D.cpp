//
// Created by 林奇 on 16/7/31.
//

#include "../include/MyFeature2D.h"



MyFeature2D::~MyFeature2D() {}

/*
 * Detect keypoints in an image.
 * image        The image.
 * keypoints    The detected keypoints.
 * mask         Mask specifying where to look for keypoints (optional). Must be a char
 *              matrix with non-zero values in the region of interest.
 */
void MyFeature2D::detect(InputArray image,
                         std::vector<KeyPoint>& keypoints,
                         InputArray mask )
{
    if( image.empty() )
    {
        keypoints.clear();
        return;
    }
    detectAndCompute(image, mask, keypoints, noArray(), false);
}


void MyFeature2D::detect(InputArrayOfArrays _images,
                         std::vector<std::vector<KeyPoint> >& keypoints,
                         InputArrayOfArrays _masks )
{
    vector<Mat> images, masks;

    _images.getMatVector(images);
    size_t i, nimages = images.size();

    if( !_masks.empty() )
    {
        _masks.getMatVector(masks);
        CV_Assert(masks.size() == nimages);
    }

    keypoints.resize(nimages);

    for( i = 0; i < nimages; i++ )
    {
        detect(images[i], keypoints[i], masks.empty() ? Mat() : masks[i] );
    }
}

/*
 * Compute the descriptors for a set of keypoints in an image.
 * image        The image.
 * keypoints    The input keypoints. Keypoints for which a descriptor cannot be computed are removed.
 * descriptors  Copmputed descriptors. Row i is the descriptor for keypoint i.
 */
void MyFeature2D::compute(InputArray image,
                          std::vector<KeyPoint>& keypoints,
                          OutputArray descriptors ,float _iso)
{
    if( image.empty() )
    {
        descriptors.release();
        return;
    }
    detectAndCompute(image, noArray(), keypoints, descriptors, true, _iso);
}

void MyFeature2D::compute(InputArrayOfArrays _images,
                          std::vector<std::vector<KeyPoint> >& keypoints,
                          OutputArrayOfArrays _descriptors )
{
    if( !_descriptors.needed() )
        return;

    vector<Mat> images;

    _images.getMatVector(images);
    size_t i, nimages = images.size();

    CV_Assert( keypoints.size() == nimages );
    CV_Assert( _descriptors.kind() == _InputArray::STD_VECTOR_MAT );

    vector<Mat>& descriptors = *(vector<Mat>*)_descriptors.getObj();
    descriptors.resize(nimages);

    for( i = 0; i < nimages; i++ )
    {
        compute(images[i], keypoints[i], descriptors[i]);
    }
}


/* Detects keypoints and computes the descriptors */
void MyFeature2D::detectAndCompute(InputArray, InputArray,
                                   std::vector<KeyPoint>&,
                                   OutputArray,
                                   bool ,float)
{
    CV_Error(Error::StsNotImplemented, "");
}

int MyFeature2D::descriptorSize() const
{
    return 0;
}

int MyFeature2D::descriptorType() const
{
    return CV_32F;
}

int MyFeature2D::defaultNorm() const
{
    int tp = descriptorType();
    return tp == CV_8U ? NORM_HAMMING : NORM_L2;
}

// Return true if detector object is empty
bool MyFeature2D::empty() const
{
    return true;
}
