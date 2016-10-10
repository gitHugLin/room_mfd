//
// Created by 林奇 on 16/7/31.
//

#ifndef MFD_MYFEATURE_H
#define MFD_MYFEATURE_H

#include "../include/MyAlgorithm.h"


class MyFeature2D : public virtual MyAlgorithm
{
public:
    virtual ~MyFeature2D();

    CV_WRAP virtual void detect( InputArray image,
                                 CV_OUT std::vector<KeyPoint>& keypoints,
                                 InputArray mask=noArray() );

    virtual void detect( InputArrayOfArrays images,
                         std::vector<std::vector<KeyPoint> >& keypoints,
                         InputArrayOfArrays masks=noArray() );

    virtual void compute( InputArray image,
                          CV_OUT CV_IN_OUT std::vector<KeyPoint>& keypoints,
                          OutputArray descriptors ,float _iso = 2);

    virtual void compute( InputArrayOfArrays images,
                          std::vector<std::vector<KeyPoint> >& keypoints,
                          OutputArrayOfArrays descriptors );

    /** Detects keypoints and computes the descriptors */
    virtual void detectAndCompute( InputArray image, InputArray mask,
                                   CV_OUT std::vector<KeyPoint>& keypoints,
                                   OutputArray descriptors,
                                   bool useProvidedKeypoints=false ,float _iso = 2);

    virtual int descriptorSize() const;
    virtual int descriptorType() const;
    virtual int defaultNorm() const;

    //! Return true if detector object is empty
    virtual bool empty() const;
};


#endif //MFD_MYFEATURE_H
