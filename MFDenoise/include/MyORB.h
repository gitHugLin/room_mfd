//
// Created by 林奇 on 16/7/30.
//

#ifndef MFD_MYORB_H
#define MFD_MYORB_H

#include "MyFeature2D.h"
#include "log.h"


class ORB_Base : public MyFeature2D
{
public:
enum { kBytes = 32, HARRIS_SCORE=0, FAST_SCORE=1 };

static Ptr<ORB_Base> create(int nfeatures=500, float scaleFactor=1.2f, int nlevels=8, int edgeThreshold=31,
                               int firstLevel=0, int WTA_K=2, int scoreType=ORB::HARRIS_SCORE, int patchSize=31, int fastThreshold=20);

virtual void setMaxFeatures(int maxFeatures) = 0;
virtual int getMaxFeatures() const = 0;

virtual void setScaleFactor(double scaleFactor) = 0;
virtual double getScaleFactor() const = 0;

virtual void setNLevels(int nlevels) = 0;
virtual int getNLevels() const = 0;

virtual void setEdgeThreshold(int edgeThreshold) = 0;
virtual int getEdgeThreshold() const = 0;

virtual void setFirstLevel(int firstLevel) = 0;
virtual int getFirstLevel() const = 0;

virtual void setWTA_K(int wta_k) = 0;
virtual int getWTA_K() const = 0;

virtual void setScoreType(int scoreType) = 0;
virtual int getScoreType() const = 0;

virtual void setPatchSize(int patchSize) = 0;
virtual int getPatchSize() const = 0;

virtual void setFastThreshold(int fastThreshold) = 0;
virtual int getFastThreshold() const = 0;
};


class MyORB : public ORB_Base
{
public:
    explicit MyORB(int _nfeatures, float _scaleFactor, int _nlevels, int _edgeThreshold,
                      int _firstLevel, int _WTA_K, int _scoreType, int _patchSize, int _fastThreshold) :
            nfeatures(_nfeatures), scaleFactor(_scaleFactor), nlevels(_nlevels),
            edgeThreshold(_edgeThreshold), firstLevel(_firstLevel), wta_k(_WTA_K),
            scoreType(_scoreType), patchSize(_patchSize), fastThreshold(_fastThreshold)
    {}

    void setMaxFeatures(int maxFeatures) { nfeatures = maxFeatures; }
    int getMaxFeatures() const { return nfeatures; }

    void setScaleFactor(double scaleFactor_) { scaleFactor = scaleFactor_; }
    double getScaleFactor() const { return scaleFactor; }

    void setNLevels(int nlevels_) { nlevels = nlevels_; }
    int getNLevels() const { return nlevels; }

    void setEdgeThreshold(int edgeThreshold_) { edgeThreshold = edgeThreshold_; }
    int getEdgeThreshold() const { return edgeThreshold; }

    void setFirstLevel(int firstLevel_) { firstLevel = firstLevel_; }
    int getFirstLevel() const { return firstLevel; }

    void setWTA_K(int wta_k_) { wta_k = wta_k_; }
    int getWTA_K() const { return wta_k; }

    void setScoreType(int scoreType_) { scoreType = scoreType_; }
    int getScoreType() const { return scoreType; }

    void setPatchSize(int patchSize_) { patchSize = patchSize_; }
    int getPatchSize() const { return patchSize; }

    void setFastThreshold(int fastThreshold_) { fastThreshold = fastThreshold_; }
    int getFastThreshold() const { return fastThreshold; }

    // returns the descriptor size in bytes
    int descriptorSize() const;
    // returns the descriptor type
    int descriptorType() const;
    // returns the default norm type
    int defaultNorm() const;

    // Compute the ORB_Impl features and descriptors on an image
    void detectAndCompute( InputArray image, InputArray mask, std::vector<KeyPoint>& keypoints,
                           OutputArray descriptors, bool useProvidedKeypoints=false, float _iso = 200 );

protected:

    int nfeatures;
    double scaleFactor;
    int nlevels;
    int edgeThreshold;
    int firstLevel;
    int wta_k;
    int scoreType;
    int patchSize;
    int fastThreshold;
};


#endif //MFD_MYORB_H
