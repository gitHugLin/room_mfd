//
// Created by 林奇 on 16/8/5.
//

#ifndef MFD_MYPOINTSETREGISTRATOR_H
#define MFD_MYPOINTSETREGISTRATOR_H


#include "MyAlgorithm.h"
#include <algorithm>
#include <iterator>
#include <limits>
#include "log.h"


namespace rk
{

    int RANSACUpdateNumIters( double p, double ep, int modelPoints, int maxIters );

    class LMSolver : public MyAlgorithm
    {
    public:
        class Callback
        {
        public:
            virtual ~Callback() {}
            virtual bool compute(InputArray param, OutputArray err, OutputArray J) const = 0;
        };

        virtual void setCallback(const Ptr<LMSolver::Callback>& cb) = 0;
        virtual int run(InputOutputArray _param0) const = 0;
    };

    Ptr<LMSolver> createLMSolver(const Ptr<LMSolver::Callback>& cb, int maxIters);

    class PointSetRegistrator : public MyAlgorithm
    {
    public:
        class Callback
        {
        public:
            virtual ~Callback() {}
            virtual int runKernel(InputArray m1, InputArray m2, OutputArray model) const = 0;
            virtual void computeError(InputArray m1, InputArray m2, InputArray model, OutputArray err) const = 0;
            virtual bool checkSubset(InputArray, InputArray, int) const { return true; }
        };

        virtual void setCallback(const Ptr<PointSetRegistrator::Callback>& cb) = 0;
        virtual bool run(InputArray m1, InputArray m2, OutputArray model, OutputArray mask) const = 0;
    };

    Ptr<PointSetRegistrator> createRANSACPointSetRegistrator(const Ptr<PointSetRegistrator::Callback>& cb,
                                                                        int modelPoints, double threshold,
                                                                        double confidence=0.99, int maxIters=1000 );

    Ptr<PointSetRegistrator> createLMeDSPointSetRegistrator(const Ptr<PointSetRegistrator::Callback>& cb,
                                                                       int modelPoints, double confidence=0.99, int maxIters=1000 );

    template<typename T> inline int compressElems( T* ptr, const uchar* mask, int mstep, int count )
    {
        int i, j;
        for( i = j = 0; i < count; i++ )
            if( mask[i*mstep] )
            {
                if( i > j )
                    ptr[j] = ptr[i];
                j++;
            }
        return j;
    }
    cv::Mat MyFindHomography( InputArray _points1, InputArray _points2,
                                int method, double ransacReprojThreshold = 3,
                                OutputArray mask = noArray(),
                              const int 	maxIters = 2000,
                              const double 	confidence = 0.995);

    cv::Mat MyFindFundamentalMat( InputArray _points1, InputArray _points2,
                                  int method, double param1, double param2,
                                  OutputArray _mask );
}

#endif //MFD_MYPOINTSETREGISTRATOR_H
