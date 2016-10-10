//
// Created by 林奇 on 16/8/5.
//

#include "../include/MyPointSetRegistrator.h"
#include "../include/MyRho.h"
#include <iostream>

namespace rk
{
    class MyLMSolverImpl : public LMSolver
    {
    public:
        MyLMSolverImpl() : maxIters(100) { init(); }
        MyLMSolverImpl(const Ptr<LMSolver::Callback>& _cb, int _maxIters) : cb(_cb), maxIters(_maxIters) { init(); }

        void init()
        {
            epsx = epsf = FLT_EPSILON;
            printInterval = 0;
        }

        int run(InputOutputArray _param0) const
        {
            Mat param0 = _param0.getMat(), x, xd, r, rd, J, A, Ap, v, temp_d, d;
            int ptype = param0.type();

            CV_Assert( (param0.cols == 1 || param0.rows == 1) && (ptype == CV_32F || ptype == CV_64F));
            CV_Assert( cb );

            int lx = param0.rows + param0.cols - 1;
            param0.convertTo(x, CV_64F);

            if( x.cols != 1 )
                transpose(x, x);

            if( !cb->compute(x, r, J) )
                return -1;
            double S = norm(r, NORM_L2SQR);
            int nfJ = 2;

            mulTransposed(J, A, true);
            gemm(J, r, 1, noArray(), 0, v, GEMM_1_T);

            Mat D = A.diag().clone();

            const double Rlo = 0.25, Rhi = 0.75;
            double lambda = 1, lc = 0.75;
            int i, iter = 0;

            if( printInterval != 0 )
            {
                printf("************************************************************************************\n");
                printf("\titr\tnfJ\t\tSUM(r^2)\t\tx\t\tdx\t\tl\t\tlc\n");
                printf("************************************************************************************\n");
            }

            for( ;; )
            {
                CV_Assert( A.type() == CV_64F && A.rows == lx );
                A.copyTo(Ap);
                for( i = 0; i < lx; i++ )
                    Ap.at<double>(i, i) += lambda*D.at<double>(i);
                solve(Ap, v, d, DECOMP_EIG);
                subtract(x, d, xd);
                if( !cb->compute(xd, rd, noArray()) )
                    return -1;
                nfJ++;
                double Sd = norm(rd, NORM_L2SQR);
                gemm(A, d, -1, v, 2, temp_d);
                double dS = d.dot(temp_d);
                double R = (S - Sd)/(fabs(dS) > DBL_EPSILON ? dS : 1);

                if( R > Rhi )
                {
                    lambda *= 0.5;
                    if( lambda < lc )
                        lambda = 0;
                }
                else if( R < Rlo )
                {
                    // find new nu if R too low
                    double t = d.dot(v);
                    double nu = (Sd - S)/(fabs(t) > DBL_EPSILON ? t : 1) + 2;
                    nu = std::min(std::max(nu, 2.), 10.);
                    if( lambda == 0 )
                    {
                        invert(A, Ap, DECOMP_EIG);
                        double maxval = DBL_EPSILON;
                        for( i = 0; i < lx; i++ )
                            maxval = std::max(maxval, std::abs(Ap.at<double>(i,i)));
                        lambda = lc = 1./maxval;
                        nu *= 0.5;
                    }
                    lambda *= nu;
                }

                if( Sd < S )
                {
                    nfJ++;
                    S = Sd;
                    std::swap(x, xd);
                    if( !cb->compute(x, r, J) )
                        return -1;
                    mulTransposed(J, A, true);
                    gemm(J, r, 1, noArray(), 0, v, GEMM_1_T);
                }

                iter++;
                bool proceed = iter < maxIters && norm(d, NORM_INF) >= epsx && norm(r, NORM_INF) >= epsf;

                if( printInterval != 0 && (iter % printInterval == 0 || iter == 1 || !proceed) )
                {
                    printf("%c%10d %10d %15.4e %16.4e %17.4e %16.4e %17.4e\n",
                           (proceed ? ' ' : '*'), iter, nfJ, S, x.at<double>(0), d.at<double>(0), lambda, lc);
                }

                if(!proceed)
                    break;
            }

            if( param0.size != x.size )
                transpose(x, x);

            x.convertTo(param0, ptype);
            if( iter == maxIters )
                iter = -iter;

            return iter;
        }

        void setCallback(const Ptr<LMSolver::Callback>& _cb) { cb = _cb; }

        Ptr<LMSolver::Callback> cb;

        double epsx;
        double epsf;
        int maxIters;
        int printInterval;
    };

    Ptr<LMSolver> createLMSolver(const Ptr<LMSolver::Callback>& cb, int maxIters)
    {
        return makePtr<MyLMSolverImpl>(cb, maxIters);
    }

    int RANSACUpdateNumIters( double p, double ep, int modelPoints, int maxIters )
    {
        if( modelPoints <= 0 )
            CV_Error( Error::StsOutOfRange, "the number of model points should be positive" );

        p = MAX(p, 0.);
        p = MIN(p, 1.);
        ep = MAX(ep, 0.);
        ep = MIN(ep, 1.);

        // avoid inf's & nan's
        double num = MAX(1. - p, DBL_MIN);
        double denom = 1. - std::pow(1. - ep, modelPoints);
        if( denom < DBL_MIN )
            return 0;

        num = std::log(num);
        denom = std::log(denom);

        return denom >= 0 || -num >= maxIters*(-denom) ? maxIters : cvRound(num/denom);
    }


    class RANSACPointSetRegistrator : public PointSetRegistrator
    {
    public:
        RANSACPointSetRegistrator(const Ptr<PointSetRegistrator::Callback>& _cb=Ptr<PointSetRegistrator::Callback>(),
                                  int _modelPoints=0, double _threshold=0, double _confidence=0.99, int _maxIters=1000)
                : cb(_cb), modelPoints(_modelPoints), threshold(_threshold), confidence(_confidence), maxIters(_maxIters)
        {
            checkPartialSubsets = false;
        }

        int findInliers( const Mat& m1, const Mat& m2, const Mat& model, Mat& err, Mat& mask, double thresh ) const
        {
            cb->computeError( m1, m2, model, err );
            mask.create(err.size(), CV_8U);

            CV_Assert( err.isContinuous() && err.type() == CV_32F && mask.isContinuous() && mask.type() == CV_8U);
            const float* errptr = err.ptr<float>();
            uchar* maskptr = mask.ptr<uchar>();
            float t = (float)(thresh*thresh);
            int i, n = (int)err.total(), nz = 0;
            for( i = 0; i < n; i++ )
            {
                int f = errptr[i] <= t;
                maskptr[i] = (uchar)f;
                nz += f;
            }
            return nz;
        }

        bool getSubset( const Mat& m1, const Mat& m2,
                        Mat& ms1, Mat& ms2, RNG& rng,
                        int maxAttempts=1000 ) const
        {
            cv::AutoBuffer<int> _idx(modelPoints);
            int* idx = _idx;
            int i = 0, j, k, iters = 0;
            int esz1 = (int)m1.elemSize(), esz2 = (int)m2.elemSize();
            int d1 = m1.channels() > 1 ? m1.channels() : m1.cols;
            int d2 = m2.channels() > 1 ? m2.channels() : m2.cols;
            int count = m1.checkVector(d1), count2 = m2.checkVector(d2);
            const int *m1ptr = m1.ptr<int>(), *m2ptr = m2.ptr<int>();

            ms1.create(modelPoints, 1, CV_MAKETYPE(m1.depth(), d1));
            ms2.create(modelPoints, 1, CV_MAKETYPE(m2.depth(), d2));

            int *ms1ptr = ms1.ptr<int>(), *ms2ptr = ms2.ptr<int>();

            CV_Assert( count >= modelPoints && count == count2 );
            CV_Assert( (esz1 % sizeof(int)) == 0 && (esz2 % sizeof(int)) == 0 );
            esz1 /= sizeof(int);
            esz2 /= sizeof(int);

            for(; iters < maxAttempts; iters++)
            {
                for( i = 0; i < modelPoints && iters < maxAttempts; )
                {
                    int idx_i = 0;
                    for(;;)
                    {
                        idx_i = idx[i] = rng.uniform(0, count);
                        for( j = 0; j < i; j++ )
                            if( idx_i == idx[j] )
                                break;
                        if( j == i )
                            break;
                    }
                    for( k = 0; k < esz1; k++ )
                        ms1ptr[i*esz1 + k] = m1ptr[idx_i*esz1 + k];
                    for( k = 0; k < esz2; k++ )
                        ms2ptr[i*esz2 + k] = m2ptr[idx_i*esz2 + k];
                    if( checkPartialSubsets && !cb->checkSubset( ms1, ms2, i+1 ))
                    {
                        // we may have selected some bad points;
                        // so, let's remove some of them randomly
                        i = rng.uniform(0, i+1);
                        iters++;
                        continue;
                    }
                    i++;
                }
                if( !checkPartialSubsets && i == modelPoints && !cb->checkSubset(ms1, ms2, i))
                    continue;
                break;
            }

            return i == modelPoints && iters < maxAttempts;
        }

        bool run(InputArray _m1, InputArray _m2, OutputArray _model, OutputArray _mask) const
        {
            bool result = false;
            Mat m1 = _m1.getMat(), m2 = _m2.getMat();
            Mat err, mask, model, bestModel, ms1, ms2;

            int iter, niters = MAX(maxIters, 1);
            int d1 = m1.channels() > 1 ? m1.channels() : m1.cols;
            int d2 = m2.channels() > 1 ? m2.channels() : m2.cols;
            int count = m1.checkVector(d1), count2 = m2.checkVector(d2), maxGoodCount = 0;

            RNG rng((uint64)-1);
            //LOGE("RNG = %d\n",(uint64)-1);

            CV_Assert( cb );
            CV_Assert( confidence > 0 && confidence < 1 );

            CV_Assert( count >= 0 && count2 == count );
            if( count < modelPoints )
                return false;

            Mat bestMask0, bestMask;

            if( _mask.needed() )
            {
                _mask.create(count, 1, CV_8U, -1, true);
                bestMask0 = bestMask = _mask.getMat();
                CV_Assert( (bestMask.cols == 1 || bestMask.rows == 1) && (int)bestMask.total() == count );
            }
            else
            {
                bestMask.create(count, 1, CV_8U);
                bestMask0 = bestMask;
            }

            if( count == modelPoints )
            {
                if( cb->runKernel(m1, m2, bestModel) <= 0 )
                    return false;
                bestModel.copyTo(_model);
                bestMask.setTo(Scalar::all(1));
                return true;
            }

            for( iter = 0; iter < niters; iter++ )
            {
                int i, goodCount, nmodels;
                if( count > modelPoints )
                {
                    bool found = getSubset( m1, m2, ms1, ms2, rng, 10000 );
                    if( !found )
                    {
                        if( iter == 0 )
                            return false;
                        break;
                    }
                }

                nmodels = cb->runKernel( ms1, ms2, model );
                if( nmodels <= 0 )
                    continue;
                CV_Assert( model.rows % nmodels == 0 );
                Size modelSize(model.cols, model.rows/nmodels);

                for( i = 0; i < nmodels; i++ )
                {
                    Mat model_i = model.rowRange( i*modelSize.height, (i+1)*modelSize.height );
                    goodCount = findInliers( m1, m2, model_i, err, mask, threshold );

                    if( goodCount > MAX(maxGoodCount, modelPoints-1) )
                    {
                        std::swap(mask, bestMask);
                        model_i.copyTo(bestModel);
                        maxGoodCount = goodCount;
                        niters = RANSACUpdateNumIters( confidence, (double)(count - goodCount)/count, modelPoints, niters );
                    }
                }
            }

            if( maxGoodCount > 0 )
            {
                if( bestMask.data != bestMask0.data )
                {
                    if( bestMask.size() == bestMask0.size() )
                        bestMask.copyTo(bestMask0);
                    else
                        transpose(bestMask, bestMask0);
                }
                bestModel.copyTo(_model);
                result = true;
            }
            else
                _model.release();

            return result;
        }

        void setCallback(const Ptr<PointSetRegistrator::Callback>& _cb) { cb = _cb; }

        Ptr<PointSetRegistrator::Callback> cb;
        int modelPoints;
        bool checkPartialSubsets;
        double threshold;
        double confidence;
        int maxIters;
    };

    class LMeDSPointSetRegistrator : public RANSACPointSetRegistrator
    {
    public:
        LMeDSPointSetRegistrator(const Ptr<PointSetRegistrator::Callback>& _cb=Ptr<PointSetRegistrator::Callback>(),
                                 int _modelPoints=0, double _confidence=0.99, int _maxIters=1000)
                : RANSACPointSetRegistrator(_cb, _modelPoints, 0, _confidence, _maxIters) {}

        bool run(InputArray _m1, InputArray _m2, OutputArray _model, OutputArray _mask) const
        {
            const double outlierRatio = 0.45;
            bool result = false;
            Mat m1 = _m1.getMat(), m2 = _m2.getMat();
            Mat ms1, ms2, err, errf, model, bestModel, mask, mask0;

            int d1 = m1.channels() > 1 ? m1.channels() : m1.cols;
            int d2 = m2.channels() > 1 ? m2.channels() : m2.cols;
            int count = m1.checkVector(d1), count2 = m2.checkVector(d2);
            double minMedian = DBL_MAX, sigma;

            RNG rng((uint64)-1);

            CV_Assert( cb );
            CV_Assert( confidence > 0 && confidence < 1 );

            CV_Assert( count >= 0 && count2 == count );
            if( count < modelPoints )
                return false;

            if( _mask.needed() )
            {
                _mask.create(count, 1, CV_8U, -1, true);
                mask0 = mask = _mask.getMat();
                CV_Assert( (mask.cols == 1 || mask.rows == 1) && (int)mask.total() == count );
            }

            if( count == modelPoints )
            {
                if( cb->runKernel(m1, m2, bestModel) <= 0 )
                    return false;
                bestModel.copyTo(_model);
                mask.setTo(Scalar::all(1));
                return true;
            }

            int iter, niters = RANSACUpdateNumIters(confidence, outlierRatio, modelPoints, maxIters);
            niters = MAX(niters, 3);

            for( iter = 0; iter < niters; iter++ )
            {
                int i, nmodels;
                if( count > modelPoints )
                {
                    bool found = getSubset( m1, m2, ms1, ms2, rng );
                    if( !found )
                    {
                        if( iter == 0 )
                            return false;
                        break;
                    }
                }

                nmodels = cb->runKernel( ms1, ms2, model );
                if( nmodels <= 0 )
                    continue;

                CV_Assert( model.rows % nmodels == 0 );
                Size modelSize(model.cols, model.rows/nmodels);

                for( i = 0; i < nmodels; i++ )
                {
                    Mat model_i = model.rowRange( i*modelSize.height, (i+1)*modelSize.height );
                    cb->computeError( m1, m2, model_i, err );
                    if( err.depth() != CV_32F )
                        err.convertTo(errf, CV_32F);
                    else
                        errf = err;
                    CV_Assert( errf.isContinuous() && errf.type() == CV_32F && (int)errf.total() == count );
                    std::sort(errf.ptr<int>(), errf.ptr<int>() + count);

                    double median = count % 2 != 0 ?
                                    errf.at<float>(count/2) : (errf.at<float>(count/2-1) + errf.at<float>(count/2))*0.5;

                    if( median < minMedian )
                    {
                        minMedian = median;
                        model_i.copyTo(bestModel);
                    }
                }
            }

            if( minMedian < DBL_MAX )
            {
                sigma = 2.5*1.4826*(1 + 5./(count - modelPoints))*std::sqrt(minMedian);
                sigma = MAX( sigma, 0.001 );

                count = findInliers( m1, m2, bestModel, err, mask, sigma );
                if( _mask.needed() && mask0.data != mask.data )
                {
                    if( mask0.size() == mask.size() )
                        mask.copyTo(mask0);
                    else
                        transpose(mask, mask0);
                }
                bestModel.copyTo(_model);
                result = count >= modelPoints;
            }
            else
                _model.release();

            return result;
        }

    };

    Ptr<PointSetRegistrator> createRANSACPointSetRegistrator(const Ptr<PointSetRegistrator::Callback>& _cb,
                                                             int _modelPoints, double _threshold,
                                                             double _confidence, int _maxIters)
    {
        return Ptr<PointSetRegistrator>(
                new RANSACPointSetRegistrator(_cb, _modelPoints, _threshold, _confidence, _maxIters));
    }


    Ptr<PointSetRegistrator> createLMeDSPointSetRegistrator(const Ptr<PointSetRegistrator::Callback>& _cb,
                                                            int _modelPoints, double _confidence, int _maxIters)
    {
        return Ptr<PointSetRegistrator>(
                new LMeDSPointSetRegistrator(_cb, _modelPoints, _confidence, _maxIters));
    }


    class Affine3DEstimatorCallback : public PointSetRegistrator::Callback
    {
    public:
        int runKernel( InputArray _m1, InputArray _m2, OutputArray _model ) const
        {
            Mat m1 = _m1.getMat(), m2 = _m2.getMat();
            const Point3f* from = m1.ptr<Point3f>();
            const Point3f* to   = m2.ptr<Point3f>();

            const int N = 12;
            double buf[N*N + N + N];
            Mat A(N, N, CV_64F, &buf[0]);
            Mat B(N, 1, CV_64F, &buf[0] + N*N);
            Mat X(N, 1, CV_64F, &buf[0] + N*N + N);
            double* Adata = A.ptr<double>();
            double* Bdata = B.ptr<double>();
            A = Scalar::all(0);

            for( int i = 0; i < (N/3); i++ )
            {
                Bdata[i*3] = to[i].x;
                Bdata[i*3+1] = to[i].y;
                Bdata[i*3+2] = to[i].z;

                double *aptr = Adata + i*3*N;
                for(int k = 0; k < 3; ++k)
                {
                    aptr[0] = from[i].x;
                    aptr[1] = from[i].y;
                    aptr[2] = from[i].z;
                    aptr[3] = 1.0;
                    aptr += 16;
                }
            }

            solve(A, B, X, DECOMP_SVD);
            X.reshape(1, 3).copyTo(_model);

            return 1;
        }

        void computeError( InputArray _m1, InputArray _m2, InputArray _model, OutputArray _err ) const
        {
            Mat m1 = _m1.getMat(), m2 = _m2.getMat(), model = _model.getMat();
            const Point3f* from = m1.ptr<Point3f>();
            const Point3f* to   = m2.ptr<Point3f>();
            const double* F = model.ptr<double>();

            int count = m1.checkVector(3);
            CV_Assert( count > 0 );

            _err.create(count, 1, CV_32F);
            Mat err = _err.getMat();
            float* errptr = err.ptr<float>();

            for(int i = 0; i < count; i++ )
            {
                const Point3f& f = from[i];
                const Point3f& t = to[i];

                double a = F[0]*f.x + F[1]*f.y + F[ 2]*f.z + F[ 3] - t.x;
                double b = F[4]*f.x + F[5]*f.y + F[ 6]*f.z + F[ 7] - t.y;
                double c = F[8]*f.x + F[9]*f.y + F[10]*f.z + F[11] - t.z;

                errptr[i] = (float)std::sqrt(a*a + b*b + c*c);
            }
        }

        bool checkSubset( InputArray _ms1, InputArray _ms2, int count ) const
        {
            const float threshold = 0.996f;
            Mat ms1 = _ms1.getMat(), ms2 = _ms2.getMat();

            for( int inp = 1; inp <= 2; inp++ )
            {
                int j, k, i = count - 1;
                const Mat* msi = inp == 1 ? &ms1 : &ms2;
                const Point3f* ptr = msi->ptr<Point3f>();

                CV_Assert( count <= msi->rows );

                // check that the i-th selected point does not belong
                // to a line connecting some previously selected points
                for(j = 0; j < i; ++j)
                {
                    Point3f d1 = ptr[j] - ptr[i];
                    float n1 = d1.x*d1.x + d1.y*d1.y;

                    for(k = 0; k < j; ++k)
                    {
                        Point3f d2 = ptr[k] - ptr[i];
                        float denom = (d2.x*d2.x + d2.y*d2.y)*n1;
                        float num = d1.x*d2.x + d1.y*d2.y;

                        if( num*num > threshold*threshold*denom )
                            return false;
                    }
                }
            }
            return true;
        }
    };

}


static bool haveCollinearPoints( const Mat& m, int count )
    {
        int j, k, i = count-1;
        const Point2f* ptr = m.ptr<Point2f>();

        // check that the i-th selected point does not belong
        // to a line connecting some previously selected points
        for( j = 0; j < i; j++ )
        {
            double dx1 = ptr[j].x - ptr[i].x;
            double dy1 = ptr[j].y - ptr[i].y;
            for( k = 0; k < j; k++ )
            {
                double dx2 = ptr[k].x - ptr[i].x;
                double dy2 = ptr[k].y - ptr[i].y;
                if( fabs(dx2*dy1 - dy2*dx1) <= FLT_EPSILON*(fabs(dx1) + fabs(dy1) + fabs(dx2) + fabs(dy2)))
                    return true;
            }
        }
        return false;
    }


    class HomographyEstimatorCallback : public rk::PointSetRegistrator::Callback
    {
    public:
        bool checkSubset( InputArray _ms1, InputArray _ms2, int count ) const
        {
            Mat ms1 = _ms1.getMat(), ms2 = _ms2.getMat();
            if( haveCollinearPoints(ms1, count) || haveCollinearPoints(ms2, count) )
                return false;

            // We check whether the minimal set of points for the homography estimation
            // are geometrically consistent. We check if every 3 correspondences sets
            // fulfills the constraint.
            //
            // The usefullness of this constraint is explained in the paper:
            //
            // "Speeding-up homography estimation in mobile devices"
            // Journal of Real-Time Image Processing. 2013. DOI: 10.1007/s11554-012-0314-1
            // Pablo Marquez-Neila, Javier Lopez-Alberca, Jose M. Buenaposada, Luis Baumela
            if( count == 4 )
            {
                static const int tt[][3] = {{0, 1, 2}, {1, 2, 3}, {0, 2, 3}, {0, 1, 3}};
                const Point2f* src = ms1.ptr<Point2f>();
                const Point2f* dst = ms2.ptr<Point2f>();
                int negative = 0;

                for( int i = 0; i < 4; i++ )
                {
                    const int* t = tt[i];
                    Matx33d A(src[t[0]].x, src[t[0]].y, 1., src[t[1]].x, src[t[1]].y, 1., src[t[2]].x, src[t[2]].y, 1.);
                    Matx33d B(dst[t[0]].x, dst[t[0]].y, 1., dst[t[1]].x, dst[t[1]].y, 1., dst[t[2]].x, dst[t[2]].y, 1.);

                    negative += determinant(A)*determinant(B) < 0;
                }
                if( negative != 0 && negative != 4 )
                    return false;
            }

            return true;
        }

        int runKernel( InputArray _m1, InputArray _m2, OutputArray _model ) const
        {
            Mat m1 = _m1.getMat(), m2 = _m2.getMat();
            int i, count = m1.checkVector(2);
            const Point2f* M = m1.ptr<Point2f>();
            const Point2f* m = m2.ptr<Point2f>();

            double LtL[9][9], W[9][1], V[9][9];
            Mat _LtL( 9, 9, CV_64F, &LtL[0][0] );
            Mat matW( 9, 1, CV_64F, W );
            Mat matV( 9, 9, CV_64F, V );
            Mat _H0( 3, 3, CV_64F, V[8] );
            Mat _Htemp( 3, 3, CV_64F, V[7] );
            Point2d cM(0,0), cm(0,0), sM(0,0), sm(0,0);

            for( i = 0; i < count; i++ )
            {
                cm.x += m[i].x; cm.y += m[i].y;
                cM.x += M[i].x; cM.y += M[i].y;
            }

            cm.x /= count;
            cm.y /= count;
            cM.x /= count;
            cM.y /= count;

            for( i = 0; i < count; i++ )
            {
                sm.x += fabs(m[i].x - cm.x);
                sm.y += fabs(m[i].y - cm.y);
                sM.x += fabs(M[i].x - cM.x);
                sM.y += fabs(M[i].y - cM.y);
            }

            if( fabs(sm.x) < DBL_EPSILON || fabs(sm.y) < DBL_EPSILON ||
                fabs(sM.x) < DBL_EPSILON || fabs(sM.y) < DBL_EPSILON )
                return 0;
            sm.x = count/sm.x; sm.y = count/sm.y;
            sM.x = count/sM.x; sM.y = count/sM.y;

            double invHnorm[9] = { 1./sm.x, 0, cm.x, 0, 1./sm.y, cm.y, 0, 0, 1 };
            double Hnorm2[9] = { sM.x, 0, -cM.x*sM.x, 0, sM.y, -cM.y*sM.y, 0, 0, 1 };
            Mat _invHnorm( 3, 3, CV_64FC1, invHnorm );
            Mat _Hnorm2( 3, 3, CV_64FC1, Hnorm2 );

            _LtL.setTo(Scalar::all(0));
            for( i = 0; i < count; i++ )
            {
                double x = (m[i].x - cm.x)*sm.x, y = (m[i].y - cm.y)*sm.y;
                double X = (M[i].x - cM.x)*sM.x, Y = (M[i].y - cM.y)*sM.y;
                double Lx[] = { X, Y, 1, 0, 0, 0, -x*X, -x*Y, -x };
                double Ly[] = { 0, 0, 0, X, Y, 1, -y*X, -y*Y, -y };
                int j, k;
                for( j = 0; j < 9; j++ )
                    for( k = j; k < 9; k++ )
                        LtL[j][k] += Lx[j]*Lx[k] + Ly[j]*Ly[k];
            }
            completeSymm( _LtL );

            eigen( _LtL, matW, matV );
            _Htemp = _invHnorm*_H0;
            _H0 = _Htemp*_Hnorm2;
            _H0.convertTo(_model, _H0.type(), 1./_H0.at<double>(2,2) );

            return 1;
        }

        void computeError( InputArray _m1, InputArray _m2, InputArray _model, OutputArray _err ) const
        {
            Mat m1 = _m1.getMat(), m2 = _m2.getMat(), model = _model.getMat();
            int i, count = m1.checkVector(2);
            const Point2f* M = m1.ptr<Point2f>();
            const Point2f* m = m2.ptr<Point2f>();
            const double* H = model.ptr<double>();
            float Hf[] = { (float)H[0], (float)H[1], (float)H[2], (float)H[3], (float)H[4], (float)H[5], (float)H[6], (float)H[7] };

            _err.create(count, 1, CV_32F);
            float* err = _err.getMat().ptr<float>();

            for( i = 0; i < count; i++ )
            {
                float ww = 1.f/(Hf[6]*M[i].x + Hf[7]*M[i].y + 1.f);
                float dx = (Hf[0]*M[i].x + Hf[1]*M[i].y + Hf[2])*ww - m[i].x;
                float dy = (Hf[3]*M[i].x + Hf[4]*M[i].y + Hf[5])*ww - m[i].y;
                err[i] = (float)(dx*dx + dy*dy);
            }
        }
    };


    class HomographyRefineCallback : public rk::LMSolver::Callback
    {
    public:
        HomographyRefineCallback(InputArray _src, InputArray _dst)
        {
            src = _src.getMat();
            dst = _dst.getMat();
        }

        bool compute(InputArray _param, OutputArray _err, OutputArray _Jac) const
        {
            int i, count = src.checkVector(2);
            Mat param = _param.getMat();
            _err.create(count*2, 1, CV_64F);
            Mat err = _err.getMat(), J;
            if( _Jac.needed())
            {
                _Jac.create(count*2, param.rows, CV_64F);
                J = _Jac.getMat();
                CV_Assert( J.isContinuous() && J.cols == 8 );
            }

            const Point2f* M = src.ptr<Point2f>();
            const Point2f* m = dst.ptr<Point2f>();
            const double* h = param.ptr<double>();
            double* errptr = err.ptr<double>();
            double* Jptr = J.data ? J.ptr<double>() : 0;

            for( i = 0; i < count; i++ )
            {
                double Mx = M[i].x, My = M[i].y;
                double ww = h[6]*Mx + h[7]*My + 1.;
                ww = fabs(ww) > DBL_EPSILON ? 1./ww : 0;
                double xi = (h[0]*Mx + h[1]*My + h[2])*ww;
                double yi = (h[3]*Mx + h[4]*My + h[5])*ww;
                errptr[i*2] = xi - m[i].x;
                errptr[i*2+1] = yi - m[i].y;

                if( Jptr )
                {
                    Jptr[0] = Mx*ww; Jptr[1] = My*ww; Jptr[2] = ww;
                    Jptr[3] = Jptr[4] = Jptr[5] = 0.;
                    Jptr[6] = -Mx*ww*xi; Jptr[7] = -My*ww*xi;
                    Jptr[8] = Jptr[9] = Jptr[10] = 0.;
                    Jptr[11] = Mx*ww; Jptr[12] = My*ww; Jptr[13] = ww;
                    Jptr[14] = -Mx*ww*yi; Jptr[15] = -My*ww*yi;

                    Jptr += 16;
                }
            }

            return true;
        }

        Mat src, dst;
    };



namespace rk{
    static bool createAndRunRHORegistrator(double confidence,
                                           int    maxIters,
                                           double ransacReprojThreshold,
                                           int    npoints,
                                           InputArray  _src,
                                           InputArray  _dst,
                                           OutputArray _H,
                                           OutputArray _tempMask){
        Mat    src = _src.getMat();
        Mat    dst = _dst.getMat();
        Mat    tempMask;
        bool   result;
        double beta = 0.35;/* 0.35 is a value that often works. */

        /* Create temporary output matrix (RHO outputs a single-precision H only). */
        Mat tmpH = Mat(3, 3, CV_32FC1);

        /* Create output mask. */
        tempMask = Mat(npoints, 1, CV_8U);

        /**
         * Make use of the RHO estimator API.
         *
         * This is where the math happens. A homography estimation context is
         * initialized, used, then finalized.
         */

        Ptr<RHO_HEST> p = rhoInit();

        /**
         * Optional. Ideally, the context would survive across calls to
         * findHomography(), but no clean way appears to exit to do so. The price
         * to pay is marginally more computational work than strictly needed.
         */

        rhoEnsureCapacity(p, npoints, beta);

        /**
         * The critical call. All parameters are heavily documented in rhorefc.h.
         *
         * Currently, NR (Non-Randomness criterion) and Final Refinement (with
         * internal, optimized Levenberg-Marquardt method) are enabled. However,
         * while refinement seems to correctly smooth jitter most of the time, when
         * refinement fails it tends to make the estimate visually very much worse.
         * It may be necessary to remove the refinement flags in a future commit if
         * this behaviour is too problematic.
         */

        result = !!rhoHest(p,
                           (const float*)src.data,
                           (const float*)dst.data,
                           (char*)       tempMask.data,
                           (unsigned)    npoints,
                           (float)       ransacReprojThreshold,
                           (unsigned)    maxIters,
                           (unsigned)    maxIters,
                           confidence,
                           4U,
                           beta,
                           RHO_FLAG_ENABLE_NR | RHO_FLAG_ENABLE_FINAL_REFINEMENT,
                           NULL,
                           (float*)tmpH.data);

        /* Convert float homography to double precision. */
        tmpH.convertTo(_H, CV_64FC1);

        /* Maps non-zero mask elems to 1, for the sake of the testcase. */
        for(int k=0;k<npoints;k++){
            tempMask.data[k] = !!tempMask.data[k];
        }
        tempMask.copyTo(_tempMask);

        return result;
    }
}


cv::Mat rk::MyFindHomography( InputArray _points1, InputArray _points2,
                            int method, double ransacReprojThreshold, OutputArray _mask,
                            const int maxIters, const double confidence)
{
    const double defaultRANSACReprojThreshold = 3;
    bool result = false;

    Mat points1 = _points1.getMat(), points2 = _points2.getMat();
    Mat src, dst, H, tempMask;
    int npoints = -1;

    for( int i = 1; i <= 2; i++ )
    {
        Mat& p = i == 1 ? points1 : points2;
        Mat& m = i == 1 ? src : dst;
        npoints = p.checkVector(2, -1, false);
        if( npoints < 0 )
        {
            npoints = p.checkVector(3, -1, false);
            if( npoints < 0 )
                CV_Error(Error::StsBadArg, "The input arrays should be 2D or 3D point sets");
            if( npoints == 0 )
                return Mat();
            convertPointsFromHomogeneous(p, p);
        }
        p.reshape(2, npoints).convertTo(m, CV_32F);
    }

    CV_Assert( src.checkVector(2) == dst.checkVector(2) );

    if( ransacReprojThreshold <= 0 )
        ransacReprojThreshold = defaultRANSACReprojThreshold;

    Ptr<rk::PointSetRegistrator::Callback> cb = makePtr<HomographyEstimatorCallback>();

    if( method == 0 || npoints == 4 )
    {
        tempMask = Mat::ones(npoints, 1, CV_8U);
        result = cb->runKernel(src, dst, H) > 0;
    }
    else if( method == RANSAC )
        result = rk::createRANSACPointSetRegistrator(cb, 4, ransacReprojThreshold, confidence, maxIters)->run(src, dst, H, tempMask);
    else if( method == LMEDS )
//        assert("LMEDS is not support!");
        result = rk::createLMeDSPointSetRegistrator(cb, 4, confidence, maxIters)->run(src, dst, H, tempMask);
    else if( method == RHO )
//        assert("RHO is not support!");
        result = rk::createAndRunRHORegistrator(confidence, maxIters, ransacReprojThreshold, npoints, src, dst, H, tempMask);
    else
        CV_Error(Error::StsBadArg, "Unknown estimation method");

    if( result && npoints > 4 && method != RHO)
    {
        rk::compressElems( src.ptr<Point2f>(), tempMask.ptr<uchar>(), 1, npoints );
        npoints = rk::compressElems( dst.ptr<Point2f>(), tempMask.ptr<uchar>(), 1, npoints );
        if( npoints > 0 )
        {
            Mat src1 = src.rowRange(0, npoints);
            Mat dst1 = dst.rowRange(0, npoints);
            src = src1;
            dst = dst1;
            if( method == RANSAC || method == LMEDS )
                cb->runKernel( src, dst, H );
            Mat H8(8, 1, CV_64F, H.ptr<double>());
            rk::createLMSolver(makePtr<HomographyRefineCallback>(src, dst), 10)->run(H8);
        }
    }

    if( result )
    {
        if( _mask.needed() )
            tempMask.copyTo(_mask);
    }
    else
        H.release();

    return H;
}

//cv::Mat MyFindHomography( InputArray _points1, InputArray _points2,
//                            OutputArray _mask, int method, double ransacReprojThreshold )
//{
//    return MyFindHomography(_points1, _points2, method, ransacReprojThreshold, _mask);
//}



/* Estimation of Fundamental Matrix from point correspondences.
   The original code has been written by Valery Mosyagin */

/* The algorithms (except for RANSAC) and the notation have been taken from
   Zhengyou Zhang's research report
   "Determining the Epipolar Geometry and its Uncertainty: A Review"
   that can be found at http://www-sop.inria.fr/robotvis/personnel/zzhang/zzhang-eng.html */

/************************************** 7-point algorithm *******************************/
namespace rk
{

    static int run7Point( const Mat& _m1, const Mat& _m2, Mat& _fmatrix )
    {
        double a[7*9], w[7], u[9*9], v[9*9], c[4], r[3];
        double* f1, *f2;
        double t0, t1, t2;
        Mat A( 7, 9, CV_64F, a );
        Mat U( 7, 9, CV_64F, u );
        Mat Vt( 9, 9, CV_64F, v );
        Mat W( 7, 1, CV_64F, w );
        Mat coeffs( 1, 4, CV_64F, c );
        Mat roots( 1, 3, CV_64F, r );
        const Point2f* m1 = _m1.ptr<Point2f>();
        const Point2f* m2 = _m2.ptr<Point2f>();
        double* fmatrix = _fmatrix.ptr<double>();
        int i, k, n;

        // form a linear system: i-th row of A(=a) represents
        // the equation: (m2[i], 1)'*F*(m1[i], 1) = 0
        for( i = 0; i < 7; i++ )
        {
            double x0 = m1[i].x, y0 = m1[i].y;
            double x1 = m2[i].x, y1 = m2[i].y;

            a[i*9+0] = x1*x0;
            a[i*9+1] = x1*y0;
            a[i*9+2] = x1;
            a[i*9+3] = y1*x0;
            a[i*9+4] = y1*y0;
            a[i*9+5] = y1;
            a[i*9+6] = x0;
            a[i*9+7] = y0;
            a[i*9+8] = 1;
        }

        // A*(f11 f12 ... f33)' = 0 is singular (7 equations for 9 variables), so
        // the solution is linear subspace of dimensionality 2.
        // => use the last two singular vectors as a basis of the space
        // (according to SVD properties)
        SVDecomp( A, W, U, Vt, SVD::MODIFY_A + SVD::FULL_UV );
        f1 = v + 7*9;
        f2 = v + 8*9;

        // f1, f2 is a basis => lambda*f1 + mu*f2 is an arbitrary f. matrix.
        // as it is determined up to a scale, normalize lambda & mu (lambda + mu = 1),
        // so f ~ lambda*f1 + (1 - lambda)*f2.
        // use the additional constraint det(f) = det(lambda*f1 + (1-lambda)*f2) to find lambda.
        // it will be a cubic equation.
        // find c - polynomial coefficients.
        for( i = 0; i < 9; i++ )
            f1[i] -= f2[i];

        t0 = f2[4]*f2[8] - f2[5]*f2[7];
        t1 = f2[3]*f2[8] - f2[5]*f2[6];
        t2 = f2[3]*f2[7] - f2[4]*f2[6];

        c[3] = f2[0]*t0 - f2[1]*t1 + f2[2]*t2;

        c[2] = f1[0]*t0 - f1[1]*t1 + f1[2]*t2 -
               f1[3]*(f2[1]*f2[8] - f2[2]*f2[7]) +
               f1[4]*(f2[0]*f2[8] - f2[2]*f2[6]) -
               f1[5]*(f2[0]*f2[7] - f2[1]*f2[6]) +
               f1[6]*(f2[1]*f2[5] - f2[2]*f2[4]) -
               f1[7]*(f2[0]*f2[5] - f2[2]*f2[3]) +
               f1[8]*(f2[0]*f2[4] - f2[1]*f2[3]);

        t0 = f1[4]*f1[8] - f1[5]*f1[7];
        t1 = f1[3]*f1[8] - f1[5]*f1[6];
        t2 = f1[3]*f1[7] - f1[4]*f1[6];

        c[1] = f2[0]*t0 - f2[1]*t1 + f2[2]*t2 -
               f2[3]*(f1[1]*f1[8] - f1[2]*f1[7]) +
               f2[4]*(f1[0]*f1[8] - f1[2]*f1[6]) -
               f2[5]*(f1[0]*f1[7] - f1[1]*f1[6]) +
               f2[6]*(f1[1]*f1[5] - f1[2]*f1[4]) -
               f2[7]*(f1[0]*f1[5] - f1[2]*f1[3]) +
               f2[8]*(f1[0]*f1[4] - f1[1]*f1[3]);

        c[0] = f1[0]*t0 - f1[1]*t1 + f1[2]*t2;

        // solve the cubic equation; there can be 1 to 3 roots ...
        n = solveCubic( coeffs, roots );

        if( n < 1 || n > 3 )
            return n;

        for( k = 0; k < n; k++, fmatrix += 9 )
        {
            // for each root form the fundamental matrix
            double lambda = r[k], mu = 1.;
            double s = f1[8]*r[k] + f2[8];

            // normalize each matrix, so that F(3,3) (~fmatrix[8]) == 1
            if( fabs(s) > DBL_EPSILON )
            {
                mu = 1./s;
                lambda *= mu;
                fmatrix[8] = 1.;
            }
            else
                fmatrix[8] = 0.;

            for( i = 0; i < 8; i++ )
                fmatrix[i] = f1[i]*lambda + f2[i]*mu;
        }

        return n;
    }


    static int run8Point( const Mat& _m1, const Mat& _m2, Mat& _fmatrix )
    {
        Point2d m1c(0,0), m2c(0,0);
        double t, scale1 = 0, scale2 = 0;

        const Point2f* m1 = _m1.ptr<Point2f>();
        const Point2f* m2 = _m2.ptr<Point2f>();
        CV_Assert( (_m1.cols == 1 || _m1.rows == 1) && _m1.size() == _m2.size());
        int i, count = _m1.checkVector(2);

        // compute centers and average distances for each of the two point sets
        for( i = 0; i < count; i++ )
        {
            m1c += Point2d(m1[i]);
            m2c += Point2d(m2[i]);
        }

        // calculate the normalizing transformations for each of the point sets:
        // after the transformation each set will have the mass center at the coordinate origin
        // and the average distance from the origin will be ~sqrt(2).
        t = 1./count;
        m1c *= t;
        m2c *= t;

        for( i = 0; i < count; i++ )
        {
            scale1 += norm(Point2d(m1[i].x - m1c.x, m1[i].y - m1c.y));
            scale2 += norm(Point2d(m2[i].x - m2c.x, m2[i].y - m2c.y));
        }

        scale1 *= t;
        scale2 *= t;

        if( scale1 < FLT_EPSILON || scale2 < FLT_EPSILON )
            return 0;

        scale1 = std::sqrt(2.)/scale1;
        scale2 = std::sqrt(2.)/scale2;

        Matx<double, 9, 9> A;

        // form a linear system Ax=0: for each selected pair of points m1 & m2,
        // the row of A(=a) represents the coefficients of equation: (m2, 1)'*F*(m1, 1) = 0
        // to save computation time, we compute (At*A) instead of A and then solve (At*A)x=0.
        for( i = 0; i < count; i++ )
        {
            double x1 = (m1[i].x - m1c.x)*scale1;
            double y1 = (m1[i].y - m1c.y)*scale1;
            double x2 = (m2[i].x - m2c.x)*scale2;
            double y2 = (m2[i].y - m2c.y)*scale2;
            Vec<double, 9> r( x2*x1, x2*y1, x2, y2*x1, y2*y1, y2, x1, y1, 1 );
            A += r*r.t();
        }

        Vec<double, 9> W;
        Matx<double, 9, 9> V;

        eigen(A, W, V);

        for( i = 0; i < 9; i++ )
        {
            if( fabs(W[i]) < DBL_EPSILON )
                break;
        }

        if( i < 8 )
            return 0;

        Matx33d F0( V.val + 9*8 ); // take the last column of v as a solution of Af = 0

        // make F0 singular (of rank 2) by decomposing it with SVD,
        // zeroing the last diagonal element of W and then composing the matrices back.

        Vec3d w;
        Matx33d U;
        Matx33d Vt;

        SVD::compute( F0, w, U, Vt);
        w[2] = 0.;

        F0 = U*Matx33d::diag(w)*Vt;

        // apply the transformation that is inverse
        // to what we used to normalize the point coordinates
        Matx33d T1( scale1, 0, -scale1*m1c.x, 0, scale1, -scale1*m1c.y, 0, 0, 1 );
        Matx33d T2( scale2, 0, -scale2*m2c.x, 0, scale2, -scale2*m2c.y, 0, 0, 1 );

        F0 = T2.t()*F0*T1;

        // make F(3,3) = 1
        if( fabs(F0(2,2)) > FLT_EPSILON )
            F0 *= 1./F0(2,2);

        Mat(F0).copyTo(_fmatrix);

        return 1;
    }


    class FMEstimatorCallback : public PointSetRegistrator::Callback
    {
    public:
        bool checkSubset( InputArray _ms1, InputArray _ms2, int count ) const
        {
            Mat ms1 = _ms1.getMat(), ms2 = _ms2.getMat();
            return !haveCollinearPoints(ms1, count) && !haveCollinearPoints(ms2, count);
        }

        int runKernel( InputArray _m1, InputArray _m2, OutputArray _model ) const
        {
            double f[9*3];
            Mat m1 = _m1.getMat(), m2 = _m2.getMat();
            int count = m1.checkVector(2);
            Mat F(count == 7 ? 9 : 3, 3, CV_64F, f);
            int n = count == 7 ? run7Point(m1, m2, F) : run8Point(m1, m2, F);

            if( n == 0 )
                _model.release();
            else
                F.rowRange(0, n*3).copyTo(_model);

            return n;
        }

        void computeError( InputArray _m1, InputArray _m2, InputArray _model, OutputArray _err ) const
        {
            Mat __m1 = _m1.getMat(), __m2 = _m2.getMat(), __model = _model.getMat();
            int i, count = __m1.checkVector(2);
            const Point2f* m1 = __m1.ptr<Point2f>();
            const Point2f* m2 = __m2.ptr<Point2f>();
            const double* F = __model.ptr<double>();
            _err.create(count, 1, CV_32F);
            float* err = _err.getMat().ptr<float>();

            for( i = 0; i < count; i++ )
            {
                double a, b, c, d1, d2, s1, s2;

                a = F[0]*m1[i].x + F[1]*m1[i].y + F[2];
                b = F[3]*m1[i].x + F[4]*m1[i].y + F[5];
                c = F[6]*m1[i].x + F[7]*m1[i].y + F[8];

                s2 = 1./(a*a + b*b);
                d2 = m2[i].x*a + m2[i].y*b + c;

                a = F[0]*m2[i].x + F[3]*m2[i].y + F[6];
                b = F[1]*m2[i].x + F[4]*m2[i].y + F[7];
                c = F[2]*m2[i].x + F[5]*m2[i].y + F[8];

                s1 = 1./(a*a + b*b);
                d1 = m1[i].x*a + m1[i].y*b + c;

                err[i] = (float)std::max(d1*d1*s1, d2*d2*s2);
            }
        }
    };

}

cv::Mat rk::MyFindFundamentalMat( InputArray _points1, InputArray _points2,
                                int method, double param1, double param2,
                                OutputArray _mask )
{
    Mat points1 = _points1.getMat(), points2 = _points2.getMat();
    Mat m1, m2, F;
    int npoints = -1;

    for( int i = 1; i <= 2; i++ )
    {
        Mat& p = i == 1 ? points1 : points2;
        Mat& m = i == 1 ? m1 : m2;
        npoints = p.checkVector(2, -1, false);
        if( npoints < 0 )
        {
            npoints = p.checkVector(3, -1, false);
            if( npoints < 0 )
                CV_Error(Error::StsBadArg, "The input arrays should be 2D or 3D point sets");
            if( npoints == 0 )
                return Mat();
            convertPointsFromHomogeneous(p, p);
        }
        p.reshape(2, npoints).convertTo(m, CV_32F);
    }

    CV_Assert( m1.checkVector(2) == m2.checkVector(2) );

    if( npoints < 7 )
        return Mat();

    Ptr<rk::PointSetRegistrator::Callback> cb = makePtr<rk::FMEstimatorCallback>();
    int result;

    if( npoints == 7 || method == FM_8POINT )
    {
        result = cb->runKernel(m1, m2, F);
        if( _mask.needed() )
        {
            _mask.create(npoints, 1, CV_8U, -1, true);
            Mat mask = _mask.getMat();
            CV_Assert( (mask.cols == 1 || mask.rows == 1) && (int)mask.total() == npoints );
            mask.setTo(Scalar::all(1));
        }
    }
    else
    {
        if( param1 <= 0 )
            param1 = 3;
        if( param2 < DBL_EPSILON || param2 > 1 - DBL_EPSILON )
            param2 = 0.99;

        if( (method & ~3) == FM_RANSAC && npoints >= 15 )
            result = rk::createRANSACPointSetRegistrator(cb, 7, param1, param2)->run(m1, m2, F, _mask);
        else
            result = rk::createLMeDSPointSetRegistrator(cb, 7, param2)->run(m1, m2, F, _mask);
    }

    if( result <= 0 )
        return Mat();

    return F;
}

cv::Mat MyFindFundamentalMat( InputArray _points1, InputArray _points2,
                                OutputArray _mask, int method, double param1, double param2 )
{
    return rk::MyFindFundamentalMat(_points1, _points2, method, param1, param2, _mask);
}


void computeCorrespondEpilines( InputArray _points, int whichImage,
                                    InputArray _Fmat, OutputArray _lines )
{
    double f[9];
    Mat tempF(3, 3, CV_64F, f);
    Mat points = _points.getMat(), F = _Fmat.getMat();

    if( !points.isContinuous() )
        points = points.clone();

    int npoints = points.checkVector(2);
    if( npoints < 0 )
    {
        npoints = points.checkVector(3);
        if( npoints < 0 )
            CV_Error( Error::StsBadArg, "The input should be a 2D or 3D point set");
        Mat temp;
        convertPointsFromHomogeneous(points, temp);
        points = temp;
    }
    int depth = points.depth();
    CV_Assert( depth == CV_32F || depth == CV_32S || depth == CV_64F );

    CV_Assert(F.size() == Size(3,3));
    F.convertTo(tempF, CV_64F);
    if( whichImage == 2 )
        transpose(tempF, tempF);

    int ltype = CV_MAKETYPE(MAX(depth, CV_32F), 3);
    _lines.create(npoints, 1, ltype);
    Mat lines = _lines.getMat();
    if( !lines.isContinuous() )
    {
        _lines.release();
        _lines.create(npoints, 1, ltype);
        lines = _lines.getMat();
    }
    CV_Assert( lines.isContinuous());

    if( depth == CV_32S || depth == CV_32F )
    {
        const Point* ptsi = points.ptr<Point>();
        const Point2f* ptsf = points.ptr<Point2f>();
        Point3f* dstf = lines.ptr<Point3f>();
        for( int i = 0; i < npoints; i++ )
        {
            Point2f pt = depth == CV_32F ? ptsf[i] : Point2f((float)ptsi[i].x, (float)ptsi[i].y);
            double a = f[0]*pt.x + f[1]*pt.y + f[2];
            double b = f[3]*pt.x + f[4]*pt.y + f[5];
            double c = f[6]*pt.x + f[7]*pt.y + f[8];
            double nu = a*a + b*b;
            nu = nu ? 1./std::sqrt(nu) : 1.;
            a *= nu; b *= nu; c *= nu;
            dstf[i] = Point3f((float)a, (float)b, (float)c);
        }
    }
    else
    {
        const Point2d* ptsd = points.ptr<Point2d>();
        Point3d* dstd = lines.ptr<Point3d>();
        for( int i = 0; i < npoints; i++ )
        {
            Point2d pt = ptsd[i];
            double a = f[0]*pt.x + f[1]*pt.y + f[2];
            double b = f[3]*pt.x + f[4]*pt.y + f[5];
            double c = f[6]*pt.x + f[7]*pt.y + f[8];
            double nu = a*a + b*b;
            nu = nu ? 1./std::sqrt(nu) : 1.;
            a *= nu; b *= nu; c *= nu;
            dstd[i] = Point3d(a, b, c);
        }
    }
}

void convertPointsFromHomogeneous( InputArray _src, OutputArray _dst )
{
    Mat src = _src.getMat();
    if( !src.isContinuous() )
        src = src.clone();
    int i, npoints = src.checkVector(3), depth = src.depth(), cn = 3;
    if( npoints < 0 )
    {
        npoints = src.checkVector(4);
        CV_Assert(npoints >= 0);
        cn = 4;
    }
    CV_Assert( npoints >= 0 && (depth == CV_32S || depth == CV_32F || depth == CV_64F));

    int dtype = CV_MAKETYPE(depth <= CV_32F ? CV_32F : CV_64F, cn-1);
    _dst.create(npoints, 1, dtype);
    Mat dst = _dst.getMat();
    if( !dst.isContinuous() )
    {
        _dst.release();
        _dst.create(npoints, 1, dtype);
        dst = _dst.getMat();
    }
    CV_Assert( dst.isContinuous() );

    if( depth == CV_32S )
    {
        if( cn == 3 )
        {
            const Point3i* sptr = src.ptr<Point3i>();
            Point2f* dptr = dst.ptr<Point2f>();
            for( i = 0; i < npoints; i++ )
            {
                float scale = sptr[i].z != 0 ? 1.f/sptr[i].z : 1.f;
                dptr[i] = Point2f(sptr[i].x*scale, sptr[i].y*scale);
            }
        }
        else
        {
            const Vec4i* sptr = src.ptr<Vec4i>();
            Point3f* dptr = dst.ptr<Point3f>();
            for( i = 0; i < npoints; i++ )
            {
                float scale = sptr[i][3] != 0 ? 1.f/sptr[i][3] : 1.f;
                dptr[i] = Point3f(sptr[i][0]*scale, sptr[i][1]*scale, sptr[i][2]*scale);
            }
        }
    }
    else if( depth == CV_32F )
    {
        if( cn == 3 )
        {
            const Point3f* sptr = src.ptr<Point3f>();
            Point2f* dptr = dst.ptr<Point2f>();
            for( i = 0; i < npoints; i++ )
            {
                float scale = sptr[i].z != 0.f ? 1.f/sptr[i].z : 1.f;
                dptr[i] = Point2f(sptr[i].x*scale, sptr[i].y*scale);
            }
        }
        else
        {
            const Vec4f* sptr = src.ptr<Vec4f>();
            Point3f* dptr = dst.ptr<Point3f>();
            for( i = 0; i < npoints; i++ )
            {
                float scale = sptr[i][3] != 0.f ? 1.f/sptr[i][3] : 1.f;
                dptr[i] = Point3f(sptr[i][0]*scale, sptr[i][1]*scale, sptr[i][2]*scale);
            }
        }
    }
    else if( depth == CV_64F )
    {
        if( cn == 3 )
        {
            const Point3d* sptr = src.ptr<Point3d>();
            Point2d* dptr = dst.ptr<Point2d>();
            for( i = 0; i < npoints; i++ )
            {
                double scale = sptr[i].z != 0. ? 1./sptr[i].z : 1.;
                dptr[i] = Point2d(sptr[i].x*scale, sptr[i].y*scale);
            }
        }
        else
        {
            const Vec4d* sptr = src.ptr<Vec4d>();
            Point3d* dptr = dst.ptr<Point3d>();
            for( i = 0; i < npoints; i++ )
            {
                double scale = sptr[i][3] != 0.f ? 1./sptr[i][3] : 1.;
                dptr[i] = Point3d(sptr[i][0]*scale, sptr[i][1]*scale, sptr[i][2]*scale);
            }
        }
    }
    else
        CV_Error(Error::StsUnsupportedFormat, "");
}


void convertPointsToHomogeneous( InputArray _src, OutputArray _dst )
{
    Mat src = _src.getMat();
    if( !src.isContinuous() )
        src = src.clone();
    int i, npoints = src.checkVector(2), depth = src.depth(), cn = 2;
    if( npoints < 0 )
    {
        npoints = src.checkVector(3);
        CV_Assert(npoints >= 0);
        cn = 3;
    }
    CV_Assert( npoints >= 0 && (depth == CV_32S || depth == CV_32F || depth == CV_64F));

    int dtype = CV_MAKETYPE(depth <= CV_32F ? CV_32F : CV_64F, cn+1);
    _dst.create(npoints, 1, dtype);
    Mat dst = _dst.getMat();
    if( !dst.isContinuous() )
    {
        _dst.release();
        _dst.create(npoints, 1, dtype);
        dst = _dst.getMat();
    }
    CV_Assert( dst.isContinuous() );

    if( depth == CV_32S )
    {
        if( cn == 2 )
        {
            const Point2i* sptr = src.ptr<Point2i>();
            Point3i* dptr = dst.ptr<Point3i>();
            for( i = 0; i < npoints; i++ )
                dptr[i] = Point3i(sptr[i].x, sptr[i].y, 1);
        }
        else
        {
            const Point3i* sptr = src.ptr<Point3i>();
            Vec4i* dptr = dst.ptr<Vec4i>();
            for( i = 0; i < npoints; i++ )
                dptr[i] = Vec4i(sptr[i].x, sptr[i].y, sptr[i].z, 1);
        }
    }
    else if( depth == CV_32F )
    {
        if( cn == 2 )
        {
            const Point2f* sptr = src.ptr<Point2f>();
            Point3f* dptr = dst.ptr<Point3f>();
            for( i = 0; i < npoints; i++ )
                dptr[i] = Point3f(sptr[i].x, sptr[i].y, 1.f);
        }
        else
        {
            const Point3f* sptr = src.ptr<Point3f>();
            Vec4f* dptr = dst.ptr<Vec4f>();
            for( i = 0; i < npoints; i++ )
                dptr[i] = Vec4f(sptr[i].x, sptr[i].y, sptr[i].z, 1.f);
        }
    }
    else if( depth == CV_64F )
    {
        if( cn == 2 )
        {
            const Point2d* sptr = src.ptr<Point2d>();
            Point3d* dptr = dst.ptr<Point3d>();
            for( i = 0; i < npoints; i++ )
                dptr[i] = Point3d(sptr[i].x, sptr[i].y, 1.);
        }
        else
        {
            const Point3d* sptr = src.ptr<Point3d>();
            Vec4d* dptr = dst.ptr<Vec4d>();
            for( i = 0; i < npoints; i++ )
                dptr[i] = Vec4d(sptr[i].x, sptr[i].y, sptr[i].z, 1.);
        }
    }
    else
        CV_Error(Error::StsUnsupportedFormat, "");
}


//void convertPointsHomogeneous( InputArray _src, OutputArray _dst )
//{
//    int stype = _src.type(), dtype = _dst.type();
//    CV_Assert( _dst.fixedType() );
//
//    if( CV_MAT_CN(stype) > CV_MAT_CN(dtype) )
//        convertPointsFromHomogeneous(_src, _dst);
//    else
//        convertPointsToHomogeneous(_src, _dst);
//}

double sampsonDistance(InputArray _pt1, InputArray _pt2, InputArray _F) {
    CV_Assert(_pt1.type() == CV_64F && _pt1.type() == CV_64F && _F.type() == CV_64F);
    CV_DbgAssert(_pt1.rows() == 3 && _F.size() == Size(3, 3) && _pt1.rows() == _pt2.rows());

    Mat pt1(_pt1.getMat());
    Mat pt2(_pt2.getMat());
    Mat F(_F.getMat());

    Vec3d F_pt1 = *F.ptr<Matx33d>() * *pt1.ptr<Vec3d>();
    Vec3d Ft_pt2 = F.ptr<Matx33d>()->t() * *pt2.ptr<Vec3d>();

    double v = pt2.ptr<Vec3d>()->dot(F_pt1);

    // square
    Ft_pt2 = Ft_pt2.mul(Ft_pt2);
    F_pt1 = F_pt1.mul(F_pt1);

    return v*v / (F_pt1[0] + F_pt1[1] + Ft_pt2[0] + Ft_pt2[1]);
}
