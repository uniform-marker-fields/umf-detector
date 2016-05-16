/*
 Copyright (c) I. Szentandrasi 2016
 Based on: prosac.c
 
 version 1.3 (Sep. 22, 2011)
 
 Author: Frederic Devernay <Frederic.Devernay@inria.fr>

 Description: a sample implementation of the PROSAC sampling algorithm, derived from RANSAC.
 
 Reference:
 O. Chum and J. Matas.
 Matching with PROSAC - progressive sample consensus.
 Proc. of Conference on Computer Vision and Pattern Recognition (CVPR), volume 1, pages 220-226,
 Los Alamitos, California, USA, June 2005.
 ftp://cmp.felk.cvut.cz/pub/cmp/articles/matas/chum-prosac-cvpr05.pdf
 
 Note:
 In the above article, the test is step 2 of Algorithm 1 seem to be reversed, and the test in step 1
 is not consistent with the text. They were corrected in this implementation.

 History:
 version 1.0 (Apr. 14, 2009): initial version
 version 1.1 (Apr. 22, 2009): fix support computation, "The hypotheses are veriï¬ed against all data"
 version 1.2 (Mar. 16, 2011): Add comments about beta, psi, and set eta0 to its original value (0.05 rather than 0.01)
 version 1.3 (Sep. 22, 2011): Check that k_n_star is never nore than T_N
 version 1.4 (Sep. 24, 2011): Don't stop until we have found at least the expected number of inliers (improvement over original PROSAC).
 version 1.5 (Oct. 10, 2011): Also stop if t > T_N (maximum number of iterations given the apriori proportion of outliers).
 version 1.6 (Oct. 10, 2011): Rewrite niter_RANSAC() and also use it to update k_n_star.
 */

#ifndef _UMF_PROSAC_H_
#define _UMF_PROSAC_H_

namespace umf {

class ProsacModel
{
public:
    virtual unsigned int findSupport(std::vector<int> &samples, std::vector<bool> &inliers) = 0;
    virtual unsigned int getDataCount() = 0;
};

class Prosac {
public:
    Prosac(int sampleSize, // m: minimum sample size to compute model (7 for 7-pt F-matrix)
        float maxOutliersProportion = 0.8f, // maximum allowed outliers proportion in the input data: used to compute T_N (can be as high as 0.95)
        float pGoodSample = 0.99f, // probability that at least one of the random samples picked up by RANSAC is free of outliers
        float beta = 0.01f,  // beta is the probability that a match is declared inlier by mistake
        float eta0 = 0.05f // eta0 is the maximum probability that a solution with more than In_star inliers in Un_star exists and was not found
                          // after k samples
        ) : sampleSize(sampleSize), maxOutliersProportion(maxOutliersProportion), pGoodSample(pGoodSample), beta(beta), eta0(eta0)
    {}

    int run(ProsacModel *model, std::vector<bool> &inliers, int maxDraws = 60000);// the max number of draws performed by this test

private:
    int sampleSize;
    float maxOutliersProportion;
    float pGoodSample;
    float beta;
    float eta0;
};

}

#endif