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

#include <vector>
#include "prosac.h"

#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <limits.h>
#include <stdlib.h>

namespace umf {


/// Computation of the Maximum number of iterations for Ransac
/// with the formula from [HZ] Section: "How many samples" p.119
static inline
int niter_RANSAC(double p, // probability that at least one of the random samples picked up by RANSAC is free of outliers
                 double epsilon, // proportion of outliers
                 int s, // sample size
                 int Nmax) // upper bound on the number of iterations (-1 means INT_MAX)
{
    // compute safely N = ceil(log(1. - p) / log(1. - exp(log(1.-epsilon) * s)))
    double logarg, logval, N;
    if (Nmax == -1) {
        Nmax = INT_MAX;
    }
    assert(Nmax >= 1);
    if (epsilon <= 0.) {
        return 1;
    }
    // logarg = -(1-epsilon)^s
    logarg = -exp(s*log(1.-epsilon)); // C++/boost version: logarg = -std::pow(1.-epsilon, s);
    // logval = log1p(logarg)) = log(1-(1-epsilon)^s)
    logval = log(1.+logarg); // C++/boost version: logval = boost::math::log1p(logarg)
    N = log(1.-p) / logval;
    if (logval  < 0. && N < Nmax) {
        return (int)ceil(N);
    }
    return Nmax;
}


// * Non-randomness: eq. (7) states that i-m (where i is the cardinal of the set of inliers for a wrong
// model) follows the binomial distribution B(n,beta). http://en.wikipedia.org/wiki/Binomial_distribution
// For n big enough, B(n,beta) ~ N(mu,sigma^2) (central limit theorem),
// with mu = n*beta and sigma = sqrt(n*beta*(1-beta)).
// psi, the probability that In_star out of n_star data points are by chance inliers to an arbitrary
// incorrect model, is set to 0.05 (5%, as in the original paper), and you must change the Chi2 value if
// you chose a different value for psi.
static inline
int Imin(int m, int n, double beta) {
    const double mu = n*beta;
    const double sigma = sqrt(n*beta*(1-beta));
    // Imin(n) (equation (8) can then be obtained with the Chi-squared test with P=2*psi=0.10 (Chi2=2.706)
    return (int)ceil(m + mu + sigma*sqrt(2.706));
}


int Prosac::run(ProsacModel *model, std::vector<bool> &inliers, int maxDraws)
{
    const int N = model->getDataCount();
    std::vector<bool> isInlier(N, false);

    std::vector<int> sample(this->sampleSize, 0);
    const int m = this->sampleSize;
    const int T_N = niter_RANSAC(this->pGoodSample, this->maxOutliersProportion, this->sampleSize, -1);
    const float beta = this->beta;
    int n_star; // termination length (see sec. 2.2 Stopping criterion)
    int I_n_star; // number of inliers found within the first n_star data points
    int I_N_best; // best number of inliers found so far (store the model that goes with it)
    const int I_N_min = static_cast<int> ((1.f-this->maxOutliersProportion)*N); // the minimum number of total inliers
    int t; // iteration number
    int n; // we draw samples from the set U_n of the top n data points
    double T_n; // average number of samples {M_i}_{i=1}^{T_N} that contain samples from U_n only
    int T_n_prime; // integer version of T_n, see eq. (4)
    int k_n_star; // number of samples to draw to reach the maximality constraint
    int i;
    //const double logeta0 = log(this->eta0);

    //prinf("PROSAC sampling test\n");
    //prinf("number of correspondences (N):%d\n", N);
    //prinf("sample size (m):%d\n", m);
    //prinf("showing the first %d draws from PROSAC\n", maxDraws);

    n_star = N;
    I_n_star = 0;
    I_N_best = 0;
    t = 0;
    n = m;
    T_n = T_N;
    for(i=0; i<m; i++) {
        T_n *= (double)(n-i)/(N-i);
        sample[i] = i;
    }
    T_n_prime = 1;
    k_n_star = T_N;
    // Note: the condition (I_N_best < I_N_min) was not in the original paper, but it is reasonable:
    // we sholdn't stop if we haven't found the expected number of inliers
    while(((I_N_best < I_N_min) || t <= k_n_star) && t < T_N && t <= maxDraws) {
        int I_N; // total number of inliers for that sample
        
        // Choice of the hypothesis generation set
        t = t + 1;
        //prinf("Iteration t=%d, ", t);
        // from the paper, eq. (5) (not Algorithm1):
        // "The growth function is then deﬁned as
        //  g(t) = min {n : T′n ≥ t}"
        // Thus n should be incremented if t > T'n, not if t = T'n as written in the algorithm 1
        if ((t > T_n_prime) && (n < n_star)) {
            double T_nplus1 = (T_n * (n+1)) / (n+1-m);
            n = n+1;
            T_n_prime = T_n_prime + static_cast<int>(ceil(T_nplus1 - T_n));
            //prinf("g(t)=n=%d, n_star=%d, T_n-1>=%d, T_n>=%d, T_n'=%d...",
            //       n, n_star, (int)ceil(T_n), (int)ceil(T_nplus1), T_n_prime);
            T_n = T_nplus1;
        }
        else {
            //prinf("g(t)=n=%d, n_star=%d, T_n>=%d, T_n'=%d: ",
            //       n, n_star, (int)ceil(T_n), T_n_prime);
        }
        // Draw semi-random sample (note that the test condition from Algorithm1 in the paper is reversed):
        if (t > T_n_prime) {
            // during the finishing stage (n== n_star && t > T_n_prime), draw a standard RANSAC sample
            // The sample contains m points selected from U_n at random
            for(int i = 0; i < m; i++)
            {
                sample[i] = rand()%n;
            }
            //prinf("Draw %d points from U_%d... ", m, n);
        }
        else {
            for(int i = 0; i < m - 1; i++)
            {
                sample[i] = rand()%(n-1);
            }
            sample[m-1] = ((n-1) < N) ? n-1 : N-1;
            // The sample contains m-1 points selected from U_{n−1} at random and u_n
            //prinf("Draw %d points from U_%d and point u_%d... ", m-1, n-1, n);
        }
        
        // INSERT Compute model parameters p_t from the sample M_t
        //printf("Model parameter estimation... ");

        // INSERT (OPTIONAL): Test for degenerate model configuration (DEGENSAC)
        //                    (i.e. discard the sample if more than 1 model is consistent with the sample)
        // ftp://cmp.felk.cvut.cz/pub/cmp/articles/matas/chum-degen-cvpr05.pdf

        // Find support of the model with parameters p_t
        // From first paragraph of section 2: "The hypotheses are veriﬁed against all data"
        I_N = model->findSupport(sample, isInlier);

        //prinf("found %d inliers!\n", I_N);

        if (I_N > I_N_best) {
            int n_best; // best value found so far in terms of inliers ratio
            int I_n_best; // number of inliers for n_best

            // INSERT (OPTIONAL): Do local optimization, and recompute the support (LO-RANSAC)
            // http://cmp.felk.cvut.cz/~matas/papers/chum-dagm03.pdf
            // for the fundamental matrix, the normalized 8-points algorithm performs very well:
            // http://axiom.anu.edu.au/~hartley/Papers/fundamental/ICCV-final/fundamental.pdf
            // ...
            // I_N = findSupport(/* model, sample, */ N, isInlier);
            
            I_N_best = I_N;

            // INSERT: Store the best model
            
            for(int i = 0; i < N; i++)
            {
                inliers[i] = isInlier[i];
            }
            
            //prinf("************\n Found best samples: ");
            //for(int i = 0; i < m; i++)
            //{
                //inliers[sample[i]] = true;
                //prinf("%i ", sample[i]);
            //}
            //prinf("\n****************\n");

            // Select new termination length n_star if possible, according to Sec. 2.2.
            // Note: the original paper seems to do it each time a new sample is drawn,
            // but this really makes sense only if the new sample is better than the previous ones.
            n_best = N;
            I_n_best = I_N;
            if (0) { // change to if(0) to disable n_star optimization (i.e. draw the same # of samples as RANSAC)
                int n_test; // test value for the termination length
                int I_n_test; // number of inliers for that test value
                double epsilon_n_best = (double)I_n_best/n_best;

                for(n_test = N, I_n_test = I_N; n_test > m; n_test--) { 
                    // Loop invariants:
                    // - I_n_test is the number of inliers for the n_test first correspondences
                    // - n_best is the value between n_test+1 and N that maximizes the ratio I_n_best/n_best
                    assert(n_test >= I_n_test);

                    // * Non-randomness : In >= Imin(n*) (eq. (9))
                    // * Maximality: the number of samples that were drawn so far must be enough
                    // so that the probability of having missed a set of inliers is below eta=0.01.
                    // This is the classical RANSAC termination criterion (HZ 4.7.1.2, eq. (4.18)),
                    // except that it takes into account only the n first samples (not the total number of samples).
                    // kn_star = log(eta0)/log(1-(In_star/n_star)^m) (eq. (12))
                    // We have to minimize kn_star, e.g. maximize I_n_star/n_star
                    //printf("n_best=%d, I_n_best=%d, n_test=%d, I_n_test=%d\n",
                    //        n_best,    I_n_best,    n_test,    I_n_test);
                    // a straightforward implementation would use the following test:
                    //if (I_n_test > epsilon_n_best*n_test) {
                    // However, since In is binomial, and in the case of evenly distributed inliers,
                    // a better test would be to reduce n_star only if there's a significant improvement in
                    // epsilon. Thus we use a Chi-squared test (P=0.10), together with the normal approximation
                    // to the binomial (mu = epsilon_n_star*n_test, sigma=sqrt(n_test*epsilon_n_star*(1-epsilon_n_star)).
                    // There is a significant difference between the two tests (e.g. with the findSupport
                    // functions provided above).
                    // We do the cheap test first, and the expensive test only if the cheap one passes.
                    if (( I_n_test * n_best > I_n_best * n_test ) &&
                        ( I_n_test > epsilon_n_best*n_test + sqrt(n_test*epsilon_n_best*(1.-epsilon_n_best)*2.706) )) {
                        if (I_n_test < Imin(m,n_test,beta)) {
                            // equation 9 not satisfied: no need to test for smaller n_test values anyway
                            break; // jump out of the for(n_test) loop
                        }
                        n_best = n_test;
                        I_n_best = I_n_test;
                        epsilon_n_best = (double)I_n_best/n_best;
                    }

                    // prepare for next loop iteration
                    I_n_test -= isInlier[n_test-1];
                } // for(n_test ...
            } // n_star optimization

            // is the best one we found even better than n_star?
            if ( I_n_best * n_star > I_n_star * n_best ) {
                assert(n_best >= I_n_best);
                // update all values
                n_star = n_best;
                I_n_star = I_n_best;
                k_n_star = niter_RANSAC(1.-this->eta0, 1.-I_n_star/(double)n_star, m, T_N);
                //prinf("new values: n_star=%d, k_n_star=%d, I_n_star=%d, I_min=%d\n", n_star, k_n_star, I_n_star, Imin(m,n_best,beta));
            }
        } // if (I_N > I_N_best)
    } // while(t <= k_n_star ...


    return I_N_best;
    //return m;
}

}
