#ifndef _UMF_ROBUST_PLANAR_H_
#define _UMF_ROBUST_PLANAR_H_

/**
 * Copyright (c) I. Szentandrasi 2016
 * Robust pose estimation from Schweighofer and Pinz.
 * The code is based upon the matlab code by pinz and OpenCV implementation
 * from Nghi Ho. License below. (https://github.com/spchuang/AugmentedRealityGo)
 * It has been rewritten to use Eigen library instead.
 *
 *   This is a C++ port using OpenCV of the Robust Pose Estimation from a Planar Target algorithm by Gerald Schweighofer and Axel Pinz.
 *   It is a line by line port of the Matlab code at
 *   http://www.emt.tugraz.at/~pinz/code/
 *   I have no idea what their license is, if any. I'll make my code BSD for now unless I later find they have a more restrictive one.
 *
 *
 *Copyright 2011 Nghia Ho. All rights reserved.
 *Redistribution and use in source and binary forms, with or without modification, are
 *permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice, this list of
 *      conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice, this list
 *      of conditions and the following disclaimer in the documentation and/or other materials
 *      provided with the distribution.
 *THIS SOFTWARE IS PROVIDED BY NGHIA HO ''AS IS'' AND ANY EXPRESS OR IMPLIED
 *WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 *FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL NGHIA HO OR
 *CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 *NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 *ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *The views and conclusions contained in the software and documentation are those of the
 *authors and should not be interpreted as representing official policies, either expressed
 *or implied, of Nghia Ho.
*/


namespace umf
{

typedef struct solution
{
    double R[3][3];
    double t[3];
    double E;
} PoseSolution;

int get2ndPose(int corr_count, double *projected, double *model,
               double rotation[3][3], double translation[3], //initial pose
               int *results, PoseSolution **solutions); //number of solutions and themselves


} //namespace
#endif
