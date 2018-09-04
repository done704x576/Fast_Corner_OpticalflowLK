#include "stdafx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "opticalflow.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

#define PATCH_RADIUS 8 //L-K mask

static int IMW = 0;
static int IMH = 0;
static xyrect ROI;
static unsigned char* previous_src = NULL;
static float* Ix = NULL;
static float* Iy = NULL;
static float* It = NULL;


void init_opticalflow_estimation
    (
    const unsigned char* src,
    const int imw,
    const int imh,
    const xyrect roi
    )
{
    IMW = imw;
    IMH = imh;
    previous_src = (unsigned char*)malloc(IMW * IMH * sizeof(unsigned char));
    memcpy(previous_src, src, IMW * IMH * sizeof(unsigned char));
    ROI = roi;  
    Ix = (float*)malloc(IMW * IMH * sizeof(float));
    Iy = (float*)malloc(IMW * IMH * sizeof(float));
    It = (float*)malloc(IMW * IMH * sizeof(float));
}

void free_opticalflow_estimation
    (
    void
    )
{
    free(previous_src);
    free(Ix);
    free(Iy);
    free(It);
}

void opticalflow_estimation
    (
    const unsigned char* src,
    const xy* corner_start,
    const int NUMCORNERS,
    xy* corner_end
    )
{
    int x = 0;
    int y = 0;
    int whole_idx = 0;
    int ix = 0;
    int previous_ix = 0;
    int iy = 0;
    int previous_iy = 0;
    int corner_idx = 0;
    float flow_u = 0.0f;
    float flow_v = 0.0f;
    //generate gradient map
    for (y = ROI.tl.y + 1; y < ROI.br.y; ++y)
        {
        for (x = ROI.tl.x + 1; x < ROI.br.x; ++x)
            {
            whole_idx = y * IMW + x;
            ix = src[whole_idx + 1] - src[whole_idx - 1];
            previous_ix = previous_src[whole_idx + 1] - previous_src[whole_idx - 1];
            iy = src[whole_idx + IMW] - src[whole_idx - IMW];
            previous_iy = previous_src[whole_idx + IMW] - previous_src[whole_idx - IMW];
            Ix[whole_idx] = (ix + previous_ix) * 0.5f;
            Iy[whole_idx] = (iy + previous_iy) * 0.5f;
            It[whole_idx] = (float)(src[whole_idx] - previous_src[whole_idx]);
            }
        }

    for (corner_idx = 0; corner_idx < NUMCORNERS; ++corner_idx)
        {
        float sumIxIx = 0;
        float sumIxIy = 0;
        float sumIyIy = 0;
        float sumIxIt = 0;
        float sumIyIt = 0;
        //build overdetermined linear system
        for (y = -PATCH_RADIUS; y <= PATCH_RADIUS; ++y)
            {
            for (x = -PATCH_RADIUS; x <= PATCH_RADIUS; ++x)
                {
                whole_idx = (corner_start[corner_idx].y + y) * IMW + (corner_start[corner_idx].x + x);
                sumIxIx += Ix[whole_idx] * Ix[whole_idx];
                sumIyIy += Iy[whole_idx] * Iy[whole_idx];
                sumIxIy += Ix[whole_idx] * Iy[whole_idx];
                sumIxIt += Ix[whole_idx] * It[whole_idx];
                sumIyIt += Iy[whole_idx] * It[whole_idx];
                }
            }
        float determinant = sumIxIx * sumIyIy - sumIxIy * sumIxIy;
        //if A^T*A is invertible
        if (determinant != 0.0f)
            {
            float determinant_inverse = 1.0f / determinant;
            flow_u = (sumIxIy * sumIyIt - sumIyIy * sumIxIt) * determinant_inverse;
            flow_v = (sumIxIy * sumIxIt - sumIxIx * sumIyIt) * determinant_inverse;
            }
        float absflow_u = fabs(flow_u);
        float absflow_v = fabs(flow_v);
        if (PATCH_RADIUS > absflow_u && PATCH_RADIUS > absflow_v && 0.05f < absflow_u && 0.05f < absflow_v)
            {
            corner_end[corner_idx].x -= (int)(flow_u * 5.0f);
            corner_end[corner_idx].y -= (int)(flow_v * 5.0f);
            }
        }
    //save last frame
    memcpy(previous_src, src, IMW * IMH * sizeof(unsigned char));
}