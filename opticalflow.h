#ifndef OPTICALFLOW_H
#define OPTICALFLOW_H

#include "fast.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
    {
    xy tl;
    xy br;
    }xyrect;

void init_opticalflow_estimation
    (
    const unsigned char* src,
    const int imw,
    const int imh,
    const xyrect roi
    );

void free_opticalflow_estimation
    (
    void
    );

void opticalflow_estimation
    (
    const unsigned char* src,
    const xy* corner_start,
    const int NUMCORNERS,
    xy* corner_end
    );

#ifdef __cplusplus
}
#endif

#endif