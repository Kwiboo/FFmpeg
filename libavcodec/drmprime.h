#ifndef DRMPRIME_H
#define DRMPRIME_H

#include <stdint.h>

#define FF_DRMPRIME_NUM_PLANES	4	// maximum number of planes

typedef struct {

    uint32_t strides[FF_DRMPRIME_NUM_PLANES];
    uint32_t offsets[FF_DRMPRIME_NUM_PLANES];
    uint32_t fd;
    uint32_t format;

} av_drmprime;

#endif // DRMPRIME_H
