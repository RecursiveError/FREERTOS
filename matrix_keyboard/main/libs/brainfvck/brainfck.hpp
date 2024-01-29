#ifndef BRIANFCK_HPP
#define BRIANFCK_HPP

#include <stdint.h>


typedef struct {
    uint8_t *prog_buf;
    uint8_t *data_buf;
    uint32_t prog_buf_size;
    uint32_t data_buf_size;
    uint32_t prog_idx;
    uint32_t data_idx;
}bfprogm_t;

typedef enum {
    NullBuffer,
    ZeroSize,
    DataBufUnderflow,
    DataBufOverflow,
    EolLoop,
    BFPass,
}bferror_t;

typedef enum {
    IOPass,
    WaitInput,
    WaitOutput,
}bfio_t;


/// @brief simple error check for brainf*ck
/// @param bfprog
/// @return error in the bf program
bferror_t bf_check(bfprogm_t bfprog);
bfio_t bf_step(bfprogm_t *bfprog);

#endif