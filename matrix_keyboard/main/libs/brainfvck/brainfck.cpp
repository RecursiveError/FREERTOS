#include "brainfck.hpp"


bferror_t bf_check(bfprogm_t bfprog){
    uint32_t shift_check = 0, loop_check = 0;
    if(!bfprog.data_buf || !bfprog.prog_buf) return NullBuffer;
    if(!bfprog.data_buf_size || !bfprog.prog_buf_size) return ZeroSize;
    for(uint32_t i = 0; i < bfprog.prog_buf_size; i++){
        switch(bfprog.prog_buf[i]){
            case '>':
                if(++shift_check > bfprog.data_buf_size)return DataBufOverflow;
                break;
            case '<':
                if(shift_check-- == 0)return DataBufUnderflow;
                break;
            case '[':
                loop_check++;
                break;
            case ']':
                if(loop_check-- == 0)return EolLoop;
                break;
            default:
                continue;
        }
    }
    if(loop_check)return EolLoop;
    return BFPass;
}

bfio_t bf_step(bfprogm_t *bfprog){
    bfio_t iostatge = IOPass;
    uint32_t to_find = 0;
    switch(bfprog->prog_buf[bfprog->prog_idx]){
        case '>':
            bfprog->data_idx += 1;
            break;
        case '<':
            bfprog->data_idx -= 1;
            break;
        case '+':
            bfprog->data_buf[bfprog->data_idx] += 1;
            break;
        case '-':
            bfprog->data_buf[bfprog->data_idx] -= 1;
            break;
        case '[':
            if(bfprog->data_buf[bfprog->data_idx] == 0){
                to_find = 1;
                for(uint32_t i = bfprog->prog_idx+1; i <  bfprog->prog_buf_size;i++){
                    if(bfprog->prog_buf[i] == '['){
                        to_find++;
                    }else if((bfprog->prog_buf[i] == ']')){
                        to_find--;
                    }
                    if(to_find == 0){
                        bfprog->prog_idx = i;
                        break;
                    }
                }
            }
            break;
        case ']':
            if(bfprog->data_buf[bfprog->data_idx] != 0){
                to_find = 1;
                for(uint32_t i = bfprog->prog_idx - 1; i > 0;i--){
                    if(bfprog->prog_buf[i] == '['){
                        to_find--;
                    }else if((bfprog->prog_buf[i] == ']')){
                        to_find++;
                    }
                    if(to_find == 0){
                        bfprog->prog_idx = i;
                        break;
                    }
                }
            }
            break;
        case ',':
            iostatge = WaitOutput;
            break;
        case '.':
            iostatge = WaitInput;
            break;
    }
    bfprog->prog_idx += 1;
    return iostatge;
}