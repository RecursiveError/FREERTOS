#ifndef CMDUTIL_HPP
#define CMDUTIL_HPP
#include <stdlib.h>
#include <stdint.h>

#define CMDB '<'
#define CMDE '>'
#define ARGB '('
#define ARGE ')'
#define CMD_MAX_SIZE sizeof(uint64_t)

enum CmdState{
    None,
    Cbegin,
    Cend,
    Abegin,
    Aend,
};

enum TokenResult {
    Pass,
    InvalidSyntax,
    CmdTooLong,
};

typedef struct{
    uint8_t *msg;
    size_t msg_size;
} msg_t;

typedef struct {
    size_t cmd_begin;
    size_t cmd_end;
    size_t arg_begin;
    size_t arg_end;
} CmdToken_t;

typedef union{
    uint8_t raw_cmd[CMD_MAX_SIZE];
    uint64_t hash_cmd;
}CmdHash_t;

typedef struct{
 CmdHash_t hash;
 msg_t args;
}TokenMsg_t;



const msg_t response_ok = {
    .msg = (uint8_t*)"OK!\n\r",
    .msg_size = 6,
};
const msg_t response_err = {
    .msg = (uint8_t*)"ERROR!\n\r",
    .msg_size = 9,
};

int strint(const uint8_t *str, uint32_t *target);
TokenResult token_raw(msg_t msg, CmdToken_t *output);
CmdHash_t create_cmd(CmdToken_t token, msg_t msg);
CmdHash_t create_cmd_from_str(const char* cmd_str);
void parse_num_args(msg_t args, char s, uint64_t *arr_out, size_t arr_out_size);

#endif