#include <cmduitl.hpp>
#include <string.h>

int strint(const uint8_t *str, uint32_t *target){
    uint32_t c = 0;
    for(size_t i = 0; str[i] != '\n'; i++){
        if(str[i] < '0' || str[i] > '9'){
            return -1;
        }
        c = (c*10) + (str[i] - '0');
    }
    *target = c;
    return 0;
}

TokenResult token_raw(msg_t msg, CmdToken_t *output){
    CmdToken_t token;
    CmdState state = CmdState::None;
    for(size_t i = 0; i < msg.msg_size; i++){
        switch(state){
            case None:
                if(msg.msg[i] == CMDB){
                    state = Cbegin;
                        token.cmd_begin = i;
                    }
            break;
            case Cbegin:
                if(msg.msg[i] == CMDE){
                    state = Cend;
                    token.cmd_end = i;
                    if((i - token.cmd_begin) > CMD_MAX_SIZE){
                        return TokenResult::CmdTooLong;
                    }
                }
            break;
            case Cend:
                if(msg.msg[i] == ARGB){
                    state = Abegin;
                    token.arg_begin = i;
                }
            break;
            case Abegin:
                if(msg.msg[i] == ARGE){
                    state = Aend;
                    token.arg_end = i;
                }
            break;
            case Aend:
            break;
        }

    }
    if(state != CmdState::Aend){
        return TokenResult::InvalidSyntax;
    }
    *output = token;
    return TokenResult::Pass;
}

CmdHash_t create_cmd(CmdToken_t token, msg_t msg){
    CmdHash_t cmd_out = {.hash_cmd = 0};
    size_t begin = token.cmd_begin+1;
    size_t end = token.cmd_end;
    for(size_t i = begin; i < end; i++){
        cmd_out.raw_cmd[i-begin] = msg.msg[i];
    }
    return cmd_out;
}

CmdHash_t create_cmd_from_str(const char* cmd_str){
    CmdHash_t cmd_out = {.hash_cmd = 0};
    if(strlen(cmd_str) < CMD_MAX_SIZE){
        strcpy((char*)cmd_out.raw_cmd, cmd_str);
    }
    return cmd_out;
}

void parse_num_args(msg_t args, char s, uint64_t *arr_out, size_t arr_out_size){
    uint64_t num = 0;
    uint8_t* args_raw = args.msg;
    size_t out_index = 0;
    uint8_t ac_char;
    for(size_t i = 0; i<args.msg_size; i++){
        ac_char = args.msg[i];
        if(ac_char >= '0' && ac_char <= '9'){
            num = (num*10) + (ac_char - '0');
        }else if(ac_char == s){
            arr_out[out_index] = num;
            out_index = (out_index + 1)%arr_out_size;
            num = 0;
        }
    }
    arr_out[out_index] = num;
}