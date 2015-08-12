
#ifndef __CMD_MSG_H_
#define __CMD_MSG_H_

#include "mrl.h"
#include "mrl_msg.h"

class RBT_CMD_MSG : public MRLmsg
{
public:
    RBT_CMD_MSG(MRLTYPE t, long s);
    int serial_number;
};


class RBT_CMD_CHANNEL : public MRL
{
public:
    RBT_CMD_CHANNEL();
    ~RBT_CMD_CHANNEL();
    RBT_CMD_CHANNEL(int shmmid, int size):MRL(shmmid, size){};

public:
    RBT_CMD_MSG* get_address() {
        return ((RBT_CMD_MSG*) MRL::get_address());
    };
};

#endif // __CMD_MSG_H_
