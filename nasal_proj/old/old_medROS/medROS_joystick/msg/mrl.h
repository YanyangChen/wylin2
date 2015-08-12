// MiniROS, mrl
// created by wylin, zrwang
// date: 2014-7-10

#ifndef __MRL_H_
#define __MRL_H_

#include "return_code.h"
#include "shrmm.h"
#include "mrl_msg.h"

class MRL
{
public:
    MRL();
    ~MRL();

    MRL(int shmmid, int size);

public:
    int write(MRLmsg* mrl_msg);
    int write(MRLmsg& mrl_msg);
    MRLmsg* get_address();

private:
    share_memory* shmm;
    void* p_shmm;
};

#endif //__MRL_H_
