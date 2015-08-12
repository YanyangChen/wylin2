
#ifndef __MRLMSG_H_
#define __MRLMSG_H_

#include "mrl_type.h"

class MRL;

class MRLmsg
{
public:
    MRLmsg(MRLTYPE t, long s);
    MRLmsg(MRLTYPE t, long s , int noclear);
    MRLmsg();

public:
    void clear();
    static int automatically_clear;

    MRLTYPE type;
    long    size;

    friend class MRL;
};

#endif // __MRLMSG_H_