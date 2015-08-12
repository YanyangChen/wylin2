

#ifndef __STAT_MSG_H_
#define __STAT_MSG_H_

#include "mrl.h"
#include "mrl_msg.h"

enum RBT_STATUS
    {
        UNINITIALIZED_STATUS = -1,
        RBT_DONE = 1,
        RBT_EXEC = 2,
        RBT_ERROR = 3
    };

class RBT_STAT_MSG : public MRLmsg
{
public:
	RBT_STAT_MSG(MRLTYPE t, long s);
	MRLTYPE command_type;			///< Ö¸ÁîÀàÐÍ
	int echo_serial_number;			///< ÏìÓ¦µÄÖ¸ÁîºÅ
	RBT_STATUS status;				///< ÔËÐÐ×´Ì¬
	int state;						///< Ö¸Áî×´Ì¬
};

class RBT_STAT_CHANNEL : public MRL
{
public:
	RBT_STAT_CHANNEL() {};
	~RBT_STAT_CHANNEL(){};

	RBT_STAT_CHANNEL(int shmmid, int size):MRL(shmmid, size) {};

public:
	RBT_STAT_MSG *get_address () {
		return ((RBT_STAT_MSG*) MRL::get_address ());
	};

};

#endif // __STAT_MSG_H_
