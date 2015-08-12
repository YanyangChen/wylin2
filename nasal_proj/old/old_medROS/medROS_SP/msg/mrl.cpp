

#include "mrl.h"

MRL::MRL():shmm(NULL), p_shmm(NULL){}

MRL::~MRL(){}

MRL::MRL(int shmmid, int size)
{
    shmm = new share_memory;
    shmm->open_shm(shmmid, size);
    p_shmm = shmm->get_address();
}

int MRL::write(MRLmsg &mrl_msg)
{
    return (write(&mrl_msg));
}

int MRL::write(MRLmsg *mrl_msg)
{
	if (shmm == NULL) return RTN_SHARE_MEMORY_NOT_EXIST;
	if (mrl_msg == NULL || shmm->get_maxsize() <= mrl_msg->size) return RTN_ERROR;

    memcpy(p_shmm, mrl_msg, mrl_msg->size);

	return RTN_OK;
}

MRLmsg* MRL::get_address()
{
    if (shmm == NULL) return NULL;
    return (MRLmsg*) (shmm->get_address());
}
