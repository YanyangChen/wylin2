#include "shrmm.h"
#include "return_code.h"

int share_memory::open_shm(int share_memory_key, int share_memory_size)
{
    int id;
    if (share_memory_key <= 0) return RTN_CREATE_SHARE_MEMORY_FAIL;
    id = shmget(share_memory_key, share_memory_size, 0666 | IPC_CREAT);
    if (id == -1) return RTN_CREATE_SHARE_MEMORY_FAIL;
    shmid = id;
    size = share_memory_size;

    addr = shmat(shmid, (void *)0, 0);
    return RTN_OK;
}

int share_memory::close_shm()
{
    int rtn;
    rtn = shmdt(addr);
    return rtn;
}

void* share_memory::get_address()
{
    if (addr == NULL) return NULL;
    return (void*) addr;
}

int share_memory::get_maxsize()
{
    return size;
}