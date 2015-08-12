#ifndef  __SHRMM_H_
#define  __SHRMM_H_

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include  <sys/types.h>
#include  <sys/ipc.h>
#include  <sys/shm.h>

class share_memory
{
public:
    share_memory() {addr = NULL; shmid=0; size=0;};
    int open_shm(int share_memory_key, int share_memory_size);
    int close_shm();
    int get_maxsize();
    void* get_address();

private:
    int shmid;
    int size;
    void* addr;
};

#endif // __SHRMM_H_
