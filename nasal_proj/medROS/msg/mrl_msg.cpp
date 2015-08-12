

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "mrl_msg.h"

int MRLmsg::automatically_clear = 1;

MRLmsg::MRLmsg()
{
}

MRLmsg::MRLmsg(MRLTYPE t, long s):
    type(t), size(s)
{
    type = t;
    size = s;
    if (automatically_clear) clear();

    if (size < (long)sizeof(MRLmsg))
    {
        printf("MRLmsg: size(=%ld) must be atleast %lu\n", size, (unsigned long) (sizeof(MRLmsg)) );

        size = sizeof(MRLmsg);
    }

    if (type <= 0)
    {
        printf("NRLmsg: type(=%ld) should be greater than zeroe.\n", (long)(type));
    }
}


MRLmsg::MRLmsg (MRLTYPE t, long s, int noclear):
	type(t),size(s)
{
	if (automatically_clear && !noclear)
    {
		clear ();
    }
	type = t;
	size = s;
	if (size < ((long) sizeof (MRLmsg)))
    {
		printf ("MRLmsg: size(=%ld) must be atleast %lu\n", size,
			(unsigned long) sizeof (MRLmsg));
		size = sizeof (MRLmsg);
    }
	if (type <= 0)
    {
		printf ("MRLmsg: type(=%ld) should be greater than zero.\n", (long) type);
	}
}

void MRLmsg::clear()
{
	long temp_size;
	MRLTYPE temp_type;
	temp_size = size;
	temp_type = type;
	if (size < ((long) sizeof (MRLmsg)))
    {
		printf ("MRLmsg: size(=%ld) must be atleast %lu\n", size,
			(unsigned long) (sizeof (MRLmsg)));
		size = sizeof (MRLmsg);
    }
	else
    {
		memset ((void *) this, 0, size);
		size = temp_size;
		type = temp_type;
    }
}
