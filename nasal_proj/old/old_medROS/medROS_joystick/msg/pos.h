#ifndef _POS_H_
#define _POS_H_

#define MAX_DIM_TASK	6
/*
// //! Cartesian coordinate
// typedef struct {
// 	double x, y, z;		
// } PmCartesian;
// 
// //! Cartesian coordinate
// typedef struct PmPose {
//     PmCartesian tran;
//     double a, b, c;
//     double u, v, w;
// } PmPose;
// 
// //! a macro that set all the components of the posture to zero.
// #define ZERO_EMC_POSE(pos) do { \
// 	pos.tran.x = 0.0;               \
// 	pos.tran.y = 0.0;               \
// 	pos.tran.z = 0.0;               \
// 	pos.a = 0.0;                    \
// 	pos.b = 0.0;                    \
// 	pos.c = 0.0;                    \
// 	pos.u = 0.0;                    \
// 	pos.v = 0.0;                    \
// 	pos.w = 0.0;} while(0)

*/

#define	DISTANCE(L, p1, p2, size)	{ \
	L = 0; \
	for(int i=0; i < (size); i++) L += ((p1).s[i] - (p2).s[i])*((p1).s[i] - (p2).s[i]);	\
	L = sqrt(L);	\
}

typedef struct Position{
	double s[MAX_DIM_TASK];
} Position;

typedef struct MotPosition{
	Position pos;
	Position vel;
	Position acc;
}MotPosition;

#endif // end of _POS_H_