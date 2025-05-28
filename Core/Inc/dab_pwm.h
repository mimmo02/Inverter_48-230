#ifndef DAB_PWMH
#define DAB_PWMH

typedef struct{
	
	// input variables
	
	float d0;
	float dshift;
	
	// output variables
	
	float don[4];
	float doff[4];
	
	int enableH[4];
	int enableL[4];
	
} TDAB;

void ComputeDonoffTriangle1( TDAB *dab);
void ComputeDonoffTriangle2( TDAB *dab);

#endif
