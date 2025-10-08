#ifndef __ZEROCHECKTASK_H
#define __ZEROCHECKTASK_H

#define Position 1

typedef struct 
{
	float Circle;           
	float CountCycle;       
	float LastValue;        
	float ActualValue;      
	float PreError;        
} ZeroCheck_Typedef;

float ZeroCheck(ZeroCheck_Typedef *Zero,float value,short Zerocheck_mode);
void ZeroCheck_cal(void);
void ZeroCheck_Init(void);

#endif

