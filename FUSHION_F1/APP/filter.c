#include "include.h"
#include "filter.h"
#include "mymath.h"

s32 Moving_Median(s32 moavarray[],u16 len ,u16 *fil_p,s32 in)
{
	u16 width_num;
	u16 now_p;
	float t;
	s8 pn=0;
	u16 start_p,i;
	s32 sum = 0;

	width_num = len ;
	
	if( ++*fil_p >= width_num )	
	{
		*fil_p = 0; //now
	}
	
	now_p = *fil_p ;	
	
	moavarray[ *fil_p ] = in;
	
	if(now_p<width_num-1) //保证比较不越界
	{
		while(moavarray[now_p] > moavarray[now_p + 1])
		{
			t = moavarray[now_p];
			moavarray[now_p] = moavarray[now_p + 1];
			moavarray[now_p + 1] = t;
			pn = 1;
			now_p ++;
			if(now_p == (width_num-1))
			{
				break;
			}
		}
	}
	
	if(now_p>0)  //保证比较不越界
	{
		while(moavarray[now_p] < moavarray[now_p - 1])
		{
			t = moavarray[now_p];
			moavarray[now_p] = moavarray[now_p - 1];
			moavarray[now_p - 1] = t;
			pn = -1;
			now_p--;
			if(now_p == 0)
			{
				break;
			}
		}
	
	}
	
	if(*fil_p == 0 && pn == 1)
	{
		*fil_p = width_num - 1;
	}
	else if(*fil_p == width_num - 1 && pn == -1)
	{
		*fil_p = 0;
	}
	else
	{
		*fil_p -= pn;
	}
	
	start_p = (u16)(0.25f * width_num );
	for(i = 0; i < width_num/2;i++)
	{
		sum += moavarray[start_p + i];
	}
	return (sum/(width_num/2));
}



void simple_3d_trans(_xyz_f_t *ref, _xyz_f_t *in, _xyz_f_t *out) //小范围内正确。
{
	static s8 pn;
	static float h_tmp_x,h_tmp_y;
	
	h_tmp_x = my_sqrt(my_pow(ref->z) + my_pow(ref->y));
	h_tmp_y = my_sqrt(my_pow(ref->z) + my_pow(ref->x));
	
	pn = ref->z < 0? -1 : 1;
	
	  out->x = ( h_tmp_x *in->x - pn *ref->x *in->z ) ;
		out->y = ( pn *h_tmp_y *in->y - ref->y *in->z ) ;
	
	out->z = ref->x *in->x + ref->y *in->y + ref->z *in->z ;

}



///////////////////////////////////////////////////////////////////////////////

// TAU = Filter Time Constant
// T   = Filter Sample Time

// A   = 2 * TAU / T

// Low Pass:
// GX1 = 1 / (1 + A)
// GX2 = 1 / (1 + A)
// GX3 = (1 - A) / (1 + A)

// High Pass:
// GX1 =  A / (1 + A)
// GX2 = -A / (1 + A)
// GX3 = (1 - A) / (1 + A)

///////////////////////////////////////

float ACC_HIGHPASS_TAU        = 4.0;
float ACC_LOWPASS_TAU        = 0.025;
float ACC_LOWPASS_SAMPLE_TIME =0.02f;
float ACC_LOWPASS_A        ;
float ACC_LOWPASS_GX1      ;
float ACC_LOWPASS_GX2      ;
float ACC_LOWPASS_GX3      ;

float BARO_LOWPASS_TAU        = 0.05f;
float BARO_LOWPASS_SAMPLE_TIME =0.02f;
float BARO_LOWPASS_A        ;
float BARO_LOWPASS_GX1      ;
float BARO_LOWPASS_GX2      ;
float BARO_LOWPASS_GX3      ;

float FLOW_LOWPASS_TAU        = 0.005f;
float FLOW_LOWPASS_SAMPLE_TIME =0.02f;
float FLOW_LOWPASS_A        ;
float FLOW_LOWPASS_GX1      ;
float FLOW_LOWPASS_GX2      ;
float FLOW_LOWPASS_GX3      ;
firstOrderFilterData_t firstOrderFilters[NUMBER_OF_FIRST_ORDER_FILTERS];

void initFirstOrderFilter(float T)
{ 
	ACC_LOWPASS_SAMPLE_TIME= T;
	ACC_LOWPASS_A       =    (2.0f * ACC_LOWPASS_TAU / ACC_LOWPASS_SAMPLE_TIME );
	ACC_LOWPASS_GX1    =     (1.0f / (1.0f + ACC_LOWPASS_A));
	ACC_LOWPASS_GX2    =     ACC_LOWPASS_GX1;
	ACC_LOWPASS_GX3     =    ((1.0f - ACC_LOWPASS_A) / (1.0f + ACC_LOWPASS_A));
	
//	ACC_LOWPASS_A       =    (2.0f * ACC_HIGHPASS_TAU / ACC_LOWPASS_SAMPLE_TIME);
//  ACC_LOWPASS_GX1     =    ( ACC_LOWPASS_A / (1.0f + ACC_LOWPASS_A));
//  ACC_LOWPASS_GX2     =   (-ACC_LOWPASS_A / (1.0f + ACC_LOWPASS_A));
//  ACC_LOWPASS_GX3     =    ((1.0f - ACC_LOWPASS_A) / (1.0f + ACC_LOWPASS_A));
  firstOrderFilters[ACC_LOWPASS_X].gx1 = ACC_LOWPASS_GX1;
	firstOrderFilters[ACC_LOWPASS_X].gx2 = ACC_LOWPASS_GX2;
	firstOrderFilters[ACC_LOWPASS_X].gx3 = ACC_LOWPASS_GX3;
	firstOrderFilters[ACC_LOWPASS_Y].gx1 = ACC_LOWPASS_GX1;
	firstOrderFilters[ACC_LOWPASS_Y].gx2 = ACC_LOWPASS_GX2;
	firstOrderFilters[ACC_LOWPASS_Y].gx3 = ACC_LOWPASS_GX3;
	firstOrderFilters[ACC_LOWPASS_Z].gx1 = ACC_LOWPASS_GX1;
	firstOrderFilters[ACC_LOWPASS_Z].gx2 = ACC_LOWPASS_GX2;
	firstOrderFilters[ACC_LOWPASS_Z].gx3 = ACC_LOWPASS_GX3;
	
	BARO_LOWPASS_SAMPLE_TIME= T;
	BARO_LOWPASS_A       =    (2.0f * BARO_LOWPASS_TAU / BARO_LOWPASS_SAMPLE_TIME );
	BARO_LOWPASS_GX1    =     (1.0f / (1.0f + BARO_LOWPASS_A));
	BARO_LOWPASS_GX2    =     BARO_LOWPASS_GX1;
	BARO_LOWPASS_GX3     =    ((1.0f - BARO_LOWPASS_A) / (1.0f + BARO_LOWPASS_A));
	firstOrderFilters[BARO_LOWPASS].gx1 = BARO_LOWPASS_GX1;
	firstOrderFilters[BARO_LOWPASS].gx2 = BARO_LOWPASS_GX2;
	firstOrderFilters[BARO_LOWPASS].gx3 = BARO_LOWPASS_GX3;
	
	FLOW_LOWPASS_SAMPLE_TIME= T;
	FLOW_LOWPASS_A       =    (2.0f * FLOW_LOWPASS_TAU / FLOW_LOWPASS_SAMPLE_TIME );
	FLOW_LOWPASS_GX1    =     (1.0f / (1.0f + FLOW_LOWPASS_A));
	FLOW_LOWPASS_GX2    =     FLOW_LOWPASS_GX1;
	FLOW_LOWPASS_GX3     =    ((1.0f - FLOW_LOWPASS_A) / (1.0f + FLOW_LOWPASS_A));
	firstOrderFilters[FLOW_LOWPASS_X].gx1 = FLOW_LOWPASS_GX1;
	firstOrderFilters[FLOW_LOWPASS_X].gx2 = FLOW_LOWPASS_GX2;
	firstOrderFilters[FLOW_LOWPASS_X].gx3 = FLOW_LOWPASS_GX3;
	firstOrderFilters[FLOW_LOWPASS_Y].gx1 = FLOW_LOWPASS_GX1;
	firstOrderFilters[FLOW_LOWPASS_Y].gx2 = FLOW_LOWPASS_GX2;
	firstOrderFilters[FLOW_LOWPASS_Y].gx3 = FLOW_LOWPASS_GX3;
}


float firstOrderFilter(float input, struct firstOrderFilterData *filterParameters,float T)
{   static u8 init; 
    float output;
    if(!init){init=1;initFirstOrderFilter(T);}
		initFirstOrderFilter(T);
    output = filterParameters->gx1 * input +
             filterParameters->gx2 * filterParameters->previousInput -
             filterParameters->gx3 * filterParameters->previousOutput;

    filterParameters->previousInput  = input;
    filterParameters->previousOutput = output;

    return output;
}
