#include "include.h"
#include "filter.h"
#include "mymath.h"

#define MED_WIDTH_NUM 50
#define MED_FIL_ITEM  30

float med_filter_tmp[MED_FIL_ITEM][MED_WIDTH_NUM ];
float med_filter_out[MED_FIL_ITEM];

u8 med_fil_cnt[MED_FIL_ITEM];
// 1  2  3                                9
float Moving_Median(u8 item,u8 width_num,float in)
{
	u8 i,j;
	float t;
	float tmp[MED_WIDTH_NUM];
	
	if(item >= MED_FIL_ITEM || width_num >= MED_WIDTH_NUM )
	{
		return 0;
	}
	else
	{
		if( ++med_fil_cnt[item] >= width_num )	
		{
			med_fil_cnt[item] = 0;
		}
		
		med_filter_tmp[item][ med_fil_cnt[item] ] = in;
		
		for(i=0;i<width_num;i++)
		{
			tmp[i] = med_filter_tmp[item][i];
		}
		
		for(i=0;i<width_num-1;i++)
		{
			for(j=0;j<(width_num-1-i);j++)
			{
				if(tmp[j] > tmp[j+1])
				{
					t = tmp[j];
					tmp[j] = tmp[j+1];
					tmp[j+1] = t;
				}
			}
		}

		
		return ( tmp[(u16)width_num/2] );
	}
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


u16 Distance[50]  = {0};
u8 measure_num=8;
float sonar_filter_oldx(float in) 
{
   
    u8 IS_success =1;
    u16 Distance1 = 0;
    u16 MAX_error1 = 0;
    u8 MAX_error_targe = 0;
    u8 count = 0;
    u8 i =0;
    u8 j =0;
    u8 num =0;  //Êµ¼ÊÖ»²âµ½µÄ´ÎÊý
    Distance[measure_num-1]=in*1000;
    for(i=0;i<measure_num-1;i++)
		{
		 Distance[i]=Distance[i+1]; 
		}

   
         //ÅÅÐò
        for(i = 0 ; i < measure_num-1 ; i++)
        {

            for(j = 0 ; j < measure_num-1-i; j++)       
            {
                if(Distance[j] > Distance[j+1] )
                {
                    Distance1 = Distance[j];
                    Distance[j] =  Distance[j+1];
                    Distance[j+1] = Distance1; 
                }
            }

        }

        //ÕÒ³ö×î´ó²î¾à
        MAX_error1 = Distance[1] - Distance[0];
        for(i = 1 ; i < measure_num-1 ; i++)
        {

            if(MAX_error1 < Distance[i+1] - Distance[i] )//Èç£º1 2 3 4 5    8 9 10    MAX_error_targe=4;
            {
                MAX_error1 =  Distance[i+1] - Distance[i];//×î´ó²î¾à
                MAX_error_targe = i;  //¼ÇÏÂ×î´ó²î¾àÖµµÄÎ»ÖÃ£¨Õâ×éÊýÖÐµÄÎ»ÖÃ£©
            }

        }
        float UltrasonicWave_Distance1=0;
        //È¡³ö×îÖÕÖµ
        if(MAX_error_targe+1 > (measure_num+1)/2) //Ç°²¿·ÖÓÐÐ§  1 2 3 4 5    8 9 10  (Èç¹ûÎ»ÓÚÖÐ¼ä£¬ºó°ë²¿ÓÅÏÈ)
        {
            for(i = 0 ; i <= MAX_error_targe ; i++)
            {
                UltrasonicWave_Distance1 += Distance[i];
            }

            UltrasonicWave_Distance1 /= (MAX_error_targe+1);//È¡Æ½¾ù

        }
        else  //ºó²¿·ÖÓÐÐ§  1 2 3   7 8 9 10
        {
             for(i = MAX_error_targe + 1 ; i < measure_num ; i++)
            {
                UltrasonicWave_Distance1 += Distance[i];
            }

            UltrasonicWave_Distance1 /= (measure_num - MAX_error_targe -1);//È¡Æ½¾ù

        }


    return  (float)UltrasonicWave_Distance1/1000.; //×ª»¯ÎªÃ×Îªµ¥Î»µÄ¸¡µãÊý
}



static float sonar_values[3] = { 0.0f };
static unsigned insert_index = 0;

static void sonar_bubble_sort(float sonar_values[], unsigned n);

void sonar_bubble_sort(float sonar_values[], unsigned n)
{
	float t;

	for (unsigned i = 0; i < (n - 1); i++) {
		for (unsigned j = 0; j < (n - i - 1); j++) {
			if (sonar_values[j] > sonar_values[j+1]) {
				/* swap two values */
				t = sonar_values[j];
				sonar_values[j] = sonar_values[j + 1];
				sonar_values[j + 1] = t;
			}
		}
	}
}

float insert_sonar_value_and_get_mode_value(float insert)
{
	const unsigned sonar_count = sizeof(sonar_values) / sizeof(sonar_values[0]);

	sonar_values[insert_index] = insert;
	insert_index++;
	if (insert_index == sonar_count) {
		insert_index = 0;
	}

	/* sort and return mode */

	/* copy ring buffer */
	float sonar_temp[sonar_count];
	memcpy(sonar_temp, sonar_values, sizeof(sonar_values));

	sonar_bubble_sort(sonar_temp, sonar_count);

	/* the center element represents the mode after sorting */
	return sonar_temp[sonar_count / 2];
}
