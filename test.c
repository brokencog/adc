
#include <stdio.h>
#include <stdlib.h>
#include <math.h>


int psi_array[] = { 30, 40, 50, 60, 70, 80, 100, 200, 250, 300, 350, 400, 450, 500, 550, 600, 650, 700 };
#define PSI_STEPS ( sizeof(psi_array) / sizeof(int) )

int map_array[] = { 110, 115, 120, 125, 130, 135, 140, 145, 150 };
#define MAP_STEPS ( sizeof(map_array) / sizeof(int) )

int stepper_motor_values[PSI_STEPS][MAP_STEPS] = {
     { 132, 156, 180, 222, 234, 246, 0, 0, 0 },
     { 132, 156, 180, 222, 234, 246, 0, 0, 0 },
     { 132, 156, 180, 222, 234, 246, 180, 192, 210 },
     { 135, 159, 184, 221, 236, 244, 267, 290, 313 },
     { 138, 162, 186, 222, 234, 246, 264, 288, 312 },
     { 132, 156, 181, 217, 232, 240, 264, 287, 309 },
     { 133, 157, 182, 218, 233, 241, 264, 287, 309 },
     { 119, 141, 164, 198, 211, 219, 240, 261, 282 },
     { 76, 98, 120, 152, 165, 172, 192, 213, 233 },
     { 67, 89, 112, 145, 159, 166, 187, 208, 229 },
     { 80, 102, 125, 158, 171, 178, 199, 220, 241 },
     { 86, 107, 129, 162, 175, 182, 202, 223, 243 },
     { 89, 112, 135, 168, 182, 189, 211, 232, 253 },
     { 84, 102, 126, 156, 168, 174, 198, 216, 234 },
     { 72, 96, 114, 144, 156, 162, 180, 204, 222 },
     { 60, 78, 102, 126, 138, 144, 162, 180, 198 },
     { 42, 60, 78, 102, 114, 120, 138, 156, 174 },
     { 18, 36, 54, 78, 84, 90, 108, 120, 138 }
};


void printtable( int index[], int header[], int array[PSI_STEPS][MAP_STEPS], int height, int width )
{
     int count=15, i, j;

     printf( "\t\t" );
     for( i=0; i<width; i++ ) {
	  printf( "[%d]%d\t", i, header[i] );
	  count += 8;
     }
     printf( "\n" );

     for( i=0; i<count; i++ )
	  printf( "-" );
     printf( "\n" );

     for( i=0; i<height; i++ ) {
	  printf( "[%d]\t%d:\t", i, index[i] );

	  for( j=0; j<width; j++ ) 
	       printf( "%d\t", array[i][j] );
	  printf( "\n" );

     }

     printf( "\n" );

}


int bsearch_array( int key, int array[], int length )
{
     int first = 0,
	  last = length - 1,
	  mid = (first + last) / 2;

     if( (key < array[first]) || (key > array[last]) )
	  return -1;
     
     while( first <= last ) {
	  if( key > array[mid] )
	       first = mid + 1;
	  else if( (key >= array[mid]) && (key < array[mid+1]) )
	       return mid;
	  else
	       last = mid - 1;

	  mid = (first + last) / 2;

     }

     return mid;

}

float linear( float i, float x0, float  y0, float x1, float y1 )
{
     float ratio = (i - y0) / (x1 - x0);
     
     return x0 + i*ratio ;

}


int calc_stepper_value( float tankpsi, float enginemap )
{
     int map_index, psi_index, stepvalue;
     float x, y, delta;

     psi_index = bsearch_array( round(tankpsi), psi_array, PSI_STEPS );
     map_index = bsearch_array( round(enginemap), map_array, MAP_STEPS );

     if( psi_index == -1 || map_index == -1 )
	  return 0;
     else
	  return linear( enginemap,
			 (float)stepper_motor_values[psi_index][map_index], (float)map_array[map_index],
			 (float)stepper_motor_values[psi_index][map_index + 1], (float)map_array[map_index+1] );


}


int main( int argc, char *argv[] )
{
     float xin, yin;

     printf( "\n");
     printtable( psi_array, map_array, stepper_motor_values, PSI_STEPS, MAP_STEPS );
     
     xin = atof( argv[1] );
     yin = atof( argv[2] );

     printf( "Stepper: %d.\n", calc_stepper_value( xin, yin ) );

     return 0;
}
