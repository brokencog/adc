
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define stepsPerRevolution 300

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

     printf( "\n\t\t" );
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




int bsearch_array( int key, const int array[], int length )
{
     int first = 0,
	  last = length - 1,
	  mid = (first + last) / 2;

     if( (key < array[first]) || (key > array[last]) )
	  return -1;

     while( first <= last ) {
	  if( key >= array[mid] )
	       first = mid + 1;
	  else if( (key >= array[mid]) && (key < array[mid+1]) )
	       return mid;
	  else
	       last = mid - 1;

	  mid = (first + last) / 2;

     }

     return mid;

}




void read_config( char *configSrc)
{
     FILE *configFile = fopen( configSrc, "r" );
     char buffer[256], *delim, *temp;
     int mapi = 0, psii = 0;

     printf( "Reading from config file: %s.\n", configSrc );

     // read configuration file.
     while( !feof(configFile) ) {

	  fgets( buffer, sizeof(buffer), configFile );

	  if( buffer[0] == '{' ) {

	       // read header line of array, which is map array.
	       fgets( buffer, sizeof(buffer), configFile );
	       
	       temp = buffer; // is this needed for strtok()??

	       printf( "%5s", "" );
	       for( mapi=0; mapi < MAP_STEPS; mapi++ )
		    printf( "%5d", mapi );
	       printf( "\n" );

	       // first element is 0 place holder.
	       delim = strtok( temp, ", " ); // place holder
	       delim = strtok( NULL, ", " );
	       delim = strtok( NULL, ", " );
	       mapi = 0;
	       printf( "\t" );
	       do {
		    map_array[mapi++] = atoi( delim );
		    printf( "%3d", map_array[mapi-1] );
		    printf( ", ");
		    delim = strtok( NULL, ", " );
	       } while( delim && (mapi < MAP_STEPS) );
	       printf( "\n" );

	       // next is the stepper arrays.
	       do {
		    mapi = 0;

		    fgets( buffer, sizeof(buffer), configFile );
		    temp = buffer;

		    delim = strtok( temp, ", " ); // first value is PSI array element.
		    delim = strtok( NULL, ", " );
		    psi_array[psii] = atoi( delim );
		    printf( "%2d|%3d", psii, psi_array[psii] );
		    printf( ":\t");
		    delim = strtok( NULL, ", " );
		    do {
			 stepper_motor_values[psii][mapi++] = atoi( delim );
			 printf( "%3d", stepper_motor_values[psii][mapi-1] );
			 printf( ", ");
			 delim = strtok( NULL, ", " );
		    } while( delim && (mapi < MAP_STEPS) );
		    printf( "\n" );
		    
	       } while( psii++ < (PSI_STEPS-1) );
//	       printf( ".++++++ end of config file read ++++." );

	  }

     }
     
}


float bi_linear(float q11, float q12, float q21, float q22,
		       float x1, float y1, float x2, float y2,
		       float x, float y) 
{
     float x2x1, y2y1, x2x, y2y, yy1, xx1;

     x2x1 = x2 - x1;
     y2y1 = y2 - y1;
     x2x = x2 - x;
     y2y = y2 - y;
     yy1 = y - y1;
     xx1 = x - x1;

     return 1.0 / (x2x1 * y2y1) * (
	  q11 * x2x * y2y +
	  q21 * xx1 * y2y +
	  q12 * x2x * yy1 +
	  q22 * xx1 * yy1 );

}





// map/step == x/y
float linear( float i, float x0, float  y0, float x1, float y1 )
{
     float slope = (x1-x0) / (y1-y0);
     float new_step =  ((x1 - x0) * (i - x1)) + x1;

     new_step = y0 + ( slope * (i-x0) );
     
     // printf( "slope: %f, (x0,y0)=(%.2f,%.2f), (%.2f,y) == %.2f.\n", slope, x0, y0, i, new_step );
     return new_step;

}


float calc_interpolation( float tankpsi, float enginemap )
{
     int map_index, psi_index, stepvalue;
     float x, y, delta;

     psi_index = bsearch_array( round(tankpsi), psi_array, PSI_STEPS );
     map_index = bsearch_array( round(enginemap), map_array, MAP_STEPS );

     printf( "PSI index: %d, MAP index: %d. Table value: %d\n", psi_index, map_index,
     	     stepper_motor_values[psi_index][map_index] );

     if( psi_index == -1 || map_index == -1 ||
	  stepper_motor_values[psi_index][map_index] == 0 ||
	  stepper_motor_values[psi_index][map_index+1] == 0 )
	  return stepsPerRevolution;

     else {
	  return bi_linear( 
	       stepper_motor_values[psi_index][map_index],  // q11
	       stepper_motor_values[psi_index][map_index+1],
	       stepper_motor_values[psi_index+1][map_index],
	       stepper_motor_values[psi_index+1][map_index+1],

	       psi_array[psi_index],  // x1
	       map_array[map_index],  // y1,
	       psi_array[psi_index+1],
	       map_array[map_index+1],

	       tankpsi, enginemap ); // x, y

	  /* return linear( enginemap, */
	  /* 		 map_array[map_index], */
	  /* 		 stepper_motor_values[psi_index][map_index], */
	  /* 		 map_array[map_index+1], */
	  /* 		 stepper_motor_values[psi_index][map_index+1] ); */
	  
     }

}


int main( int argc, char *argv[] )
{
     float mapin, psiin, interpolatedvalue;
     int steps, previous, i=1;
     char buffer[10];

     if( (argc == 1) || (argc > 6) ) {
	  printtable( psi_array, map_array, stepper_motor_values, PSI_STEPS, MAP_STEPS );
	  printf( "usage: %s [-c <config file>] PSI MAP Prev_Stepper\n", argv[0] );
	  exit( 0 );

     } else if( (argc > 2) && (argv[1][0] == '-') && (argv[1][1] == 'c') ) {
	  read_config( argv[2] );
	  i = 3;
	  if( argc < 6 )
	       exit(0);

     } else
	  i = 1;

	  
     psiin = atof( argv[i++] );
     mapin = atof( argv[i++] );
     previous = atol( argv[i++] );

     do {

	  interpolatedvalue = calc_interpolation( psiin, mapin );
	  steps = -1 * ( previous - interpolatedvalue );
	  
	  printf( "Previous Stepper: %d.  Interpolated result: %.2f.  Steps to move: %d.\n",
		  previous, interpolatedvalue, steps );

	  previous = interpolatedvalue;

	  printf( "Enter PSI: " );
	  fgets( buffer, 10, stdin );  
	  psiin = atof( buffer );
	  printf( "Enter MAP: " );
	  fgets( buffer, 10, stdin );  
	  mapin = atof( buffer );
	  
     } while( mapin && psiin );

     return previous;
     
}
