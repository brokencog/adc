
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
     /* { 132, 156, 180, 222, 234, 246, 0, 0, 0 }, */
     /* { 132, 156, 180, 222, 234, 246, 0, 0, 0 }, */
     /* { 132, 156, 180, 222, 234, 246, 180, 192, 210 }, */
     /* { 135, 159, 184, 221, 236, 244, 267, 290, 313 }, */
     /* { 138, 162, 186, 222, 234, 246, 264, 288, 312 }, */
     /* { 132, 156, 181, 217, 232, 240, 264, 287, 309 }, */
     /* { 133, 157, 182, 218, 233, 241, 264, 287, 309 }, */
     /* { 119, 141, 164, 198, 211, 219, 240, 261, 282 }, */
     /* { 76, 98, 120, 152, 165, 172, 192, 213, 233 }, */
     /* { 67, 89, 112, 145, 159, 166, 187, 208, 229 }, */
     /* { 80, 102, 125, 158, 171, 178, 199, 220, 241 }, */
     /* { 86, 107, 129, 162, 175, 182, 202, 223, 243 }, */
     /* { 89, 112, 135, 168, 182, 189, 211, 232, 253 }, */
     /* { 84, 102, 126, 156, 168, 174, 198, 216, 234 }, */
     /* { 72, 96, 114, 144, 156, 162, 180, 204, 222 }, */
     /* { 60, 78, 102, 126, 138, 144, 162, 180, 198 }, */
     /* { 42, 60, 78, 102, 114, 120, 138, 156, 174 }, */
     /* { 18, 36, 54, 78, 84, 90, 108, 120, 138 } */
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
	  if( key > array[mid] )
	       first = mid + 1;	
	  else if( (key >= array[mid]) && (key < array[mid+1]) )
	       return mid;
	  else if( key == array[mid] )
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
	  return stepsPerRevolution;
     else
	  return linear( enginemap,
			 (float)stepper_motor_values[psi_index][map_index], (float)map_array[map_index],
			 (float)stepper_motor_values[psi_index][map_index + 1], (float)map_array[map_index+1] );


}


void read_config( char *configSrc)
{
     FILE *configFile = fopen( configSrc, "r" );
     char buffer[256], *delim, *temp;
     int mapi = 0, psii = 0;

     // read configuration file.
     do {
	  fgets( buffer, 256, configFile );

	  // handle simple cases: leading comment, and { starting array
	  if( buffer[0] == '#' || buffer[0] == '\n' ) {

	  } else if( buffer[0] == '{' ) {

	       // read header line of array, which is map array.
	       fgets( buffer, 256, configFile );
	       temp = buffer;

	       // first element is 0 place holder.
	       delim = strtok( temp, ", " ); // place holder
	       delim = strtok( NULL, ", " ); // first value.
	       delim = strtok( NULL, ", " ); // first value.
	       do {
		    map_array[mapi++] = atoi( delim );
	       	    delim = strtok( NULL, ", " );
	       } while( delim );

	       // next is the stepper arrays.	       
	       do {
		    mapi = 0;

		    fgets( buffer, 256, configFile );
		    temp = buffer;

		    delim = strtok( temp, ", " ); // first value is PSI array element.
		    delim = strtok( NULL, ", " );  // first stepper element
		    psi_array[psii] = atoi( delim );
		    delim = strtok( NULL, ", " );  // first stepper element
		    do {
			 stepper_motor_values[psii][mapi++] = atoi( delim );
			 //printf( "%s: %d.\n", delim, stepper_motor_values[psii][mapi-1] );
			 delim = strtok( NULL, ", " );
		    } while( delim );
	       
	       } while( psii++ < PSI_STEPS );

	  } else {
	       /* temp = buffer; */
	       /* delim = strchr( temp++, ',' ); */
	       /* delim[0] = '\0'; */
	       /* printf( "Name: %s, value: %f.\n", buffer, atof( &delim[1] ) ); */
	  }
	       
     } while( !feof(configFile) );

     
}



int main( int argc, char *argv[] )
{
     float xin, yin;
     int tablevalue, steps, previous, i=1;

     if( argc < 4 ) {
	  printtable( psi_array, map_array, stepper_motor_values, PSI_STEPS, MAP_STEPS );
	  printf( "usage: %s MAP PSI Prev_Stepper\n", argv[0] );
	  exit( 0 );
     } else if( argc > 4 ) {
	  read_config( argv[1] );
	  i = 2;	  
     }
     
     xin = atof( argv[i++] );
     yin = atof( argv[i] );
     previous = atol( argv[3] );

     tablevalue = calc_stepper_value( xin, yin );
     steps = trunc( previous - ( stepsPerRevolution - tablevalue ) );

     printf( "Table value: %d.  myStepper.step(%d).\n", tablevalue, steps );

     return steps;

}
