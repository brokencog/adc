
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define stepsPerRevolution 300

int psi_array[] = { 30, 40, 50, 60, 70, 80, 100, 200, 250, 300, 350, 400, 450, 500, 550, 600, 650, 700 };
#define PSI_STEPS ( sizeof(psi_array) / sizeof(int) )

int map_array[] = { 110, 115, 120, 125, 130, 135, 140, 145, 150 };
#define MAP_STEPS ( sizeof(map_array) / sizeof(int) )

int stepper_motor_values[PSI_STEPS][MAP_STEPS];




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
	  else
	       last = mid - 1;

	  mid = (first + last) / 2;

     }

     return mid;

}

float linear( float i, float x0, float  y0, float x1, float y1 )
{
     float ratio;
     
     // x0 + i * ( (i-y0)/(x1-x0) )
     return x0 + ( i * ((i - y0)/(x1-y0)) );
}


int calc_stepper_value( float tankpsi, float enginemap )
{
     int map_index, psi_index, stepvalue;
     float x, y, delta;

     psi_index = bsearch_array( round(tankpsi), psi_array, PSI_STEPS );
     map_index = bsearch_array( round(enginemap), map_array, MAP_STEPS );

     printf( "Found map: %d, and map+1: %d.\n", map_array[map_index], map_array[map_index+1] );
     printf( "Stepper value: %d, +1: %d.\n", stepper_motor_values[psi_index][map_index], stepper_motor_values[psi_index][map_index+1] );
     

     if( psi_index == -1 || map_index == -1 )
	  return stepsPerRevolution;

     else {
	  return linear( enginemap,
			 stepper_motor_values[psi_index][map_index],
			 map_array[map_index],
			 stepper_motor_values[psi_index][map_index+1],
			 map_array[map_index+1] );
	  
     }

}


void read_config( char *configSrc)
{
     FILE *configFile = fopen( configSrc, "r" );
     char buffer[256], *delim, *temp;
     int mapi = 0, psii = 0;

     // read configuration file.
     while( !feof(configFile) ) {

	  fgets( buffer, sizeof(buffer), configFile );
//	  printf( "%s", buffer );

	  if( buffer[0] == '{' ) {

	       // read header line of array, which is map array.
	       fgets( buffer, sizeof(buffer), configFile );
	       
	       temp = buffer; // is this needed for strtok()??

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
		    printf( "%3d", psi_array[psii] );
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
     yin = atof( argv[i++] );
     previous = atol( argv[i++] );

     tablevalue = calc_stepper_value( xin, yin );
     printf( "Table value: %d. Previous: %d,  myStepper.step(%d).\n", tablevalue, previous, -1*(previous - tablevalue) );

     return steps;

}
