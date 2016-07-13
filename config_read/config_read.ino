#include <SdFat.h>

SdFat SD;
SdFile configFile;

float *adc_values;
int *psi_array,  *map_array, **stepper_motor_values;

const int PSI_STEPS=9;

void setup()
{
     Serial.begin(9600);
     SD.begin( 8 );
     configFile.open( "config" );
     
     // allocate arrays.  We'll never de-allocate them ...
     adc_values = (float*)malloc( sizeof(float) * 9 );
     psi_array = (int*)malloc( sizeof(int) * 9 );
     map_array = (int*)malloc( sizeof(int) * 18 );
     stepper_motor_values = (int**)malloc( sizeof(int) * (18*9) );


     read_config();
     
}

void read_config()
{
     char buffer[256], *delim, *temp;
     int mapi = 0, psii = 0, nRead = 0, i, j;

     Serial.println( "\n\n");


     // read configuration file.
     while( (nRead=configFile.fgets(buffer, sizeof(buffer))) > 0 ) {

     	  if( buffer[0] == '{' ) {

     	       // read header line of array, which is map array.
     	       nRead = configFile.fgets( buffer, sizeof(buffer) );
     	       temp = buffer;

     	       // first element is 0 place holder.
     	       delim = strtok( temp, ", };" ); // place holder
	       delim = strtok( NULL, ", };" );
	       mapi = 0;
     	       while( delim && (delim[0] != '\n') ) {
     		    map_array[mapi++] = atoi( delim );
     	       	    delim = strtok( NULL, ", };" );
     		    Serial.print( map_array[mapi-1] );
     		    Serial.print( ", " );
     	       }
	       Serial.print( "\n\n" );


     	       // next is the stepper arrays.
	       Serial.println( "Reading through stepper array." );

     	       do {
     		    mapi = 0;

		    nRead = configFile.fgets( buffer, sizeof(buffer) );
		    temp = buffer;

		    delim = strtok( temp, ", };" );
		    delim = strtok( NULL, ", };" );
     		    psi_array[psii] = atoi( delim );

     		    Serial.print( psi_array[psii] );
		    Serial.print( ": ");
		    
     		    do {
     			 stepper_motor_values[psii][mapi++] = atoi( delim );
     			 delim = strtok( NULL, ",}; " );
			 Serial.print( stepper_motor_values[psii][mapi-1] );
     			 Serial.print(", ");
			 Serial.print( "[" );
			 Serial.print( delim );
			 Serial.print( "]");
			 
     		    } while( delim && (delim[0] != '\n') );
     		    Serial.print("\n");

     	       } while( psii++ < PSI_STEPS && delim );
     	       Serial.println( "\n========" );

     	  }

     }
     
}


void loop() {

     
}
