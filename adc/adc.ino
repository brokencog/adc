/*
  Project: ADC recorder
  Started: 2 July 2016
*/

#include <Stepper.h>
#include <Time.h>
#include <Math.h>
#include <SPI.h>
#include <SdFat.h>


/*
   SD Card Shield overview.

   SeeedStudo SD Shield:
   http://www.seeedstudio.com/wiki/SD_Card_shield_V4.0
   D4: SD_CS;
   D11: SD_DI;
   D12: SD_DO;
   D13: SD_CLK.

   OSEPP SD Shield:
   http://osepp.com/products/shield-arduino-compatible/micro-sd-shield/
   SPI (SCK on D13, MISO on D12, MOSI on D11, and CS on D8)

   Arduino Mega
   There may an issue with using pin 10 on the Mega.  If so, the SD CS must be moved to pin 53.

   On the Mega, the SPI lines used are different than OSEPP expects for the Uno.
   Bend the following pins, to enable CS on 53:
   8 to 53 MEGA
   11 to 51 MEGA
   12 to 50 MEGA
   13 to 52 MEGA


   Each time the device is powered on or reset, a new data file will be opened for logging data.
   The file name is in the format data_xx.txt
   SD Card may be safely removed during operation with one caveat.  The file currently in use will be truncated
   and new data written.  Thus, to ensure preserving previous data, press the reset button or power off prior
   to removing the SD card.

   -----> SD_CS_PIN must be set to the appropriate CS pin.
   As per the data sheets above, the OSEPP expects CS on pin 8 and the Seeed on pin 4.
   The Mega _might_ have an issue with those pins, in which case use 53.

   For verification, the LED on the Arduino goes HIGH during all SD card operations.  It should flash (dimly).


   //
   // calculate stepper motor position based on MAP and tank pressure
           |  110  115  120 125 130 135 140 145 150
       -----------------------------------------------------------------------------
       30  |  132  156  180 222 234 246 0 0 0    
       40  |  132  156  180 222 234 246 0 0 0
       50  |  132  156  180 222 234 246 180 192 210
       60  |  135  159  184 221 236 244 267 290 313
       70  |  138  162  186 222 234 246 264 288 312
       80  |  132  156  181 217 232 240 264 287 309
       100 |  133  157  182 218 233 241 264 287 309
       200 |  119  141  164 198 211 219 240 261 282
       250 |  76   98   120 152 165 172 192 213 233
       300 |  67   89   112 145 159 166 187 208 229
       350 |  80   102  125 158 171 178 199 220 241
       400 |  86   107  129 162 175 182 202 223 243
       450 |  89   112  135 168 182 189 211 232 253
       500 |  84   102  126 156 168 174 198 216 234
       550 |  72   96   114 144 156 162 180 204 222
       600 |  60   78   102 126 138 144 162 180 198
       650 |  42   60   78  102 114 120 138 156 174
       700 |  18   36   54  78  84  90  108 120 138

*/

#define SD_CS_PIN 53

// delay between each iteration, in milliseconds.
#define CONTROLER_PAUSE_TIME 0



//
// analog input pin/value pairs.
//
#define map_pin A5
#define dieselflow_in_pin A7
#define dieselflow_out_pin A2
#define ngflowmeter_pin A3
#define psi_pin A4
#define exhaustgastemp_pin A8
#define tanktemp_pin A6

#define ledPin 13
#define solenoidPin 31


//
// following are indices into the various stepper motor arrays.
// They were based on the pin arrangement above, but no longer.
#define map 0
#define dieselflow_in 1
#define dieselflow_out 2
#define ngflowmeter 3
#define psi 4
#define exhaustgastemp 5
#define tanktemp 6
#define diesel_flow 7
#define solenoid_position 8
#define previous_stepper 9
#define stepper_value 10
#define ANALOG_VALUES 11



//
// SeeedStudio Stepper motor shield for Mega
#define enA  3  // Enable pin 1 on Motor Control Shield  
#define enB  11  // Enable pin 2 on Motor Control Shield  
#define dirA 12  // Direction pin dirA on Motor Control Shield
#define dirB 13  // Direction pin dirB on Motor Control Shield




// 
// Initialize the stepper library on pins 12 and 13:
const int stepsPerRevolution = 300;  // Change this to fit the number of steps per revolution for your motor
Stepper myStepper( stepsPerRevolution, dirA, dirB );
 

//
// tables for calculating stepper motor movement
float adc_values[ANALOG_VALUES];

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

const char *adc_labels[ANALOG_VALUES] = { "MAP", "IN", "OUT", "NG", "PSI", "EGT", "TT", "Flow", "Solenoid", "Previous", "Stepper" };



//
// globals
char g_str_buffer[125];
char g_datafile[15];

SdFat SD;





//
// read config file from SD card.
// very limited formatting.
int read_config( SdFile configFile )
{
     char buffer[256], *delim, *temp;
     int mapi = 0, psii = 0;

     Serial.println( " ... reading.");

     // read configuration file.
     while( configFile.available() ) {

	  configFile.fgets( buffer, sizeof(buffer) );

	  if( buffer[0] == '{' ) {

	       // read header line of array, which is map array.
	       configFile.fgets( buffer, sizeof(buffer) );
	       
	       temp = buffer; // is this needed for strtok()??

	       // first element is 0 place holder.
	       delim = strtok( temp, ", " ); // place holder
	       delim = strtok( NULL, ", " );
	       delim = strtok( NULL, ", " );
	       mapi = 0;
	       do {
		    map_array[mapi++] = atoi( delim );
		    Serial.print( map_array[mapi-1] );
		    Serial.print( ", ");
		    delim = strtok( NULL, ", " );
	       } while( delim && (mapi < MAP_STEPS) );
	       Serial.println( "\n.--------------.");


	       // next is the stepper arrays.
	       do {
		    mapi = 0;

		    configFile.fgets( buffer, sizeof(buffer) );
		    temp = buffer;

		    delim = strtok( temp, ", " ); // first value is PSI array element.
		    delim = strtok( NULL, ", " );
		    psi_array[psii] = atoi( delim );
		    Serial.print( psi_array[psii] );
		    Serial.print( ":\t");
		    delim = strtok( NULL, ", " );
		    do {
			 stepper_motor_values[psii][mapi++] = atoi( delim );
			 Serial.print( stepper_motor_values[psii][mapi-1] );
			 Serial.print( ", ");
			 delim = strtok( NULL, ", " );
		    } while( delim && (mapi < MAP_STEPS) );
		    Serial.print( "\n" );
		    
	       } while( psii++ < (PSI_STEPS-1) );
	       Serial.println( ".++++++ end of config file read ++++." );

	  }

     }
     
}



//
//
void setup()
{
     SdFile dataFile;
     
     int i;


     // Serial output
     Serial.begin( 9600 );
     sprintf( g_str_buffer, "\n\nADC Setup().  Compiled at: %s %s.\nSD CS pin: %d.\n", __DATE__, __TIME__, SD_CS_PIN );
     Serial.println( g_str_buffer );
  

     //
     // LED shall indicate SD card activity
     pinMode( ledPin, OUTPUT );

     //
     // SD Fat initialization
     if( !SD.begin( SD_CS_PIN ) )
	  Serial.println( "Unable to initialize SD Card." );

     else{
	  // read configuration file if present.
	  Serial.print( "Opening config file ...." );
	  dataFile.open("config", O_READ );
	  if( dataFile.isOpen() )
	       read_config( dataFile );
	  else
	       Serial.println( "Unable to open config file." );

	  dataFile.close();
	  
     }


     // generate name for new data file.
     i = 0;
     do {
	  sprintf( g_datafile, "data_%d.txt", i++ );
     } while( SD.exists(g_datafile) && (i<256) );
     if( i >= 256 ) {
	  Serial.print( "Unable to open data file." );
     }


     //
     // ADC initialization
     // https://www.arduino.cc/en/Tutorial/DigitalPins
     pinMode( map_pin, INPUT );
     pinMode( dieselflow_in_pin, INPUT );
     pinMode( dieselflow_out_pin, INPUT );
     pinMode( psi_pin, INPUT );
     pinMode( exhaustgastemp_pin, INPUT );
     pinMode( tanktemp_pin, INPUT );


     // must switch to low for NG to flow -- when not using NG, set to HIGH
     // this is for the MOSFET circuit.
     pinMode( solenoidPin, OUTPUT );
     digitalWrite( solenoidPin, HIGH );

     
     // Motor Initialization
     // set the speed in rpm:
     myStepper.setSpeed( 60 );

     pinMode( enA, OUTPUT );
     digitalWrite( enA, HIGH );
     pinMode( enB, OUTPUT );
     digitalWrite( enB, HIGH );


     // initially close both the solenoid and NG valve.
     for( int i=0; i<stepsPerRevolution; i++ )
	  myStepper.step( -10 );
     
     adc_values[solenoid_position] = (float)HIGH;   // because we store in the ADC array ... convenience.
     digitalWrite( solenoidPin, (int)adc_values[solenoid_position] );

     adc_values[previous_stepper] = stepsPerRevolution;
     adc_values[stepper_value] = stepsPerRevolution;

    
     //
     // print header, and store in data file.

     digitalWrite( ledPin, HIGH );
     dataFile.open( g_datafile, O_RDWR|O_AT_END|O_CREAT );
     if( ! dataFile.isOpen() )
	  Serial.println( "unable to open data file in Setup.");

     Serial.println( "Motor 120rpm, 300 steps stop to stop.  Solenoid off, NG valve shut.\n" );

     Serial.print( "\nTime:\t" );
     dataFile.print( "\r\nTime:\t" );
     for( i=0; i<ANALOG_VALUES; i++ ) {
	  sprintf( g_str_buffer, "%s\t", adc_labels[i] );
	  dataFile.print( g_str_buffer );
	  Serial.print( g_str_buffer );
     }
     Serial.print( "\n" );
     dataFile.print( "\r\n" );
     dataFile.close();

     digitalWrite( ledPin, LOW );

}



//
//
void write_data( float adcvals[ANALOG_VALUES] )
{
     SdFile dataFile;
     int i;
     char buff[15];
     
     digitalWrite( ledPin, HIGH );
     dataFile.open( g_datafile, O_RDWR|O_AT_END|O_CREAT );

     // date time stamp
     sprintf( g_str_buffer, "%d:%d:%d\t", hour(), minute(), second() );
     Serial.print( g_str_buffer );
     dataFile.print( g_str_buffer );

     // for sequence of values, refer to index above ANALOG_VAlUES
     // nicer would be to have a matching array of value names.
     for( i=0; i < ANALOG_VALUES; i++ ) {
	  dtostrf( adcvals[i], 4, 3, buff );   // sadly, Arduino doesn't print floats natively
	  sprintf( g_str_buffer, "%s, ", buff );
	  Serial.print( g_str_buffer );
	  dataFile.print( g_str_buffer );
     }
     sprintf( g_str_buffer, "\r\n" );
     Serial.print( g_str_buffer );
     dataFile.print( g_str_buffer );

     dataFile.close();

     digitalWrite( ledPin, LOW );

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


void loop()
{

     //
     // Arduino ADC has 1024 steps.  Volts = Ain * (MaxVolts / 1023)
     // read analog inputs (twice to stabilize ADC), convert to volts, apply calculation from spreadsheet.
     analogRead( map_pin );
     adc_values[map] = (39.958 * (analogRead(map_pin) * (5.0/1023)) ) + 8.142;
     analogRead( dieselflow_in_pin );
     adc_values[dieselflow_in] = (1.3442 * (analogRead(dieselflow_in_pin) * (5.0/1023.0)) ) - 0.0168;
     analogRead( dieselflow_out_pin );
     adc_values[dieselflow_out] = (0.5767 * (analogRead(dieselflow_out_pin) * (5.0/1023.0)) ) - .0102;
     analogRead( ngflowmeter_pin );
     adc_values[ngflowmeter] = ((40.276 * (analogRead(ngflowmeter_pin) * (5.0/1023.0)) ) - .2208) * .7175/60;
     analogRead( psi_pin );
     adc_values[psi] = (268.68 * (analogRead(psi_pin) * (5.0/1023.0)) ) - 115.62;
     analogRead( exhaustgastemp_pin );
     adc_values[exhaustgastemp] = (249.21 * (analogRead(exhaustgastemp_pin) * (5.0/1023.0)) ) - 2.5189;
     analogRead( tanktemp_pin );
     adc_values[tanktemp] = (197.9 * (analogRead(tanktemp_pin) * (5.0/1023.0)) ) - 251.2;


     // calculate flow based on IN - OUT
     adc_values[diesel_flow] = adc_values[dieselflow_in] - adc_values[dieselflow_out];
     
     // determine stepper motor position to move to
     // open is negative, hence the inversion
     adc_values[stepper_value] = -1.0 * ( adc_values[previous_stepper] - calc_interpolation( adc_values[psi], adc_values[map] ) );

     myStepper.step( (int)adc_values[stepper_value] );

     // if the position for stepper motor is valid, open the solenoid
     if( (adc_values[stepper_value] > 0) && (adc_values[stepper_value] < 300) ) {
	  adc_values[solenoid_position] = LOW;
     } else
	  adc_values[solenoid_position] = HIGH;
     digitalWrite( solenoidPin, (int)adc_values[solenoid_position] );

     // save for next iteration's comparison.
     adc_values[previous_stepper] = adc_values[stepper_value];

     // save data to SD and display.
     write_data( adc_values );

     delay( CONTROLER_PAUSE_TIME );
     
}
