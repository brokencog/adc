/*

  Project: ADC recorder
  Started: 2 July 2016

*/


#include <Wire.h>
#include <Adafruit_MotorShield.h>

#include <Time.h>
#include <TimeLib.h>
#include <Math.h>
#include <SD.h>


//
// SD Filesystem
//
//Create the variables to be used by SdFat Library
SdFile root, file;
Sd2Card card;
SdVolume volume;
char datetimestr[25] = "formatDateTime()";
char contents[125];           //This will be a data buffer for writing contents to the file.


//
// analog input pin/value pairs.
//
#define map_pin A0      // 5v
#define dieselflow_in_pin A1   // 12v
#define dieselflow_out_pin A2  // 12v
#define ngflowmeter_pin A3     // 12v
#define psi_pin A4    // 5v
#define exhaustgastemp_pin A5  // 12v
#define tanktemp_pin A6        // 12v
#define ledPin 13

#define map 0
#define dieselflow_in 1
#define dieselflow_out 2
#define ngflowmeter 3
#define psi 4
#define exhaustgastemp 5
#define tanktemp 6
#define diesel_flow 7
#define stepper_value 8
#define ANALOG_SOURCES 9
     
char *adc_labels[ANALOG_SOURCES] = { "MAP", "IN", "OUT", "NG", "PSI", "EGT", "TT", "Flow", "Stepper" };
float adc_values[ANALOG_SOURCES] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };


// Stepper motor
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *myMotor = AFMS.getStepper( 200, 2 );

//
//
void setup()
{
     // Serial output
     Serial.begin( 9600 );           // set up Serial library at 9600 bps

     // SD fs
     pinMode(10, OUTPUT);       //Pin 10 must be set as an output for the SD communication to work.
     card.init();               //Initialize the SD card and configure the I/O pins.
     volume.init(card);         //Initialize a volume on the SD card.
     root.openRoot(volume);     //Open the root directory in the volume. 

     // ADC
     pinMode( ledPin, OUTPUT );

     formatDateTime();
     digitalWrite( ledPin, HIGH );
     sprintf( contents, "\nmin:sec: %s, %s, %s, %s, %s, %s, %s, %s, %s\n",
	      adc_labels[0], adc_labels[1], adc_labels[2], adc_labels[3],
	      adc_labels[4], adc_labels[5], adc_labels[6], adc_labels[7], adc_labels[8] );

     file.open( root, "DATA.TXT", O_CREAT | O_WRITE ); // O_APPEND
     file.print( contents );
     file.close();
     digitalWrite( ledPin, LOW );
     
     Serial.print( contents );

     // Motor
     AFMS.begin();    
     myMotor->setSpeed( 10 );
     Serial.println( "Setup configured motor to 10rpm.");

}


//
//

void formatDateTime()
{
     sprintf( datetimestr, "%d/%d/%d %d:%d:%d", year(), month(), day(), minute(), second() );
}


//
//
void write_data( float adcvals[ANALOG_SOURCES] )
{
     int i;
     char buff[20];
     
     digitalWrite( ledPin, HIGH );

     file.open( root, "DATA.TXT", O_CREAT | O_APPEND | O_WRITE );

     formatDateTime();
     
     sprintf( contents, "%s: ", datetimestr );
     Serial.print( contents );
     file.print( contents );

     // for sequence of values, refer to index define's above ANALOG_SOURCES
     // nicer would be to have a matching array of value names.
     for( i=0; i < (ANALOG_SOURCES - 1); i++ ) {
     	  dtostrf( adcvals[i], 4, 3, buff );   // sadly, Arduino doesn't print floats natively
     	  sprintf( contents, "%s, ", buff );
     	  Serial.print( contents );
     	  file.print( contents );
     }
     dtostrf( adcvals[i], 4, 3, buff );
     sprintf( contents, "%s\n", buff );
     Serial.print( contents );
     file.print( contents );

     file.close();

     digitalWrite( ledPin, LOW );

}


//
// calculate stepper motor position based on MAP and tank pressure
/*     

           |  110  115	120	125	130	135	140	145	150
       -----------------------------------------------------------------------------
       30  |  132  156	180	222	234	246	0	0	0    
       40  |  132  156	180	222	234	246	0	0	0
       50  |  132  156	180	222	234	246	180	192	210
       60  |  135  159	184	221	236	244	267	290	313
       70  |  138  162	186	222	234	246	264	288	312
       80  |  132  156	181	217	232	240	264	287	309
       100 |  133  157	182	218	233	241	264	287	309
       200 |  119  141	164	198	211	219	240	261	282
       250 |  76   98 	120	152	165	172	192	213	233
       300 |  67   89 	112	145	159	166	187	208	229
       350 |  80   102	125	158	171	178	199	220	241
       400 |  86   107	129	162	175	182	202	223	243
       450 |  89   112	135	168	182	189	211	232	253
       500 |  84   102	126	156	168	174	198	216	234
       550 |  72   96 	114	144	156	162	180	204	222
       600 |  60   78 	102	126	138	144	162	180	198
       650 |  42   60 	78	102	114	120	138	156	174
       700 |  18   36 	54	78	84	90	108	120	138

*/

//
// helper routine input value is tank pressure (30, 40, 50 ...) return is index in tank_psi_array
//
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


//
//
void loop()
{

     // this flash won't be visible unless Serial terminal is connected
     // as things are too fast without it.
     digitalWrite( ledPin, HIGH );

     //
     // Arduino ADC has 1024 steps.  Volts = Ain / 1024
     // read analog inputs, convert to volts, apply calculation from spreadsheet.
     // calculate diesel flow as g/s, round tank press to nearest 10.
     adc_values[map] = (39.958 * (analogRead(map_pin) * (5.0/1023)) ) + 8.142;
     adc_values[dieselflow_in] = (1.3442 * (analogRead(dieselflow_in_pin) * (12.0/1023.0)) ) - 0.0168;
     adc_values[dieselflow_out] = (.5767 * (analogRead(dieselflow_out_pin) * (12.0/1023.0)) ) - .0102;
     adc_values[ngflowmeter] = ((40.276 * (analogRead(ngflowmeter_pin) * (12.0/1023.0)) ) - .2208) * .7175/60;
     adc_values[psi] = (268.68 * (analogRead(psi_pin) * (5.0/1023.0)) ) - 115.62;
     adc_values[exhaustgastemp] = (249.21 * (analogRead(exhaustgastemp_pin) * (12.0/1023.0)) ) - 2.5189;
     adc_values[tanktemp] = (197.9 * (analogRead(tanktemp_pin) * (12.0/1023.0)) ) - 251.2;
    
     // calculate flow based on IN - OUT
     adc_values[diesel_flow] = adc_values[dieselflow_in] - adc_values[dieselflow_out];
     // stepper value to output is based on a lookup table.
     adc_values[stepper_value] = calc_stepper_value( adc_values[psi], adc_values[map] );

     write_data( adc_values );

     myMotor->step( adc_values[stepper_value], FORWARD, SINGLE );

     digitalWrite( ledPin, LOW );

     delay( 500 );
     
}
