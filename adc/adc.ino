/*
  Project: ADC recorder
  Started: 2 July 2016
*/

#include <Stepper.h>
#include <Time.h>
#include <Math.h>


//
// SD Filesystem
//
/* SeeedStudo SD Shield:
http://www.seeedstudio.com/wiki/SD_Card_shield_V4.0
D4: SD_CS;
D11: SD_DI;
D12: SD_DO;
D13: SD_CLK.
*/

// choose appropriate SD Shield by commenting the non-used shield:
// #define SEEED_SD_SHIELD
#define OSEPP_SD_SHIELD


#ifdef OSEPP_SD_SHIELD
#include <SD.h>
SdFile root, file;
#define SD_CS_PIN 10

#else  // Arduino SD shield
#include <SPI.h>
#include <SdFat.h>
SdFat SD;
File file;
#define SD_CS_PIN 53

#endif




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
#define solenoidPin 40

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

#define POS_OPEN +1
#define NEG_SHUT -1

const int stepsPerRevolution = 300;  // Change this to fit the number of steps per revolution for your motor


// 
// Initialize the stepper library on pins 12 and 13:
Stepper myStepper(stepsPerRevolution, dirA, dirB);            
 
//
// calculate stepper motor position based on MAP and tank pressure
/*     
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


const char *adc_labels[ANALOG_VALUES] = { "MAP", "IN", "OUT", "NG", "PSI", "EGT", "TT", "Flow", "Solenoid", "Previous", "Stepper" };
float adc_values[ANALOG_VALUES] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

const int psi_array[] = { 30, 40, 50, 60, 70, 80, 100, 200, 250, 300, 350, 400, 450, 500, 550, 600, 650, 700 };
#define PSI_STEPS ( sizeof(psi_array) / sizeof(int) )

const int map_array[] = { 110, 115, 120, 125, 130, 135, 140, 145, 150 };
#define MAP_STEPS ( sizeof(map_array) / sizeof(int) )

const int stepper_motor_values[PSI_STEPS][MAP_STEPS] = {
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


//
// globals
char g_str_buffer[125];           //This will be a data buffer for writing g_str_buffer to the file.



//
//
void setup()
{
#ifdef OSEPP_SD_SHIELD
     Sd2Card card;
     SdVolume volume;
#endif


     // Serial output
     Serial.begin( 9600 );           // set up Serial library at 9600 bps
     Serial.println( "\n\n\n" );
     
     // ADC initialization
     // https://www.arduino.cc/en/Tutorial/DigitalPins
     pinMode( A0, INPUT );
     pinMode( A1, INPUT );
     pinMode( A2, INPUT );
     pinMode( A3, INPUT );
     pinMode( A4, INPUT );
     pinMode( A5, INPUT );
     pinMode( A6, INPUT );


     // 
     // NG Solenoid switch to high for NG to flow -- when not using NG, set to LOW
     // toggle solendoid switch to provide audible validation circuit is working
     pinMode( solenoidPin, OUTPUT );
     digitalWrite( solenoidPin, HIGH );
     delay( 250 );
     digitalWrite( solenoidPin, LOW );
     


     //
     // Motor Initialization
     // set the speed in rpm:
     myStepper.setSpeed(120);     

     pinMode( enA, OUTPUT );
     digitalWrite( enA, HIGH );
     pinMode( enB, OUTPUT );
     digitalWrite( enB, HIGH );

     // initially close both the solenoid and NG valve.
     myStepper.step( POS_OPEN * stepsPerRevolution );   
     adc_values[solenoid_position] = (float)LOW;   // because we store in the ADC array ... convenience.
     digitalWrite( solenoidPin, (int)adc_values[solenoid_position] );

     adc_values[previous_stepper] = stepsPerRevolution;
     adc_values[stepper_value] = stepsPerRevolution;
    
     Serial.println( "Setup configured motor to 120rpm, 300 steps stop to stop.  Solenoid off, and NG valve shut." );

     //
     // LED shall indicate SD card activity
     pinMode( ledPin, OUTPUT );
     digitalWrite( ledPin, HIGH );

     //
     // SD Fat initialization
#ifdef OSEPP_SD_SHIELD
     pinMode( SD_CS_PIN, OUTPUT );       // output for the SD communication to work.
     card.init();               //Initialize the SD card and configure the I/O pins.
     volume.init(card);         //Initialize a volume on the SD card.
     root.openRoot(volume);     //Open the root directory in the volume. 
     file.open( root, "DATA.TXT", O_CREAT | O_WRITE ); // O_APPEND 

#else
     Serial.print( "SdFat initialization ..." );
     if( !SD.begin( SD_CS_PIN ) )
	  Serial.println( " failed." );

     file = SD.open( "DATA.TXT", FILE_WRITE );

#endif
     sprintf( g_str_buffer, "\ntime:\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n",
	      adc_labels[0], adc_labels[1], adc_labels[2], adc_labels[3],
	      adc_labels[4], adc_labels[5], adc_labels[6], adc_labels[7], adc_labels[8],
	      adc_labels[9], adc_labels[10], adc_labels[11] );
     file.print( g_str_buffer );
     file.close();
     Serial.print( g_str_buffer );

     digitalWrite( ledPin, LOW );
        
}



//
//
void write_data( float adcvals[ANALOG_VALUES] )
{
     int i;
     char buff[20];
     
     digitalWrite( ledPin, HIGH );

#ifdef OSEPP_SD_SHIELD
     file.open( root, "DATA.TXT", O_CREAT | O_APPEND | O_WRITE );
#else
     file = SD.open( "DATA.TXT", FILE_WRITE );
#endif


     // date time stamp
     sprintf( g_str_buffer, "%d:%d:%d\t", hour(), minute(), second() );
     Serial.print( g_str_buffer );
     file.print( g_str_buffer );

     // for sequence of values, refer to index define's above ANALOG_VAlUES
     // nicer would be to have a matching array of value names.
     for( i=0; i < ANALOG_VALUES; i++ ) {
	  dtostrf( adcvals[i], 4, 3, buff );   // sadly, Arduino doesn't print floats natively
	  sprintf( g_str_buffer, "%s, ", buff );
	  Serial.print( g_str_buffer );
	  file.print( g_str_buffer );
     }
     sprintf( g_str_buffer, "\r\n" );
     Serial.print( g_str_buffer );
     file.print( g_str_buffer );

     file.close();

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
	  return stepsPerRevolution;
     else
	  return linear( enginemap,
			 (float)stepper_motor_values[psi_index][map_index], (float)map_array[map_index],
			 (float)stepper_motor_values[psi_index][map_index + 1], (float)map_array[map_index+1] );


}


//
//
void loop()
{
     int stepperDirection = 0;


     //
     // Arduino ADC has 1024 steps.  Volts = Ain / 1024
     // read analog inputs (twice to stabilize ADC), convert to volts, apply calculation from spreadsheet.
     // calculate diesel flow as g/s, round tank press to nearest 10.
     analogRead( map_pin );
     adc_values[map] = (39.958 * (analogRead(map_pin) * (5.0/1023)) ) + 8.142;
     analogRead( dieselflow_in_pin );
     adc_values[dieselflow_in] = (1.3442 * (analogRead(dieselflow_in_pin) * (12.0/1023.0)) ) - 0.0168;
     analogRead( dieselflow_out_pin );
     adc_values[dieselflow_out] = (0.5767 * (analogRead(dieselflow_out_pin) * (12.0/1023.0)) ) - .0102;
     analogRead( ngflowmeter_pin );
     adc_values[ngflowmeter] = ((40.276 * (analogRead(ngflowmeter_pin) * (5.0/1023.0)) ) - .2208) * .7175/60;
     analogRead( psi_pin );
     adc_values[psi] = (268.68 * (analogRead(psi_pin) * (12.0/1023.0)) ) - 115.62;
     analogRead( exhaustgastemp_pin );
     adc_values[exhaustgastemp] = (249.21 * (analogRead(exhaustgastemp_pin) * (12.0/1023.0)) ) - 2.5189;
     analogRead( tanktemp_pin );
     adc_values[tanktemp] = (197.9 * (analogRead(tanktemp_pin) * (12.0/1023.0)) ) - 251.2;

     // calculate flow based on IN - OUT
     adc_values[diesel_flow] = adc_values[dieselflow_in] - adc_values[dieselflow_out];

     
     // stepper value to output is based on a lookup table.
     adc_values[stepper_value] = calc_stepper_value( adc_values[psi], adc_values[map] );

     adc_values[stepper_value] = trunc( adc_values[previous_stepper] -
					( stepsPerRevolution - calc_stepper_value( xin, yin ) ) );


     // if new position, then move stepper.
     // and, set state of solenoid switch based on position of NG valve
     if( adc_values[stepper_value] != 0 ) {
          myStepper.step( adc_values[stepper_value] );
	  adc_values[solenoid_position] = LOW;
	  if( adc_values[stepper_value] != stepsPerRevolution )
	       adc_values[solenoid_position] = HIGH;

     }
     adc_values[previous_stepper] = adc_values[stepper_value]; 

     // set solenoid switch state.
     digitalWrite( solenoidPin, adc_values[solenoid_position] );

     write_data( adc_values );


     delay( 1000 );
     
}
