 /*
  * ------------------------------------------------------------------------------------
  * Vehicle Speed Detection with HB100 X-Band Motion Sensor and Arduino UNO
  * Determine viability of the HB100 and test vehicle detection and speed measurement algorithms.
  * This is a developmental step to a project that may never finish, or never be finished.
  * Posted to GitHub in the hope it might prove useful to someone. 
  * YRMV but YOYO.
  * 
  * D.L. Poole -  27 August 2015
  * 
  * Ultimate Objective:
  * To clock and log speeds of vehicles moving along a residential street from a 
  * mailbox position along the side of the street.  While the HB100 lacks the range to 
  * clock a vehicle and warn a driver "Your Speed is" on a co-located display in time  
  * for him to read it, my unit proved sufficient to clock a vehicle before it passed the sensor.  
  * 
  * Hardware:
  * An IF Amplifier design per the AgileSense MSAN-001 Application Note at
  * http://www.limpkin.fr/public/HB100/HB100_Microwave_Sensor_AInstantaneousPeriodlication_Note.pdf
  * with the following changes:
  *   Reduced 2.3nF capacitors to 47pF to raise LP corner and pass vehicle doppler shifts at 50 MPH 
  *   Reduced 4.7nF capacitors to 0.22uF to raise HP corner (may not have been necessary)
  *   Added a 0.5v P-P output through 10K/1K voltage divider for field recording on a laptop
  *   
  * Experimental Setup
  * Doppler shifts of passing vehicles were recorded as .wav files with a laptop's record gain set 
  * to slightly below clipping at the clip point of the IF amplifier, that is, the desirable amplifier 
  * clipping was still present, but recorded linearly, so it could later be normalized to 0dBFS.  The 
  * laptop's headphone output was capable of 4V P-P at full volume and thus could recreate the field 
  * signals on the bench for software development and testing.  The results achieved required clipping   
  * at a still lower signal level.  This was done by amplifying the .wav file +18 db, thereby inducing
  * "digital" clipping within the audio editor before playing it back into the Arduino. This *should* 
  * be equivalent to inducing it in the IF amplifier itself by reducing one or both gain setting 
  * resistors for an additional 18db then taking new field data, but this hasn't been tested.  It
  * might cause stability problems requiring an additional stage. 
  *   
  * The laptop's audio out was interfaced to an Arduino UNO per the introductory page to the 
  * FrequencyPeriod library, which can be found at 
  * http://interface.khm.de/index.php/lab/interfaces-advanced/frequency-measurement-library
  * KHM 2010 /  Martin Nawrath
  * Kunsthochschule fuer Medien Koeln
  * Academy of Media Arts Cologne
  *
  * This interface uses an additional resistor and digitalwrites to pin 5 to introduce an offset, and 
  * thereby some hysteresis into the timer comparison function of the Arduino.  That library was forked 
  * into this code with that offset initialized differently to keep the timer quiescent and the serial 
  * dump empty until the field audio was replayed and to synchronize a data dump described below with 
  * the audio file. 
  * 
  * Period Measurement and Frequency conversion
  * getPeriod() returns an instantaneous period of the input measured in 16MHz clocks.  The input 
  * is squared up with a software drive hysteresis mechanism, so the period between 
  * two negative going zero crossings is measured.  With no vehicle present, the thermal noise from 
  * the HB100 sensor results in random period measurements. The instantaneous period is converted to 
  * an instantaneous frequency by dividing it into the clock frequency.
  * 
  * Frequency Averaging  
  * As the target approaches the sensor, the return signal progressively overcomes the thermal noise 
  * and the period approaches that of the Doppler shift with progressively less variability. 
  * Noise is removed and the Doppler shift is estimated by use of an exponential moving average in 
  * which SmoothingCoefficient was an original experimental parameter.  The lower that smoothing 
  * coefficient, the greater the weight of prior data on the result, the lower the noise 
  * contribution, and the greater the accuracy of the estimate, at the expense of a slower response
  * to the shift. 
  * 
  * Signal to Noise Ratio Measurement
  * The degree of randomness is measured by a moving variance, that is, a similar exponential 
  * moving average, but of the squared difference between the instantaneous periods and the 
  * moving average period. This is square rooted into a standard deviation for ease of comparison
  * with the moving average period.  The same smoothing coefficient is used as for the moving
  * average frequency and the moving variance so the two measures track.  Since standard deviation 
  * represents Signal to Noise ratio, comparing it with fixed thresholds determines the presence 
  * or loss of a target.  
  * 
  * Measurement Trigger
  * Since the sensor must lie to the side of the path of the target, there exists a point 
  * where the Doppler shift begins to fall due to the increasing cosine of the bearing 
  * angle.  The Doppler shift rapidly falls towards a zero beat as the vehicle comes abreast 
  * of the sensor.  Though the path loss is low and the return signal to noise ratio is high
  * at that point, the rapidly decreasing frequency relative to the moving average frequency 
  * causes the moving variance to increase sharply. It was found experimentally that the 
  * moving average frequency accurately represents the actual target speed at the moment the 
  * moving variance reaches its minimum.  For vehicles passing the sensor from behind, the
  * sensors antennae patterns preclude detection until the zero beat and the moving standard 
  * deviation increase more slowly as the vehicle leaves the field. Only a slow vehicle within
  * about 10 feet (in the same lane) can be detected and the accuracy is reduced by heading angle. 
  * 
  * Debug and Visualization Dump
  * if PrintRawData=true, a tab-delimited dump is sent to the serial monitor suitable for pasting into
  * a spreadsheet for plotting or analysis. This proved invaluable in visualizing how moving standard
  * deviation is indicative of the presence of a vehicle and in choosing the smoothing coefficient and
  * thresholds.  
  * 
  * A log time starts with the first valid instantaneous period so that it matches time scale of the
  * audio file being used.
  * A dump line is issued for each instantaneouls period consisting of: 
  * LogTime  Period  Frequency  MovingAvgFrequency  MovingStdDevShift  MovingAvgSpeed  Status
  * The Status provides a way for an Excel plot on a miles-per-hour scale to show how and when the algorithm 
  * detects the vehicle captures its speed; Status is 10 as long as the moving standard deviation 
  * suggests no vehicle is present; it is 20 when the moving standard falls to the detection threshold 
  * for a vehicle. Noise will change it randomly between 20 and 30 as the moving standard reaches
  * a lower value, or not.  The estimate dvehicle speed is revised at each new lower value, which becomes 
  * the determined speed as the moving stardard begins its rise as explained above. When the moving standard 
  * deviation exceeds a second, higher threshold, the vehicle is presumed to have passed the sensor and the 
  * determined speed is appended to that line of the dump. The ultimate intent is to remove the logging 
  * to save memory and convert this into a function that awaits a vehicle and returns its speed for time of 
  * day and speed logging.  
  * 
  * Accuracy
  * Absolute accuracy depends on the 10GHz operating frequency of the HB-100, the clock frequency of 
  * the Arduino, and the bearing angle of the target.  The first is rather difficult to measure without
  * advanced gear and will be significantly different if you live in a non-US jurisdication or buy a 
  * non-US HB100 on the Internet; the Arduio's clock speed isn't specified as to accuracy but is more 
  * easily measured. This code assumes both are on frequency and doesn't attempt to correct for bearing angle, 
  * but it has agreed with the speedometer on my Prius when driving past it at 20-30 MPH, and clocked
  * vehicles doing 33.5 to 39.9 headed into a curve in a 35 MPH zone.  YRMV, but a particular HB-100 and
  * Arduino could be recalibrated against a vehicle moving a known speed by adjusting the conversion factor 
  * determined in setup().
  *  
  */
bool PrintRawData;                //print debug/visualization log if true
bool VehiclePresent;              //vehicle present if true
long int InstantaneousPeriod;     //instantaneous period in 16 MHz counts
long int LogTime;                 //log start time in millis()
float FreqPerPeriod;              //measured frequency per period
float SmoothingCoefficient;       //smoothing SmoothingCoefficienticient for moving average
float MovingAvgFreq;              //moving average frequency
float MovingVarFreq;              //moving variance of frequency
float MovingStdDevFreq;           //standard deviation of moving variance
float MovingAvgSpeed;             //average miles per hour
float MinMovingStdDevFreq;        //minimum signal to noise point
float VehiclePresentThreshold;    //variance threshold to detect vehicle
float VehiclePassedThreshold;     //variance threshold to detect vehicle has passed
float VehicleSpeed;               //vehicle speed result
float SensorFrequency;            //Assumed or measured RF frequency of HB-100 sensor module
float ConversionFactor;           //MPH per Hz of Doppler shift set in setup() per Doppler Equation
/*
 * Setup Coefficients. Experimental coefficients
 */
void setup() {
  SensorFrequency=10.525e+9;            //Assumed of measured RF frequency of HB-100 sensor module
  ConversionFactor=2.0*SensorFrequency/(6.707e+8);  //Doppler Shift Formula c in MPH
  SmoothingCoefficient=0.01;            //smoothing coefficient for moving averages

  VehiclePresentThreshold=300.0;        //threshold moving variance to detect vehicle
  VehiclePassedThreshold=400.0;         //threshold moving variance to lose track of vehicle
  MovingVarFreq=400.0*400.0;            //initialize moving variance to start
  VehiclePresent=false;                 //initialize to idle state
  PrintRawData=true;                    //for debugging dump
  Serial.begin(115200);                 //set up serial port for debugging dump
  begin();                              //start interrupt services
  Serial.println("Setup Complete");     //print banner
  Serial.println("HINT: To copy this data, select and copy a few lines at the top,");
  Serial.println("scroll to the bottom, then shift-copy to the bottom.");
/*
 * Wait until the hardware input first becomes active then initialize LogTime so logged times follows sample time scale
 */
  while (getPeriod()==0){
  }
  LogTime=millis();
}
/*
 * Main Loop.  You can check out any time you like, but you can never leave.
 */
void loop() {
/*
 * Get latest period in 16Mhz clocks, not necessarily successive periods as cpu time may exceed interrupt rate.
 * Ignore periods longer than 10 Hz, e.g. periodic counter overflows when there is no input, and noise-induced 
 * periods shorter than 2KHz which only waste CPU time. Calculate the instanteous Doppler shift in Hz. 
 * Calculate moving average and a moving variance on instantaneous frequency. Convert moving variance to 
 * a moving standard deviation on the same scale as the moving average.  Use the conversion factor  
 * developed in setup() to convert moving average (Doppler Shift) frequency to a moving average vehicle speed 
 * in MPH.
 */
  InstantaneousPeriod=getPeriod();    //a valid period, not necessarily a contiguous one
    if (InstantaneousPeriod>8000 && InstantaneousPeriod<160000 ){    //ignore obviously invalid returns
      FreqPerPeriod= 16000000.0 / InstantaneousPeriod;    //calculate frequency from period
      MovingAvgFreq=(1.0-SmoothingCoefficient)*MovingAvgFreq+SmoothingCoefficient*FreqPerPeriod;  //compute moving average frequency
      MovingVarFreq=(1.0-SmoothingCoefficient)*MovingVarFreq+SmoothingCoefficient*(FreqPerPeriod-MovingAvgFreq)*(FreqPerPeriod-MovingAvgFreq);  //compute moving average std dev
      MovingStdDevFreq=sqrt(MovingVarFreq);
      MovingAvgSpeed=MovingAvgFreq/ConversionFactor;    //compute average vehicle speed
/*
 * Optionally dump debug/visualization data
 */
        if(PrintRawData==true){
        Serial.print (millis()-LogTime );
        Serial.print ("\t");
        Serial.print (InstantaneousPeriod);
        Serial.print ("\t");
        Serial.print(FreqPerPeriod);
        Serial.print("\t");
        Serial.print(MovingAvgFreq);
        Serial.print("\t");
        Serial.print(MovingStdDevFreq);
        Serial.print("\t");
        Serial.print(MovingAvgSpeed);
        Serial.print("\t");
        }
      if (VehiclePresent==false) {
/*
 * No vehicle present.  Test moving variance against threshold and detect vehicle
 * if below it.  Status 10 is added to the debug line to plot as no vehicle present
 */     
          MinMovingStdDevFreq=VehiclePresentThreshold;
          if(MovingStdDevFreq<VehiclePresentThreshold){
            VehiclePresent=true;
          }
              if(PrintRawData==true){
              Serial.print(10);
              }
      }
      else {
/*
 *  Vehicle is present.  Track the minimum moving variance during the presence and
 *  the moving average speed at that minimumm.  Add status 20 to the debug line if 
 *  this datum lowered the minimum or add status 30 if it did not. 
 */
      if(MovingStdDevFreq<MinMovingStdDevFreq) {
        MinMovingStdDevFreq=MovingStdDevFreq;
        VehicleSpeed=MovingAvgSpeed;
            if(PrintRawData==true){
            Serial.print(20);
            }              
      }
      else {
            if(PrintRawData==true){
            Serial.print(30);
            }
      }
      /*
       * Detect passed vehicle and change state.  Append settled speed to the debug print
       * after it has passed 
       */
      if(MovingStdDevFreq>VehiclePassedThreshold){
        VehiclePresent=false;
        Serial.print ("\t");
        Serial.print(VehicleSpeed);
      }
    }
    /*   
     *  Close out the debug print line if one
     */
    if(PrintRawData==true){
    Serial.println();
    }
  }
}

//-----------------------------------------------------------------------------------------------------
/*
  FreqPeriod 
  Counter1 unit capturing analog comparator event measuring period length of analog signal on input AIN1 
  AIN0 set to VCC/2 by resistors
  pin5 used as output for comparator feedback
  Analog comparator interrupt service is used to calculate period length
  Timer1 overflow interrupt service used to count overflow events

  Martin Nawrath KHM LAB3
  Kunsthochschule für Medien Köln
  Academy of Media Arts
  http://www.khm.de
  http://interface.khm.de/index.php/labor/experimente/
 
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  
  History:
    June 2010 - V0.0 
    Jan/12 - V1.1    Arduino 1.0
*/
volatile unsigned char  f_capt;   //block any compiler optimization
volatile unsigned int capta;
volatile unsigned long int captd;
volatile int ocnt;

void begin(){
  
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega48__) || defined (__AVR_ATmega88__) || defined (__AVR_ATmega328P__) || (__AVR_ATmega1280__)

  pinMode(5,OUTPUT);
  digitalWrite(5,1);  //Stabilize Comparator until first transition

  TCCR1A =0;
  TCCR1B =0;

  ADCSRB &= ~(1<<ACME); //Select AIN1 as negative input to comparator 
  ACSR &= ~(1<<ACIE); //Disable Comparator Interrupt before enabling comparator 
  ACSR &= ~(1<<ACD) ;    // Analog Comparator disable off
  // ACSR |= (1<<ACBG);   // Analog Comparator Bandgap reference on
  ACSR |= (1<<ACIE);   // Analog Comparator Interrupt enable
  ACSR |= (1<<ACIC);   // Enable Comparator for Timer/Counter 1 capture

  ACSR &= ~(1<<ACIS0); // Interrupt on Falling Edge
  ACSR |= (1<<ACIS1);  // Interrupt on Falling Edge

  //  CS12 CS11 CS10 Description
  //    0   0    0   No clock source (Timer/Counter stopped).
  //    0   0    1   clkI/O/1 (No prescaling)
  //    0   1    0   clkI/O/8 (From prescaler)
  //    0   1    1   clkI/O/64 (From prescaler)
  //    1   0    0   clkI/O/256 (From prescaler)

  TCCR1B |= (1<<CS10); // set prescaler to 16 MHz
  TCCR1B &= ~(0<<CS11);
  TCCR1B &= ~(0<<CS12);

  TCCR1B |= (1<<ICNC1);  // input noise canceler on
  TCCR1B &= ~(1<<ICES1); // input capture edge select =0 

  DIDR1 |= (1<< AIN0D);  // disable digital input buffer AIN0/1
  DIDR1 |= (1<< AIN1D);

  TIMSK1 |= (1<<TOIE1);  // Timer/Counter1, Overflow Interrupt Enable

//  TIMSK0 &= ~(1<<TOIE0); // Timer/Counter0, Overflow Interrupt disable
  
#endif

}
//***************************************************************************
unsigned long int  getPeriod() {
      unsigned long int rr= 0; 
  if ( f_capt){
    f_capt=0;
    rr= captd;
}
return(rr);
}

//***************************************************************************
// Timer1 Overflow Interrupt Service
ISR(TIMER1_OVF_vect   ) {
  ocnt++;  // count number of timer1 overflows
}
//***************************************************************************
// Analog Comparator Interrupt Service
ISR(ANALOG_COMP_vect  ) {

  if (!(ACSR &  (1<<ACIS0))) {  // comparator falling edge
    digitalWrite(5,0);          // reduce comparator threshold level on AIN0
    ACSR |= (1<<ACIS0);         // next comparator detection on rising edge 

            // compute period  length  timer1 capture value ,  account timer1 numer of overflows  
    captd= ICR1 + ocnt* 0x10000; 

            // compute period difference 
    captd=captd-capta; 
          
    capta=ICR1;     // store capture value
    ocnt=0;         // reset number of timer1 overflows
    f_capt=1;       // measure ready flag
  }   
  else  {
    ACSR &= ~(1<<ACIS0);         //  next comparator detection on falling edge 
    digitalWrite(5,1);           // elevate comparator threshold level on AIN0
  }
}
//-----------------------------------------------------------------------------------------------------

