  Vehicle Speed Detection with HB100 X-Band Motion Sensor and Arduino UNO
  Determine viability of the HB100 and test vehicle detection and speed measurement algorithms.
  This was a developmental step to a project that might never be finished.
  Posted to GitHub in the hope it might prove useful to someone. 
  YRMV but YOYO.
  
  D.L. Poole -  27 August 2015
  
  Ultimate Objective:
  To clock and log speeds of vehicles moving along a residential street from a 
  mailbox position along the side of the street.  While the HB100 lacks the range to 
  clock a vehicle and warn a driver "Your Speed is" on a co-located display in time  
  for him to read it, my unit proved sufficient to clock a vehicle before it passed the sensor.  
  
  Hardware:
  An IF Amplifier design per the AgileSense MSAN-001 Application Note at
  http://www.limpkin.fr/public/HB100/HB100_Microwave_Sensor_AInstantaneousPeriodlication_Note.pdf
  with the following changes:
    Reduced 2.3nF capacitors to 47pF to raise LP corner and pass vehicle doppler shifts at 50 MPH 
    Reduced 4.7nF capacitors to 0.22uF to raise HP corner (may not have been necessary)
    Added a 0.5v P-P output through 10K/1K voltage divider for field recording on a laptop
    
  Experimental Setup
  Doppler shifts of passing vehicles were recorded as .wav files with a laptop's record gain set 
  to slightly below clipping at the clip point of the IF amplifier, that is, the desirable amplifier 
  clipping was still present, but recorded linearly, so it could later be normalized to 0dBFS.  The 
  laptop's headphone output was capable of 4V P-P at full volume and thus could recreate the field 
  signals on the bench for software development and testing.  The results achieved required clipping   
  at a still lower signal level.  This was done by amplifying the .wav file +18 db, thereby inducing
  "digital" clipping within the audio editor before playing it back into the Arduino. This *should
  be equivalent to inducing it in the IF amplifier itself by reducing one or both gain setting 
  resistors for an additional 18db then taking new field data, but this hasn't been tested.  It
  might cause stability problems requiring an additional stage. 
    
  The laptop's audio out was interfaced to an Arduino UNO per the introductory page to the 
  FrequencyPeriod library, which can be found at 
  http://interface.khm.de/index.php/lab/interfaces-advanced/frequency-measurement-library
  KHM 2010 /  Martin Nawrath
  Kunsthochschule fuer Medien Koeln
  Academy of Media Arts Cologne
  
  This interface uses an additional resistor and digitalwrites to pin 5 to introduce an offset, and 
  thereby some hysteresis into the timer comparison function of the Arduino.  That library was forked 
  into this code with that offset initialized differently to keep the timer quiescent and the serial 
  dump empty until the field audio was replayed and to synchronize a data dump described below with 
  the audio file. 
  
  Period Measurement and Frequency conversion
  getPeriod() returns an instantaneous period of the input measured in 16MHz clocks.  The input 
  is squared up with a software drive hysteresis mechanism, so the period between 
  two negative going zero crossings is measured.  With no vehicle present, the thermal noise from 
  the HB100 sensor results in random period measurements. The instantaneous period is converted to 
  an instantaneous frequency by dividing it into the clock frequency.
  
  Frequency Averaging  
  As the target approaches the sensor, the return signal progressively overcomes the thermal noise 
  and the period approaches that of the Doppler shift with progressively less variability. 
  Noise is removed and the Doppler shift is estimated by use of an exponential moving average in 
  which SmoothingCoefficient was an original experimental parameter.  The lower that smoothing 
  coefficient, the greater the weight of prior data on the result, the lower the noise 
  contribution, and the greater the accuracy of the estimate, at the expense of a slower response
  to the shift. 
  
  Signal to Noise Ratio Measurement
  The degree of randomness is measured by a moving variance, that is, a similar exponential 
  moving average, but of the squared difference between the instantaneous periods and the 
  moving average period. This is square rooted into a standard deviation for ease of comparison
  with the moving average period.  The same smoothing coefficient is used as for the moving
  average frequency and the moving variance so the two measures track.  Since standard deviation 
  represents Signal to Noise ratio, comparing it with fixed thresholds determines the presence 
  or loss of a target.  
  
  Measurement Trigger
  Since the sensor must lie to the side of the path of the target, there exists a point 
  where the Doppler shift begins to fall due to the increasing cosine of the bearing 
  angle.  The Doppler shift rapidly falls towards a zero beat as the vehicle comes abreast 
  of the sensor.  Though the path loss is low and the return signal to noise ratio is high
  at that point, the rapidly decreasing frequency relative to the moving average frequency 
  causes the moving variance to increase sharply. It was found experimentally that the 
  moving average frequency accurately represents the actual target speed at the moment the 
  moving variance reaches its minimum.  For vehicles passing the sensor from behind, the
  sensors antennae patterns preclude detection until the zero beat and the moving standard 
  deviation increase more slowly as the vehicle leaves the field. Only a slow vehicle within
  about 10 feet (in the same lane) can be detected and the accuracy is reduced by heading angle. 
  
  Debug and Visualization Dump
  if PrintRawData=true, a tab-delimited dump is sent to the serial monitor suitable for pasting into
  a spreadsheet for plotting or analysis. This proved invaluable in visualizing how moving standard
  deviation is indicative of the presence of a vehicle and in choosing the smoothing coefficient and
  thresholds.  
  
  A log time starts with the first valid instantaneous period so that it matches time scale of the
  audio file being used.
  A dump line is issued for each instantaneouls period consisting of: 
  LogTime  Period  Frequency  MovingAvgFrequency  MovingStdDevShift  MovingAvgSpeed  Status
  The Status provides a way for an Excel plot on a miles-per-hour scale to show how and when the algorithm 
  detects the vehicle captures its speed; Status is 10 as long as the moving standard deviation 
  suggests no vehicle is present; it is 20 when the moving standard falls to the detection threshold 
  for a vehicle. Noise will change it randomly between 20 and 30 as the moving standard reaches
  a lower value, or not.  The estimate dvehicle speed is revised at each new lower value, which becomes 
  the determined speed as the moving stardard begins its rise as explained above. When the moving standard 
  deviation exceeds a second, higher threshold, the vehicle is presumed to have passed the sensor and the 
  determined speed is appended to that line of the dump. The ultimate intent is to remove the logging 
  to save memory and convert this into a function that awaits a vehicle and returns its speed for time of 
  day and speed logging.  
  
  Accuracy
  Absolute accuracy depends on the 10GHz operating frequency of the HB-100, the clock frequency of 
  the Arduino, and the bearing angle of the target.  The first is rather difficult to measure without
  advanced gear and will be significantly different if you live in a non-US jurisdication or buy a 
  non-US HB100 on the Internet; the Arduio's clock speed isn't specified as to accuracy but is more 
  easily measured. This code assumes both are on frequency and doesn't attempt to correct for bearing angle, 
  but it has agreed with the speedometer on my Prius when driving past it at 20-30 MPH, and clocked
  vehicles doing 33.5 to 39.9 headed into a curve in a 35 MPH zone.  YRMV, but a particular HB-100 and
  Arduino could be recalibrated against a vehicle moving a known speed by adjusting the conversion factor 
  determined in setup().
