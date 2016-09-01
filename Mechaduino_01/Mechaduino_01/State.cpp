  //Contains the declaration of the state variables for the control loop  


//interrupt vars

volatile float ei = 0.0;
volatile int U = 0;  //control effort (abs)
volatile float r = 0.0;  //setpoint
volatile float y = 0.0;  // measured angle
volatile float yw = 0.0;
volatile float yw_1 = 0.0;
volatile float e = 0.0;  // e = r-y (error)
volatile float p = 0.0;  // proportional effort
volatile float i = 0.0;  // integral effort
volatile float PA = 1.8;  //

volatile float u = 0.0;  //real control effort (not abs)
volatile float u_1 = 0.0;
volatile float e_1 = 0.0;
volatile float u_2 = 0.0;
volatile float e_2 = 0.0;
volatile float u_3 = 0.0;
volatile float e_3 = 0.0;
volatile long counter = 0;

volatile long wrap_count = 0;
volatile float y_1 = 0;

//const float iMAX = 1.0;
const float iMAX = 0.3;          // Maximum Motor current in amps  ?? This per winding -> Do we need to multiply by 2?
//const float rSense = 0.150;      // MGJ_Issues - sense resistors Schematic shows .25 ohms! So this value is wrong.
const float rSense = 0.250;      // MGJ_Issues - sense resistors Schematic shows .25 ohms! So this value is wrong.

volatile int uMAX = (255/3.3)*(iMAX*10*rSense); // MGJ_Issues - Where is this formula derived from?  (77.27272727272727) * (.3*10*.15) = 77.27272727272727 *0.45 
                                                //uMax = 34.77272727272727 ???  

// MGJ_Issues - rSense .25     (77.27272727272727) * (.3*10*.25) = 77.27272727272727 *0.75 
                                                //uMax = 57.95454545454545 ???

// from A4954 Dual Full-Bridge DMOS PWM Motor Driver data sheet
/*   VREF
      The maximum value of current limiting is set by the selection of RSx and the voltage at the VREFx pin in each channel. 
      The transconductance function is approximated by the maximum value of current limiting, ITripMAX (A), which is set by:

    ITripMAX = VREF/(Av*RS)
    
     where VREF is the input voltage on the VREFx pin (V) (3.3V) and RS (.25 Ohms) is the resistance of the sense resistor (Ω) on the corresponding LSSx terminal.
     ITripMax is the rating of the motor 0.3Amps
     Solving for Av  =>   Av*RS = VREF/ITripMax => Av = (VREF/ITripMax) / RS
     AV = (3.3/0.3)/.25 = 44

     What is AV?
   *                                                 
 */

volatile float ITerm;

volatile char mode;


//___________________________________

const float pi = 3.14159;
const int  half = 134;//128;                // MGJ_Issues this variable appears not to be used.

float new_angle = 0.0; //input angle        // MGJ_Issues this variable appears not to be used.
float current_angle = 0.0; //current angle  // MGJ_Issues this variable appears not to be used.
float diff_angle = 0.0;                     // MGJ_Issues this variable appears not to be used.
int val1 = 0;                               // Appears to be VREF_2 for A4954 Motor Controller Chip - see output() function
int val2 = 0;                               // Appears to be VREF_1 for A4954 Motor Controller Chip - see output() function
