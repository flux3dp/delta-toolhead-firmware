#define AUTOMATIC	1
#define MANUAL	0
#define DIRECT  0
#define REVERSE  1
#define true 1
#define false 0
	
double dispKp;				// * we'll hold on to the tuning parameters in user-entered 
double dispKi;				//   format for display purposes
double dispKd;				//
    
double kp;                  // * (P)roportional Tuning Parameter
double ki;                  // * (I)ntegral Tuning Parameter
double kd;                  // * (D)erivative Tuning Parameter

int controllerDirection;

double *myInput;              // * Pointers to the Input, Output, and Setpoint variables
double *myOutput;             //   This creates a hard link between the variables and the 
double *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.
			  
unsigned long lastTime;
double ITerm, lastInput;

unsigned long SampleTime;
double outMin, outMax;
_Bool inAuto;
_Bool newAuto;

double input;
double output;
double error;
double SampleTimeInSec;
double dInput;

unsigned long timeChange;
unsigned long now;
unsigned long gettime;

void setPID(double* Input, double* Output, double* Setpoint,double Kp, double Ki, double Kd, int ControllerDirection);
//commonly used functions **************************************************************************
//void setPID(double*, double*, double*,        // * constructor.  links the PID to the Input, Output, and 
 //       double, double, double, int);     //   Setpoint.  Initial tuning parameters are also set here
	
void PID_SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

_Bool PID_Compute();                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

void PID_SetOutputLimits(double Min, double Max); //clamps the output to a specific range. 0-255 by default, but
										  //it's likely the user will want to change this depending on
										  //the application
	


  //available but not commonly used functions ********************************************************
void PID_SetTunings(double, double,       // * While most users will set the tunings once in the 
                    double);         	  //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
void PID_SetControllerDirection(int);	  // * Sets the Direction, or "Action" of the controller. DIRECT
										  //   means the output will increase when error is positive. REVERSE
										  //   means the opposite.  it's very unlikely that this will be needed
										  //   once it is set in the constructor.
void PID_SetSampleTime(int);              // * sets the frequency, in Milliseconds, with which 
                                          //   the PID calculation is performed.  default is 100
										  
										  
										  
//Display functions ****************************************************************
double PID_GetKp();						  // These functions query the pid for interal values.
double PID_GetKi();						  //  they were created mainly for the pid front-end,
double PID_GetKd();						  // where it's important to know what is actually 
int PID_GetMode();						  //  inside the PID.
int PID_GetDirection();					  //


void PID_Initialize();