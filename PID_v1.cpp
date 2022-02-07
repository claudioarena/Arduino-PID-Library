/**********************************************************************************************
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License
 **********************************************************************************************/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <PID_v1.h>

#define IIR_FILTER_D 1
#define IIR_FILTER_OUT 1
#define DEBUG

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, int POn, int ControllerDirection)
{
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    inAuto = false;

    PID::SetOutputLimits(0, 255);				//default output limit corresponds to
												//the arduino pwm limits

    SampleTime = 100*1000L;					//default Controller Sample Time is 0.1 seconds

    PID::SetControllerDirection(ControllerDirection);
    PID::SetTunings(Kp, Ki, Kd, POn);

    lastTime = micros()-SampleTime;
	debugPrint = false;
}

/*Constructor (...)*********************************************************
 *    To allow backwards compatability for v1.1, or for people that just want
 *    to use Proportional on Error without explicitly saying so
 ***************************************************************************/

PID::PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, int ControllerDirection)
    :PID::PID(Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection)
{

}


/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool PID::Compute()
{
   if(!inAuto) return false;
   unsigned long now = micros();
   unsigned long timeChange = (now - lastTime);
   if (timeChange >= SampleTime)
   {
	   /*Compute all the working error variables*/
      double input = *myInput;
	  double error = *mySetpoint - input;	
	  double dInput = (input - lastInput);
	  double filteredInput = alpha_D_IIR * input + (1 - alpha_D_IIR) * lastFilteredInput;
	  double dFilteredInput = (filteredInput - lastFilteredInput);

      outputSum+= (ki * error);

	  /*Add Proportional on Measurement, if P_ON_M is specified*/
	  if (!pOnE) outputSum -= kp * dInput;

      if(outputSum > outMax) outputSum= outMax;
      else if(outputSum < outMin) outputSum= outMin;

      /*Add Proportional on Error, if P_ON_E is specified*/
	   double output;
      if(pOnE) output = kp * error;
      else output = 0;

      /*Compute Rest of PID Output*/
	  output += outputSum - kd * dFilteredInput;

	    if(output > outMax) output = outMax;
      else if(output < outMin) output = outMin;

		double filteredOutput = alpha_OUT_IIR * output+ (1 - alpha_OUT_IIR) * lastFilteredOutput;
		output = filteredOutput;
	    *myOutput = output;

		if (debugPrint) PrintInfo(dInput, dFilteredInput, error, output);

      /*Remember some variables for next time*/
      lastInput = input;
	  lastFilteredInput = filteredInput;
	  lastFilteredOutput = filteredOutput;
      lastTime = now;

	    return true;
   }
   else return false;
}

void PID::PrintInfo(double dInput, double dFilteredInput, double error, double output) {
		double P = 0;
		if (!pOnE) P = kp * dInput;
		else P = kp * error;

		double I = (ki * error);
		double D = kd * dFilteredInput;

		Serial.print(error);
		Serial.print(",");
		Serial.print(dInput);
		Serial.print(",");

		Serial.print(output);
		Serial.print(",");
		Serial.print(P);
		Serial.print(",");
		Serial.print(I);
		Serial.print(",");
		Serial.print(D);
		Serial.print(",");
		Serial.print(dInput);
		Serial.print(",");
		Serial.print(dFilteredInput);
		Serial.println();
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void PID::SetTunings(double Kp, double Ki, double Kd, int POn)
{
   if (Kp<0 || Ki<0 || Kd<0) return;

   pOn = POn;
   pOnE = POn == P_ON_E;

   dispKp = Kp; dispKi = Ki; dispKd = Kd;

   double SampleTimeInSec = ((double)SampleTime)/((double)1000*1000);
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;

  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}

/* SetTunings(...)*************************************************************
 * Set Tunings using the last-rembered POn setting
 ******************************************************************************/
void PID::SetTunings(double Kp, double Ki, double Kd){
    SetTunings(Kp, Ki, Kd, pOn); 
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void PID::SetSampleTime(int NewSampleTime)
{
	if (NewSampleTime > 0)
	{
		double ratio = (double)NewSampleTime
			/ (double)SampleTime;
		ki *= ratio;
		kd /= ratio;
		SampleTime = (unsigned long)NewSampleTime;
		alpha_D_IIR = 1;	//Set IIR filters to have no effect (alpha=1)
		alpha_OUT_IIR = 1;

#ifdef DEBUG
		Serial.print("Sample time set to: ");
		Serial.println(SampleTime);
		Serial.print("New Ki, Kd: ");
		Serial.print(ki);
		Serial.print(", ");
		Serial.println(kd);
#endif
	}
}

/* SetDFilterTime(...) *********************************************************
 * sets the period, in Milliseconds, for the IIR filter
 ******************************************************************************/
void PID::SetDFilterTime(int newIIRFilterTime)
{
#ifdef IIR_FILTER_D
	alpha_D_IIR = (double)SampleTime / newIIRFilterTime;
	if (alpha_D_IIR > 1) alpha_D_IIR = 1;
	if (alpha_D_IIR < 0.001) alpha_D_IIR = 0.001;
#else
	alpha_D_IIR = 1;
#endif

#ifdef DEBUG
	Serial.print("IIR filter D alpha: ");
	Serial.println(alpha_D_IIR);
#endif
}

/* SetOutFilterTime(...) *********************************************************
 * sets the period, in Milliseconds, for the IIR filter
 ******************************************************************************/
void PID::SetOutFilterTime(int newIIRFilterTime)
{
#ifdef IIR_FILTER_OUT
	alpha_OUT_IIR = (double)SampleTime / newIIRFilterTime;
	if (alpha_OUT_IIR > 1) alpha_OUT_IIR = 1;
	if (alpha_OUT_IIR < 0) alpha_OUT_IIR = 0;
#else
		alpha_OUT_IIR = 1;
#endif

#ifdef DEBUG
		Serial.print("IIR filter OUT alpha: ");
		Serial.println(alpha_OUT_IIR);
#endif
}

void PID::toggleDebugPrint()
{
	debugPrint = !debugPrint;
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID::SetOutputLimits(double Min, double Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;

   if(inAuto)
   {
	   if(*myOutput > outMax) *myOutput = outMax;
	   else if(*myOutput < outMin) *myOutput = outMin;

	   if(outputSum > outMax) outputSum= outMax;
	   else if(outputSum < outMin) outputSum= outMin;
   }

#ifdef DEBUG
   Serial.print("New output limits");
   Serial.print(outMin);
   Serial.print(", ");
   Serial.println(outMax);
#endif
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PID::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto && !inAuto)
    {  /*we just went from manual to auto*/
        PID::Initialize();
    }
    inAuto = newAuto;

#ifdef DEBUG
	if(newAuto) Serial.println("Mode now to AUTO");
	else Serial.println("Mode now to MANUAL");
#endif
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PID::Initialize()
{
   outputSum = *myOutput;
   lastInput = *myInput;
   if(outputSum > outMax) outputSum = outMax;
   else if(outputSum < outMin) outputSum = outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID::SetControllerDirection(int Direction)
{
   if(inAuto && Direction !=controllerDirection)
   {
	    kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
   controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double PID::GetKp(){ return  dispKp; }
double PID::GetKi(){ return  dispKi;}
double PID::GetKd(){ return  dispKd;}
int PID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
int PID::GetDirection(){ return controllerDirection;}

