#include "Control.h"

Control::Control()
{
	// Set some initial gains that are reasonable for our types of systems
	propGain = 0.5f;
	derivGain = 0.35f;
	deadband = 5.0f;
};

//Control Algorithms Avaliable
float* Control::PDControl(float angleY, float angleZ, float rateY, float rateZ)
{
 
  float yFire = -propGain*angleY - derivGain*rateZ;
  float zFire = -propGain*angleZ - derivGain*rateZ;
  // Send Through Deadband Control to Actually Mix the thrusters
  //Control::DeadbandControl(yFire, zFire);
  float control[2];
  control[0] = yFire;
  control[1] = zFire;
  return control;
};
void Control::DeadbandControl(float yControl, float zControl)
{
  // Reset values to 0
  ThrustMix[0] = 0;
  ThrustMix[1] = 0;
  ThrustMix[2] = 0;
  ThrustMix[3] = 0;

  if (yControl > deadband)
  {ThrustMix[0] = 1;}
  else if (yControl<-deadband)
  {ThrustMix[1] = 1;}
  
  if (zControl > deadband)
  {ThrustMix[2] = 1;}
  else if (zControl<-deadband)
  {ThrustMix[3] = 1;}
};

// Tunable Values
void Control::setPropGain(float propGain) {  this->propGain; };
void Control::setDerivGain(float derivGain) {  this->derivGain; };
void Control::setDeadband(float deadband) { this->deadband; };

float* Control::getThrustMix(){return this->ThrustMix;};