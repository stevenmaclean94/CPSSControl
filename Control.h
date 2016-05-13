#ifndef Control_h_
#define Control_h_


class Control
{
public:
	Control();

	float* PDControl(float angleY, float angleZ, float rateY, float rateZ);
	void DeadbandControl(float yControl, float zControl);

	void setPropGain(float PropGain);
	void setDerivGain(float DerivGain);
	void setDeadband(float Deadband);

	float* getThrustMix();
private:
	float ThrustMix[4];
	float deadband;

	// Gains
	float propGain;
	float derivGain;
};
#endif
