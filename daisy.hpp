/*
The idea
Geometry is expressed to the system as a script. This can be interpreted as the
parametric function x(u), where x is an n-vector, n being the number of axes.
Now u doesn't have to be time. In fact, to avoid physical constraints, u will
necessarily be different than time. But it itself is a function of time. So the
idea is to start with u(t)=t and perturb it until all constraints are met.

Some symbols
up is the derivative of u. p is for prime.
upf is the fractional part of up. f is for fractional.
dupf is the change in upf.
*/

#include <string>
#include <vector>
#include <sstream>

const unsigned FRACTIONAL_BITS=8;

struct UpfPoint{
	UpfPoint(unsigned upf, unsigned t): upf(upf), t(t) {}
	unsigned upf, t;
};

class Motor{
	public:
		Motor(float maxSpeed, float maxAcceleration);
		void step(bool positive);
		void tick();
		void reset();
		float readMaxSpeed() const;
		float readMaxAcceleration() const;
		int readX() const;
		void readFasts(std::vector<unsigned>& wentTooFast, std::vector<unsigned>& acceleratedTooFast) const;
	private:
		float maxSpeed, maxAcceleration;
		int x;
		unsigned ticksSinceLastStep, ticksBetweenPreviousSteps;
		int signLastStep;
		unsigned t;
		std::vector<unsigned> wentTooFast, acceleratedTooFast;
};

class Axis{
	public:
		Axis(float maxSpeed, float maxAcceleration, unsigned frequencyMultipler);
		void setMovement(int dx, unsigned du);
		void setTiming(int dupf, unsigned dt);
		bool tick();//return true if a step happened
		void reset();
		bool readFinished();
		unsigned readFrequencyMultiplier() const;
		const Motor& readMotor() const;
		unsigned readRemainingMovements() const;
		unsigned readRemainingTimings() const;
		unsigned readU() const;
	private:
		void nextMovement();
		void nextTiming();
		//per axis
		Motor motor;
		unsigned frequencyMultiplier, u;
		std::vector<int> upcomingDx, upcomingDupf;
		std::vector<unsigned> upcomingDu, upcomingDt;
		bool finished;
		//per movement
		unsigned x, dx, uM, du;
		bool dxPositive;
		//per timing
		unsigned uT, upif, dupf, ticks, dt;
		int dupfSign;
};

class Hub{
	public:
		Hub();
		void addAxis(
			float maxSpeed, float maxAcceleration, unsigned frequencyMultiplier
		);
		void setMovement(const std::vector<int>& dx, unsigned du);
		void setTiming(int dupf, unsigned dt);
		float subTick();//returns number of hub ticks that pass
		void reset();
		const Axis& readAxis(unsigned i) const;
		unsigned numberOfAxes() const;
	private:
		std::vector<Axis> axes;
		unsigned subTicks, subTicksPerTick;
};

class Computer{
	public:
		std::string setScript(std::string);
		void setTiming(const std::vector<UpfPoint>& upf);
		void sendToHub();
		float subTick();//returns number of hub ticks that pass
		void getTheoreticalPosition(float u, unsigned i, float& x) const;
		float getMaxSpeed(float u) const;
		void reset();
		const Hub& readHub() const;
		const std::vector<UpfPoint>& readUpf() const{ return upf; }
	private:
		class Segment{
			public:
				std::string become(const std::string&, float maxSpeed, unsigned axes);
				const std::vector<int>& readX() const;
				float readMaxSpeed() const{ return maxSpeed; }
			private:
				std::vector<int> x;
				float maxSpeed;
		};
		std::vector<Segment> segments;
		Hub hub;
		std::vector<UpfPoint> upf;
};

class System{
	public:
		std::string setScript(std::string);
		void go(std::vector<float>* tooFasts=NULL);
		void getPosition(std::vector<std::vector<float> >& position, float samplePeriod=0.0f) const;
		void getU(std::vector<std::vector<float> >& u, float samplePeriod=0.0f) const;
		void getUp(std::vector<float>& up, float samplePeriod) const;
		void getError(std::vector<float>& error, float samplePeriod) const;
		float getMaxBandwidth();
		void getMaxSpeed(std::vector<float>& maxSpeed, float samplePeriod) const;
	private:
		struct Event{
			float t;
			std::vector<int> x;
			unsigned remainingMovements, remainingTimings;
			std::vector<unsigned> u;
		};
		void simulate();
		void sample(float t);
		Computer computer;
		std::vector<Event> events;
};
