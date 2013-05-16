/** @mainpage
Geometry is expressed to the system as a script. This can be interpreted as the
parametric function x(u), where x is an n-vector, n being the number of axes.
Now u doesn't have to be time. In fact, to avoid physical constraints, u may be
different than time. But u is a function of time. So the idea is to start with
u(t)=t and perturb it until all constraints are met.

The system consists of a computer, a hub, axis controllers, and motors.
- The computer talks to the hub.
- The hub talks to the axis controllers.
- Each axis controller talks to a motor.

The actual physical system is open-loop. From start to end, the computer tells
the motor where to go, but the motor never says back to anything "I got there."
However, as long as certain physical constraints are met, motors should manage
to get where they are told to go. Solving an optimal u as described above is a
hard problem, so the system is simulated and perturbed to find a sufficient u.
In this way feedback is possible, because a simulated motor can say "I might not
have gotten there." So all in all, an open-loop is simulated and in some sense
becomes closed-loop, but once the benefits of feedback are gained, the
parameters are recorded and ready to be sent for an actual, final, physical,
open-loop run.

The specific perturbation done is not entirely important. Some solutions will
work better than others.

Some symbols
- up is the derivative of u. p is for prime.
- upf is the fractional part of up. f is for fractional.
- dupf is the change in upf. d is for delta.
*/

/** @file */

#include <string>
#include <vector>
#include <sstream>

/** @var const unsigned FRACTIONAL_BITS
Fixed point arithmetic is needed because some of this code is supposed to
eventually run on microcontrollers without floating point arithmetic. Here, the
number of bits which are fractional is defined. So, say there is a real number
X which needs to be represented on a microcontroller. We define the integer
variable xf. In pseudocode,

X=xf/2^FRACTIONAL_BITS

Naively, this translates to the following:

X=xf>>FRACTIONAL_BITS

However using this piece of code exactly would result in the fractional bits
being truncated. But say we want to calculate X*Y with integer precision, where
Y is an integer and so can be represented by the integer y in code. Then,

X*Y=(xf/2^FRACTIONAL_BITS)*y

X*Y=(xf>>FRACTIONAL_BITS)*y

X*Y=(xf*y)>>FRACTIONAL_BITS

And now the needed fractional bits come into play before the result is
truncated.

Unity in a fractional number is 1<<FRACTIONAL_BITS.

To convert a fixed point number to a floating point number, we do

floating=float(fixed)/(1<<FRACTIONAL_BITS)

It might be helpful to think that dividing by unity here is like a unit
conversion. Like there are 1000 m in a km, in some sense there are
1<<FRACTIONAL_BITS fixed point numbers in a floating point number.
*/
const unsigned FRACTIONAL_BITS=8;

/**
When perturbing u, it is easiest to modify u'. When a constraint is
violated at time t, u'(t) and its surroundings can be reduced. u'' must not
have infinities; a piecewise linear u' is sufficient. It is also good to know
that
- 0<u' because the geometry should never be traced backward
- u'<1 because u' is initialized to 1 for all t,
	and the path should never be sped up in a perturbation.
So u is stored as a bunch of u' points.
*/
struct UpfPoint{
	UpfPoint(unsigned upf, unsigned t): upf(upf), t(t) {}
	unsigned upf, t;
};

/**
A representation of an actual physical motor. It doesn't have any smarts; the
code here remains here. All it does is respond to ticks and steps and makes a
note if it has gone too fast or accelerated too fast.
*/
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

/**
A representation of an axis controller, corresponding to a motor. Some code in
here is representative of the code that would end up on an actual
microcontroller, hence the fixed point variables.

An axis controller handles two things: movements and timings. Movements are
derived from the geometry defined by the user. They are the pieces of the
stepwise function x(u). Timings are perturbed to fit the geometry and physical
constraints outside the axis controller. They are the pieces of the linear
stepwise function u'(t).

Two similar points:
- A motor will never step more than once per tick because this is physically
	impossible.
- The difference between u before and after a tick will never be more than 1,
	because u'<1.

As such, instead of doing something like

y=f(x);

we can do

if(abs(y-f(x+1))<abs(y-f(x))) ++y;

That is, check which of the closest two available slots is closest.
If it is the next one, increment.

There are three representations of u in this class. u, uT, and uM. u is only for
simulating the system, and it's straightforward. uT and uM increment at the same
time u does, but get reset to 0 when a new timing or movement happens. This
makes calculations easier.
*/
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

/**
A representation of a hub. Some code in here is representative of the code that
would end up on an actual microcontroller. The hub is responsible for receiving
instructions from a computer and then translating them to individual axis
instructions.

The hub is mostly complicated here because it has to allow the movement of the
system to be documented so it can be checked and perturbed if needed. The hub is
oversimple because it doesn't really deal with bandwidth right now, instead just
sending all information to the axes up front.
*/
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

/**
A representation of the computer. Don't be scared by the circularity here. In an
actual physical system, some of this code will be used to interpret more-human
instructions and translate and send them to the hub.
*/
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

/**
A representation of the entire physical system, including computer, hub, axes,
and motors. None of this code is used during the final physical run. It is used
beforehand to predict how the physical system will respond and allows a
successful timing to be perturbed.
*/
class System{
	public:
		/**
		Supply the more-human description of the geometry.
		Call this before calling go.
		All time is measured in ticks and time measured in steps. So a speed is in ticks per step.
		A script goes like this. First, state your axes. This line
		axis <maxSpeed> <maxAcceleration> <frequencyMultiplier>
		declares a single axis; you'll have as many of these lines as you do axes.
		frequencyMultiplier is how much faster this axis's clock goes compared to the hub clock.
		You can specify a maximum actual speed like so:
		maxSpeed <maxSpeed>
		You can do this any time after axes have been declared. It will apply to all following lines.
		To specify a line,
		line <dx1> <dx2> <dx3> ... <dxn>
		Where n is the number of axes, and dxi is the amount axis i should move.
		*/
		std::string setScript(std::string);

		/**
		Find a working timing function for the previously set script,
		run with the final timing,
		and document that run so that the "get" functions can be called.
		*/
		void go(std::vector<float>* tooFasts=NULL);

		/**
		Get position as a function of time.
		Position is a vector of samples in time, each sample is a vector of motor positions.
		If samplePeriod is 0, each sample is given instead of interpolated to ensure a consistent time between samples.
		*/
		void getPosition(std::vector<std::vector<float> >& position, float samplePeriod=0.0f) const;

		/** get u(t) */
		void getU(std::vector<std::vector<float> >& u, float samplePeriod=0.0f) const;

		/** get u'(t) */
		void getUp(std::vector<float>& up, float samplePeriod) const;

		/** Get error as a function of time, axes all pythagoreaned together. */
		void getError(std::vector<float>& error, float samplePeriod) const;

		/**
		Get the minimum max bandwidth required to complete the run.
		That is, the max bandwidth required could be infinity, if everything were sent up front at once, but this is a dumb strategy.
		Even though all commands are sent up front in the simulation,
		we can still know the maximum number required per time interval.
		Infinite memory is assumed, but reducing bandwidth tends to reduce required memory.
		*/
		float getMaxBandwidth();
	private:
		struct Event{
			float t;
			std::vector<int> x;
			unsigned remainingMovements, remainingTimings;
			std::vector<unsigned> u;
		};
		void getMaxSpeed(std::vector<float>& maxSpeed, float samplePeriod) const;
		void simulate();
		void sample(float t);
		Computer computer;
		std::vector<Event> events;
};
