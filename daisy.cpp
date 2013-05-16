#include "daisy.hpp"

#include <cmath>
#include <iostream>
#include <fstream>
#include <algorithm>

using namespace std;

unsigned uabs(int x){ return x<0?-x:x; }

float interpolate(float a, float b, float bness){ return a*(1-bness)+b*bness; }

//map [xi, xf] to [yi, yf]
float linear(float x, float xi, float xf, float yi, float yf){
	return (x-xi)/(xf-xi)*(yf-yi)+yi;
}

//=====Motor=====//
Motor::Motor(float maxSpeed, float maxAcceleration):
	maxSpeed(maxSpeed), maxAcceleration(maxAcceleration), signLastStep(1)
{
	reset();
}

void Motor::step(bool positive){
	//figure the motor's speed between now and the last step and check if it's too fast
	if(ticksSinceLastStep!=0&&1.0f/ticksSinceLastStep>maxSpeed)
		wentTooFast.push_back(t);
	//figure the motor's acceleartion between now and the last last step and check if it's too much
	if(
		ticksBetweenPreviousSteps!=0
		&&
		abs(signLastStep*1.0f/ticksBetweenPreviousSteps-(positive?1:-1)*1.0f/ticksSinceLastStep)
		>
		maxAcceleration
	)
		acceleratedTooFast.push_back(t);
	//step and document
	ticksBetweenPreviousSteps=ticksSinceLastStep;
	ticksSinceLastStep=0;
	positive?++x:--x;
	signLastStep=positive?1:-1;
}

void Motor::tick(){
	++ticksSinceLastStep;
	++t;
}

void Motor::reset(){
	x=0;
	ticksSinceLastStep=0;
	ticksBetweenPreviousSteps=0;
	wentTooFast.clear();
	acceleratedTooFast.clear();
	t=0;
}

float Motor::readMaxSpeed() const{ return maxSpeed; }
float Motor::readMaxAcceleration() const{ return maxAcceleration; }
int Motor::readX() const{ return x; }

void Motor::readFasts(vector<unsigned>& _wentTooFast, vector<unsigned>& _acceleratedTooFast) const{
	_wentTooFast=wentTooFast;
	_acceleratedTooFast=acceleratedTooFast;
}

//=====Axis=====//
Axis::Axis(float maxSpeed, float maxAcceleration, unsigned frequencyMultiplier):
	motor(maxSpeed, maxAcceleration), frequencyMultiplier(frequencyMultiplier)
{
	reset();
}

void Axis::setMovement(int _dx, unsigned _du){
	upcomingDx.push_back(_dx);
	upcomingDu.push_back(_du);
}

void Axis::setTiming(int _dupf, unsigned _dt){
	upcomingDupf.push_back(_dupf);
	upcomingDt.push_back(_dt);
}

bool Axis::tick(){
	bool stepped=false;
	//update t
	if(ticks==dt*frequencyMultiplier) nextTiming();
	++ticks;
	//update u
	if(uM==du) nextMovement();
	const unsigned f=frequencyMultiplier;//just make stuff shorter so it's easier to see
	const unsigned b=FRACTIONAL_BITS;//just make stuff shorter so it's easier to see
	if(
		uabs(int(2*dt*(uT+1)*f*f-dupfSign*(dupf*ticks*ticks>>b)-(2*dt*upif*ticks>>b)*f))
		<               //this is the only difference!
		uabs(int(2*dt*(uT+0)*f*f-dupfSign*(dupf*ticks*ticks>>b)-(2*dt*upif*ticks>>b)*f))
	){
		++uT;
		++uM;
		++u;
	}
	//update x
	motor.tick();
	if(
		uabs(int((x+1)*du-dx*uM))
		<         //this is the only difference!
		uabs(int((x+0)*du-dx*uM))
	){
		motor.step(dxPositive);
		++x;
		stepped=true;
	}
	//done
	return stepped;
}

void Axis::reset(){
	motor.reset();
	u=0;
	upcomingDx.clear();
	upcomingDupf.clear();
	upcomingDu.clear();
	upcomingDt.clear();
	finished=false;
	uM=0;
	du=0;
	upif=0;
	dupf=0;
	ticks=0;
	dt=0;
}

bool Axis::readFinished(){ return finished; }
unsigned Axis::readFrequencyMultiplier() const{ return frequencyMultiplier; }
const Motor& Axis::readMotor() const{ return motor; }
unsigned Axis::readRemainingMovements() const{ return upcomingDx.size(); }
unsigned Axis::readRemainingTimings() const{ return upcomingDu.size(); }
unsigned Axis::readU() const{ return u; }

void Axis::nextMovement(){
	//reset
	x=0;
	uM=0;
	//if there are no more movements, don't move
	if(!upcomingDx.size()){
		dx=0;
		du=1;
		finished=true;
		return;
	}
	//if there are more movements, get the next one
	dx=uabs(upcomingDx[0]);
	du=upcomingDu[0];
	dxPositive=upcomingDx[0]>=0?true:false;
	//and erase it from the list
	upcomingDx.erase(upcomingDx.begin());
	upcomingDu.erase(upcomingDu.begin());
}

void Axis::nextTiming(){
	//prepare for next
	uT=0;
	ticks=0;
	upif+=dupfSign*dupf;
	//if there are no more timings, just do u'(t)=1
	if(!upcomingDupf.size()){
		dupf=0;
		dt=1<<FRACTIONAL_BITS;
		return;
	}
	//if there are more timings, get the next one
	dupf=uabs(upcomingDupf[0]);
	dt=upcomingDt[0];
	dupfSign=upcomingDupf[0]>=0?1:-1;
	//and erase it from the list
	upcomingDupf.erase(upcomingDupf.begin());
	upcomingDt.erase(upcomingDt.begin());
}

//=====Hub=====//
Hub::Hub(): subTicksPerTick(1) { reset(); }

void Hub::addAxis(
	float maxSpeed, float maxAcceleration, unsigned frequencyMultiplier
){
	axes.push_back(Axis(maxSpeed, maxAcceleration, frequencyMultiplier));
	//make sure factors(frequencyMultiplier) is a subset of factors(subTicksPerTick)
	if(subTicksPerTick%frequencyMultiplier!=0){
		unsigned r=subTicksPerTick;
		for(unsigned factor=2; frequencyMultiplier!=1; factor==2?factor=3:factor+=2){
			bool factorFound;
			do{
				factorFound=false;
				if(frequencyMultiplier%factor==0){
					factorFound=true;
					frequencyMultiplier/=factor;
					if(r%factor==0) r/=factor;
					else subTicksPerTick*=factor;
					if(frequencyMultiplier==1) break;
				}
			} while(factorFound);
		}
	}
}

void Hub::setMovement(const vector<int>& dx, unsigned du){
	for(unsigned i=0; i<axes.size(); ++i) axes[i].setMovement(dx[i], du);
}

void Hub::setTiming(int dupf, unsigned dt){
	for(unsigned i=0; i<axes.size(); ++i) axes[i].setTiming(dupf, dt);
}

float Hub::subTick(){
	unsigned previousSubTicks=subTicks;
	bool stepHappened=false;
	//tick axes until a step happens
	while(!stepHappened){
		bool allFinished=true;
		//tick axes and note whether they stepped or finished
		for(unsigned i=0; i<axes.size(); ++i){
			allFinished&=axes[i].readFinished();
			if(subTicks%(subTicksPerTick/axes[i].readFrequencyMultiplier())==0)
				if(axes[i].tick()) stepHappened=true;
		}
		//if all have finished, the run is done
		if(allFinished) return 0.0f;
		//increment
		++subTicks;
	}
	//measure how much time has passed
	float result=1.0f*(subTicks-previousSubTicks)/subTicksPerTick;
	//keep this from overflowing
	subTicks%=subTicksPerTick;
	//finish
	return result;
}

void Hub::reset(){
	for(unsigned i=0; i<axes.size(); ++i) axes[i].reset();
	subTicks=0;
}

const Axis& Hub::readAxis(unsigned i) const{ return axes[i]; }
unsigned Hub::numberOfAxes() const{ return axes.size(); }

//=====Computer=====//
string toString(int i){
	stringstream ss;
	ss<<i;
	return ss.str();
}

string Computer::setScript(string script){
	segments.clear();
	hub=Hub();
	//initialize parsing stuff
	stringstream scriptStream(script);
	string line;
	int lineNumber=1;
	bool inBody=false;
	float maxSpeed=0.0f;
	//parse
	while(getline(scriptStream, line)){
		//get a token
		stringstream lineStream(line);
		string token;
		lineStream>>token;
		//see what the token is and process the rest of the line
		if(token=="axis"){
			if(inBody) return
				"Error on line "+
				toString(lineNumber)+
				": Must declare axes before body.";
			float axisMaxSpeed, axisMaxAcceleration;
			unsigned frequencyMultiplier;
			lineStream>>axisMaxSpeed>>axisMaxAcceleration;
			if(!(lineStream>>frequencyMultiplier)) return
				"Error on line "+toString(lineNumber)+": Missing axis parameters.";
			hub.addAxis(axisMaxSpeed, axisMaxAcceleration, frequencyMultiplier);
		}
		else if(token=="line"){
			inBody=true;
			segments.resize(segments.size()+1);
			string error=segments.back().become(line, maxSpeed, hub.numberOfAxes());
			if(error!="") return "Error on line "+toString(lineNumber)+": "+error;
		}
		else if(token=="maxSpeed") lineStream>>maxSpeed;
		//if it's not something we know, it's an error
		else
			return "Error on line "+toString(lineNumber)+": unknown specification.";
		//increment
		++lineNumber;
	}
	//done
	return "";
}

void Computer::setTiming(const vector<UpfPoint>& _upf){ upf=_upf; }

void Computer::sendToHub(){
	//send movements
	for(unsigned i=0; i<segments.size(); ++i){
		//Determine how much u this movement gets. Ensure no more than one step per tick is required.
		int maxX=0;
		for(unsigned j=0; j<segments[i].readX().size(); ++j)
			maxX=max(maxX, abs(segments[i].readX()[j]));
		//send
		hub.setMovement(segments[i].readX(), maxX);
	}
	//send timings
	for(unsigned i=1; i<upf.size(); ++i)
		hub.setTiming(upf[i].upf-upf[i-1].upf, upf[i].t-upf[i-1].t);
}

float Computer::subTick(){ return hub.subTick(); }

void Computer::getTheoreticalPosition(float u, unsigned i, float& x) const{
	x=0.0f;
	float previous=x;
	unsigned j, maxX=0;
	//figure out what segment u goes into
	for(j=0; j<segments.size(); ++j){
		//keep track of theoretical position
		previous=x;
		x+=segments[j].readX()[i];
		//figure out how much u is allocated to this segment
		maxX=0;
		for(unsigned k=0; k<segments[j].readX().size(); ++k)
			maxX=max(maxX, uabs(segments[j].readX()[k]));
		//if we're beyond this segment
		if(u>maxX){
			u-=maxX;//decrease u accordingly and carry on
			if(j==segments.size()-1) return;//if u is not in a segment, return the final location
		}
		//otherwise, this is the segment
		else break;
	}
	//interpolate between start and end of that segment
	x=linear(u, 0, maxX, previous, x);
}

float Computer::getMaxSpeed(float u) const{
	unsigned i;
	//figure out what segment u goes into
	for(i=0; i<segments.size(); ++i){
		//figure out how much u is allocated to this segment
		unsigned maxX=0;
		for(unsigned j=0; j<segments[i].readX().size(); ++j)
			maxX=max(maxX, uabs(segments[i].readX()[j]));
		//if we're beyond this segment
		if(u>maxX){
			u-=maxX;//decrease u accordingly and carry on
			if(i==segments.size()-1) break;//if u is not in a segment, use the last segment
		}
		//otherwise, this is the segment
		else break;
	}
	//return the max speed of the segment
	return segments[i].readMaxSpeed();
}

void Computer::reset(){
	hub.reset();
	sendToHub();
}

const Hub& Computer::readHub() const{ return hub; }

string Computer::Segment::become(const string& line, float _maxSpeed, unsigned axes){
	maxSpeed=_maxSpeed;
	x.resize(axes);
	stringstream ss(line);
	string s;
	ss>>s;
	if(s=="line"){
		for(unsigned i=0; i<axes; ++i)
			if(!(ss>>x[i])) return "Not enough dimensions on line specification.";
	}
	else return "Unknown segment type.";
	return "";
}

const vector<int>& Computer::Segment::readX() const{ return x; }

//=====System=====//
string System::setScript(string script){ return computer.setScript(script); }

//find a point near t
//if there isn't one, insert one
//return the index of that point
int insertIfNoneNear(float t, vector<UpfPoint>& upf){
	//this shouldn't happen!
	if(t<0) return -1;
	//find the nearest previous point, and try to find a near point
	int index=-1;
	unsigned previous=0;
	float difference=1.0f;//this defines what near is
	for(unsigned i=0; i<upf.size(); ++i){
		if(abs(t-upf[i].t)<difference){
			index=i;
			difference=abs(t-upf[i].t);
		}
		if(upf[i].t<t) previous=i;
	}
	//insert if not found
	if(index<0){
		//we desire the upf at this place
		unsigned upfHere;
		//get the previous upf value
		unsigned upfPrevious=upf[previous].upf;
		//get the next upf value
		if(previous+1>=upf.size()) upfHere=upfPrevious;
		else{
			unsigned upfNext=upf[previous+1].upf;
			upfHere=(unsigned)interpolate(1.0f*upfPrevious, 1.0f*upfNext, 1.0f*(t-upf[previous].t)/(upf[previous+1].t-upf[previous].t));
		}
		//interpolate and insert
		upf.insert(upf.begin()+previous+1, UpfPoint(upfHere, (unsigned)t));
		//set the index accordingly
		index=previous+1;
	}
	//return index of point
	return index;
}

//given a coverage and a upf, dip upf at t if it's not covered
//a dip consists of the following:
//upf near t is reduced by a factor, then
//upf[T] where t-dt<T<t is ensured to be less than the interpolation between upf[t-dt] and upf[t], and
//upf[T] where t<T<t+dt is ensured to be less than the interpolation between upf[t] and upf[t+dt]
void dip(vector<UpfPoint>& upf, float t, vector<float>& coverage){
	const float dt=128.0f;//empirically chosen
	//check if this spot has been covered already
	for(unsigned i=0; i<coverage.size(); ++i) if(abs(t-coverage[i])<dt) return;
	//set this spot as covered
	coverage.push_back(t);
	//ensure enough points to create a dip
	int leftIndex=insertIfNoneNear(t-dt, upf);
	int rightIndex=insertIfNoneNear(t+dt, upf);
	int index=insertIfNoneNear(t, upf);
	//get all points in the dip
	vector<unsigned> indices;
	for(unsigned i=0; i<upf.size(); ++i)
		if(abs(t-upf[i].t)<dt)
			indices.push_back(i);
	//make the dip
	float ti=t-dt;//t initial
	float tf=t+dt;//t final
	upf[index].upf/=2;//decrease upf near t by a factor
	unsigned lowUpf=upf[index].upf;//get upf near t
	unsigned leftHighUpf=1<<FRACTIONAL_BITS, rightHighUpf=1<<FRACTIONAL_BITS;//set upf near t-dt and t+dt to unity as a default value
	if(leftIndex>=0) leftHighUpf=upf[leftIndex].upf;//if a left point was found, use it
	if(rightIndex>=0) rightHighUpf=upf[rightIndex].upf;//if a right point was found, use it
	for(unsigned i=0; i<indices.size(); ++i){//for all points in the dip
		float th=1.0f*upf[indices[i]].t;//calculate t at this point (th="t here")
		//interpolate to find the maximum allowable upf
		unsigned maxUpf;
		if(th>t)
			maxUpf=(unsigned)linear(th, t, tf, 1.0f*lowUpf, 1.0f*rightHighUpf);
		else
			maxUpf=(unsigned)linear(th, ti, t, 1.0f*leftHighUpf, 1.0f*lowUpf);
		//reduce upf here if needed
		upf[indices[i]].upf=min(upf[indices[i]].upf, maxUpf);
	}
}

void System::go(vector<float>* tooFasts){
	//start with u'(t)=1, or nearly so anyway -- the first upf point has to be (0, 0)
	vector<UpfPoint> upf;
	upf.push_back(UpfPoint(0, 0));
	upf.push_back(UpfPoint(1<<FRACTIONAL_BITS, 1));
	//figure out a good timing
	while(true){
		//run
		computer.setTiming(upf);
		simulate();
		//initialize perturbation stuff
		bool good=true;
		vector<float> coverage;
		if(tooFasts) tooFasts->clear();
		//motor constraints
		for(unsigned i=0; i<computer.readHub().numberOfAxes(); ++i){
			vector<unsigned> wentTooFast, acceleratedTooFast;
			computer.readHub().readAxis(i).readMotor().readFasts(wentTooFast, acceleratedTooFast);
			//perturb for motors that went too fast
			for(int j=wentTooFast.size()-1; j>=0; --j){
				float t=1.0f*wentTooFast[j]/computer.readHub().readAxis(i).readFrequencyMultiplier();
				if(tooFasts) tooFasts->push_back(t);
				dip(upf, t, coverage);
				good=false;
			}
			//perturb for motors that accelerated too fast
			for(int j=acceleratedTooFast.size()-1; j>=0; --j){
				float t=1.0f*acceleratedTooFast[j]/computer.readHub().readAxis(i).readFrequencyMultiplier();
				if(tooFasts) tooFasts->push_back(t);
				dip(upf, t, coverage);
				good=false;
			}
		}
		//get the position and max speed
		const float samplePeriod=1.0f;//sample period chosen empirically
		vector<vector<float> > position;
		getPosition(position, samplePeriod);
		vector<float> maxSpeed;
		getMaxSpeed(maxSpeed, samplePeriod);
		//figure out the speed based on the position and if it's above the max speed, perturb
		for(unsigned i=position.size()-1; i>=1; --i){
			//figure the speed
			float speed=0.0f;
			for(unsigned j=0; j<position[i].size(); ++j){
				float d=position[i][j]-position[i-1][j];
				speed+=d*d;
			}
			speed=sqrt(speed);
			//perturb if it's above the max speed
			if((i<maxSpeed.size()&&speed>maxSpeed[i])||speed>maxSpeed.back()){
				float t=1.0f*i*samplePeriod;
				if(tooFasts) tooFasts->push_back(t);
				dip(upf, t, coverage);
				good=false;
			}
		}
		//prepare for finishing, and finish if the run was successful
		if(tooFasts) sort(tooFasts->begin(), tooFasts->end());
		if(good) break;
	}
	//do the real run
	simulate();
}

void System::getPosition(vector<vector<float> >& position, float samplePeriod) const{
	unsigned sample=0, i=1;
	while(true){
		//go to the event after the current sample time, or finish if it's beyond the last event
		float t;
		if(samplePeriod<=0.0f) t=events[sample].t;
		else t=sample*samplePeriod;
		while(i<events.size()&&events[i].t<=t) ++i;
		if(i>=events.size()) break;
		//interpolate between previous event and this one
		position.resize(position.size()+1);
		for(unsigned j=0; j<events[i].x.size(); ++j){
			float s=(t-events[i-1].t)/(events[i].t-events[i-1].t);
			position.back().push_back(interpolate(1.0f*events[i-1].x[j], 1.0f*events[i].x[j], s));
		}
		//next
		++sample;
	}
}

void System::getU(vector<vector<float> >& u, float samplePeriod) const{
	unsigned sample=0, i=1;
	while(true){
		//go to the event after the current sample time, or finish if it's beyond the last event
		float t;
		if(samplePeriod<=0.0f) t=events[sample].t;
		else t=sample*samplePeriod;
		while(i<events.size()&&events[i].t<=t) ++i;
		if(i>=events.size()) break;
		//interpolate between previous event and this one
		u.resize(u.size()+1);
		for(unsigned j=0; j<events[i].x.size(); ++j){
			float s=(t-events[i-1].t)/(events[i].t-events[i-1].t);
			u.back().push_back(interpolate(1.0f*events[i-1].u[j], 1.0f*events[i].u[j], s));
		}
		//next
		++sample;
	}
}

void System::getUp(vector<float>& up, float samplePeriod) const{
	unsigned sample=0, i=1;
	while(true){
		//go to the event after the current sample time, or finish if it's beyond the last event
		float t=sample*samplePeriod;
		while(i<computer.readUpf().size()&&computer.readUpf()[i].t<=t) ++i;
		if(t>=events.back().t) break;
		//if we're beyond the last upf point, use the last upf point
		if(i==computer.readUpf().size())
			up.push_back(1.0f*computer.readUpf().back().upf/(1<<FRACTIONAL_BITS));
		//otherwise interpolate between previous event and this one
		else{
			float s=(t-computer.readUpf()[i-1].t)/(computer.readUpf()[i].t-computer.readUpf()[i-1].t);
			up.push_back(interpolate(1.0f*computer.readUpf()[i-1].upf, 1.0f*computer.readUpf()[i].upf, s)/(1<<FRACTIONAL_BITS));
		}
		//next
		++sample;
	}
}

void System::getError(vector<float>& error, float samplePeriod) const{
	//get the position
	vector<vector<float> > actualPosition;
	getPosition(actualPosition, samplePeriod);
	//get u
	vector<vector<float> > u;
	getU(u, samplePeriod);
	//get the theoretical position with u, and compare it to the position
	for(unsigned i=0; i<actualPosition.size(); ++i){
		float e2=0.0f;
		//pythagoras all the differences together
		for(unsigned j=0; j<actualPosition[i].size(); ++j){
			float theoreticalPosition;
			computer.getTheoreticalPosition(u[i][j], j, theoreticalPosition);
			float d=actualPosition[i][j]-theoreticalPosition;
			e2+=d*d;
		}
		//stick in the result
		error.push_back(sqrt(e2));
	}
}

float System::getMaxBandwidth(){
	//start with 0 because that's the answer if there's nothing that ever happens
	float m=0.0f;
	//this is how many commands need to be sent throughout the entire run
	unsigned initial=events[0].remainingMovements+events[0].remainingTimings;
	//go through the run and keep track of max bandwidth
	for(unsigned i=0; i<events.size(); ++i){
		//divide the number of commands that were necessarily send by the current time
		float b=(initial-(events[i].remainingMovements+events[i].remainingTimings))/events[i].t;
		//record it if it's the biggest so far
		m=max(m, b);
	}
	//finish
	return m;
}

void System::getMaxSpeed(std::vector<float>& maxSpeed, float samplePeriod) const{
	vector<float> up;
	getUp(up, samplePeriod);//u' is used over u because the recorded u is different for each axis
	float u=0.0f;
	for(unsigned i=1; i<up.size(); ++i){
		maxSpeed.push_back(computer.getMaxSpeed(u));
		u+=(up[i]+up[i-1])/2*samplePeriod;//increase u by the average u' value over the sample period multiplied by the sample period
	}
}

void System::simulate(){
	events.clear();
	computer.reset();
	float t=0.0f;
	sample(t);
	while(true){
		float dt=computer.subTick();
		if(dt<=0.0f) break;
		t+=dt;
		sample(t);
	}
}

void System::sample(float t){
	Event e;
	e.t=t;
	for(unsigned i=0; i<computer.readHub().numberOfAxes(); ++i){
		e.x.push_back(computer.readHub().readAxis(i).readMotor().readX());
		e.u.push_back(computer.readHub().readAxis(i).readU());
	}
	e.remainingMovements=computer.readHub().readAxis(0).readRemainingMovements();
	e.remainingTimings=computer.readHub().readAxis(0).readRemainingTimings();
	events.push_back(e);
}
