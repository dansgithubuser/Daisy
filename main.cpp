#include "daisy.hpp"

#include <SFML/System.hpp>
#include <SFML/Window.hpp>
#include <SFML/OpenGL.hpp>

#include <string>
#include <fstream>
#include <iostream>

using namespace std;

int main(){
	ofstream cerrFile("stderr.txt");
	cerr.rdbuf(cerrFile.rdbuf());
	//graphics initialization
	sf::Window window(sf::VideoMode(800, 600, 32), "Daisy");
	glClearColor(0.f, 0.f, 0.f, 0.f);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(
		90.0f,
		1.0f*window.getSize().x/window.getSize().y,
		1.0f,
		500.0f
	);
	//daisy
	System system;
	string error=system.setScript(
		"axis 0.8 10.5 1\n"
		"axis 1.5 10.5 1\n"
		"maxSpeed 2.0\n"
		"line 100 0\n"
		"line 0 -80\n"
		"line 20 0\n"
		"line 0 80\n"
		"line 20 0\n"
		"line 60 -80\n"
		"line 0 80\n"
		"line 20 0\n"
		"line 0 -100\n"
		"line -20 0\n"
		"line -60 80\n"
		"line 0 -80\n"
		"line -80 0\n"
		"line 0 20\n"
		"line 20 0\n"
		"line 0 60\n"
		"line -60 0\n"
		"line 0 -60\n"
		"line 20 0\n"
		"line 0 -20\n"
		"line -280 0\n"
		"line 0 20\n"
		"line 80 60\n"
		"line -80 0\n"
		"line 0 20\n"
		"line 100 0\n"
		"line 0 -20\n"
		"line -80 -60\n"
		"line 100 0\n"
		"line 0 80\n"
		"line 20 0\n"
		"line 0 -80\n"
		"line 60 0\n"
		"line 0 80\n"
		"line 20 0\n"
		"line 0 -80\n"
		"line 20 0\n"
		"line 0 80\n"
	);
	std::vector<float> tooFasts;
	system.go(&tooFasts);
	const float samplePeriod=1.0f;
	vector<vector<float> > path;
	system.getPosition(path, samplePeriod);
	std::vector<float> up;
	system.getUp(up, samplePeriod);
	std::vector<float> errors;
	system.getError(errors, samplePeriod);
	//loop
	unsigned t=0;
	sf::Clock clock;
	bool dragging=false;
	float yaw=0.0f, pitch=0.0f;
	int previousMouseX=0, previousMouseY=0;
	while(window.isOpen()){
		sf::Event sfEvent;
		while(window.pollEvent(sfEvent))
			switch(sfEvent.type){
				case sf::Event::Closed: window.close(); break;
				case sf::Event::Resized:
					glViewport(0, 0, sfEvent.size.width, sfEvent.size.height);
					glMatrixMode(GL_PROJECTION);
					glLoadIdentity();
					gluPerspective(90.0f, 1.0f*sfEvent.size.width/sfEvent.size.height, 1.0f, 500.0f);
					break;
				case sf::Event::KeyPressed:
					if(sfEvent.key.code==sf::Keyboard::Q) window.close();
					break;
				case sf::Event::MouseButtonPressed:
					if(sfEvent.mouseButton.button==sf::Mouse::Left){
						dragging=true;
						previousMouseX=sfEvent.mouseButton.x;
						previousMouseY=sfEvent.mouseButton.y;
					}
					break;
				case sf::Event::MouseButtonReleased:
					if(sfEvent.mouseButton.button==sf::Mouse::Left) dragging=false;
					break;
				case sf::Event::MouseMoved:
					if(dragging){
						yaw  +=360.0f*(sfEvent.mouseMove.x-previousMouseX)/window.getSize().x;
						pitch+=360.0f*(sfEvent.mouseMove.y-previousMouseY)/window.getSize().y;
						if(yaw<0.0f) yaw+=360.0f;
						else if(yaw>=360.0f) yaw-=360.0f;
						if(pitch<-90.0f) pitch=-90.0f;
						else if(pitch>90.0f) pitch=90.0f;
						previousMouseX=sfEvent.mouseMove.x;
						previousMouseY=sfEvent.mouseMove.y;
					}
					break;
				default: break;
			}
		//transform
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glTranslatef(0.0f, 0.0f, -200.0f);
		glRotatef(yaw, 0.0f, 1.0f, 0.0f);
		glRotatef(pitch, 1.0f, 0.0f, 0.0f);
		//draw
		glClear(GL_COLOR_BUFFER_BIT);
		glBegin(GL_LINE_STRIP);
		for(unsigned i=0, j=0; i<path.size(); ++i){
			float r=1.0f, g=1.0f, b=1.0f;
			if(abs(int(i)-int(t))<1+path.size()/600) g=b=0.0f;
			else while(j<tooFasts.size()&&i>=tooFasts[j]){
				g=0.0f;
				++j;
			}
			glColor3f(r, g, b);
			glVertex3f(path[i][0], path[i][1], 0.0f);
		}
		glEnd();
		//transform
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		//draw up
		glBegin(GL_LINE_STRIP);
		glColor3f(0.0f, 0.0f, 1.0f);
		glVertex3f(-250.0f, -0.0f, -200.0f);
		glVertex3f(-250.0f, -150.0f, -200.0f);
		glVertex3f(-150.0f, -150.0f, -200.0f);
		glEnd();
		glBegin(GL_LINE_STRIP);
		for(unsigned i=0, j=0; i<up.size(); ++i){
			float r=1.0f, g=1.0f, b=0.0f;
			if(abs(int(i)-int(t))<1+path.size()/600) g=0.0f;
			else while(j<tooFasts.size()&&i>=tooFasts[j]){
				g=0.0f;
				b=1.0f;
				++j;
			}
			glColor3f(r, g, b);
			glVertex3f(i*500/path.size()-250.0f, 100*up[i]-150.0f, -200.0f);
		}
		glEnd();
		//draw error
		glBegin(GL_LINE_STRIP);
		for(unsigned i=0; i<errors.size(); ++i){
			float r=1.0f, g=0.0f, b=0.0f;
			glColor3f(r, g, b);
			glVertex3f(i*500/path.size()-250.0f, 100*errors[i]-150.0f, -200.0f);
		}
		glEnd();
		//increase time
		t+=1+path.size()/600;
		if(t>=path.size()) t=0;
		window.display();
		//give processor a break
		sf::sleep(sf::seconds(1/60.0f));
	}
	//finish
	return 0;
}
