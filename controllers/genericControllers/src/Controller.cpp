#include <genericControllers/Controller.h>

#include <math.h>

using namespace controller;


Controller::Controller()
{
}

Controller::~Controller()
{
}

//Intended to be overwritten by child classes
void Controller::update(){

}

void Controller::init(){

}
