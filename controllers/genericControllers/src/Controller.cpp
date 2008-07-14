#include <genericControllers/Controller.h>

#include <math.h>

using namespace CONTROLLER;


Controller::Controller()
{
}

Controller::~Controller()
{
}

//Intended to be overwritten by child classes
void Controller::Update(){

}

void Controller::Init(){

}
