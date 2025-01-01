#include "controller.h"

Controller::Controller() : service()
{
}

void Controller::setup()
{
    service.initService();
}

void Controller::setDoorStatus(bool status)
{
    service.setDoorStatus(status);
}

void Controller::setRingStatus(bool status)
{
    service.setRingStatus(status);
}

boolean Controller::getDoorStatus()
{
    return service.getDoorStatus();
}

void Controller::streamData()
{
    service.syncFirebaseData();
}