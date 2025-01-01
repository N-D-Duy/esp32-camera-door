#ifndef SERVICE_H
#define SERVICE_H

#include <Firebase_ESP_Client.h>

class Service {
public:
    Service();
    void syncFirebaseData();
    void initService();
    void setDoorStatus(bool status);
    boolean getDoorStatus();
    void setRingStatus(bool status);

private:
    boolean _doorStatus;
    boolean _ringStatus;
};

#endif