#ifndef SENSORBOX_HPP
#define SENSORBOX_HPP

#include "SensorBase.hpp"

class SensorBox : public SensorBase
{
public:
    SensorBox();
    SensorBox(TFT_eSPI* tft);
    ~SensorBox();
    void sendToDB();
    void getFromDB();
protected:    
    virtual void preSetupState();
    virtual void setupSetup();
    virtual void setupRuntime();
    virtual void setupSleep();
    virtual void setupReset();

    virtual void loopSetup();
    virtual void loopRuntime();
    virtual void loopSleep();
    virtual void loopReset();
};

#endif // DOORSENSOR_HPP