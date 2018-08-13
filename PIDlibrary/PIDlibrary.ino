// double lastErr;
double errSum;
double lastInput;
unsigned long lastTime = 0;
double input, output, setpoint;
double kp, ki, kd;
double iTerm = 0;      // use i-term to avoid sudden bumping when ki changes suddently
int sampleTime = 1000; // in millis second
double outMax, outMin;
bool isAuto = false;

#define AUTOMATIC 1
#define MANUAL 0

void myPidCompute()
{
    // turn pid off / on
    if(!isAuto)
        return;


    unsigned long curTime = millis();
    double timeChange = (double)(curTime - lastTime);

    if (timeChange > sampleTime)
    {
        double curErr = setpoint - input;
        iTerm += ki * curErr;
        double dErr = lastInput - input; // dErr/dt = -dInput/dt

        // clamping iTerm
        if (iTerm < outMin)
            iTerm = outMin;
        else if (iTerm > outMax)
            iTerm = outMax;

        // compute pid output
        output = kp * curErr + iTerm + kd * dErr;

        // clamping output
        if (output < outMin)
            output = outMin;
        else if (output > outMax)
            output = outMax;

        // update time, error;
        lastTime = curTime;
        lastInput = input;
    }
}

void setPidConstants(double Kp, double Ki, double Kd)
{
    kp = Kp;
    ki = Ki * sampleTime;
    kd = Kd / sampleTime;
}

void setSampleTimeInMillis(int newSampleTime)
{
    if (newSampleTime > 0)
    {
        ki *= (newSampleTime / sampleTime);
        kd /= (newSampleTime / sampleTime);
        sampleTime = newSampleTime;
    }
}

void setOutputLimits(double min, double max)
{
    if (min > max)
        return;
    
    outMin = min;
    outMax = max;

    // clamping output
    if (output < outMin)
        output = outMin;
    else if (output > outMax)
        output = outMax;

    // clamping iTerm
    if (iTerm < outMin)
        iTerm = outMin;
    else if (iTerm > outMax)
        iTerm = outMax;
}

void setMode(int mode){
    isAuto = mode == AUTOMATIC ? true:false;
}