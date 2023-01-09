#ifndef PID_H
#define PID_H

typedef struct{

    float KP = 0, KI = 0, KD = 0, SetPoint = 0, TimeSampling = 50;
    float min = 0, max = 0;
    float feedback = 0, controlSignal = 0, outputValue = 0;
    float error = 0, lastError = 0, totalError = 0, deltaError = 0;

}PIDParameter_t;

unsigned long currentTime = 0;

class PID{

    private :

    public :

        void setConstrain(PIDParameter_t *pid,float minInput, float maxInput);
        void setPID(PIDParameter_t *pid, float KPInput, float KIInput, float KDInput);
        void setSetPoint(PIDParameter_t *pid, float setPointInput);
        void setTimeSampling(PIDParameter_t *pid, float TimeSamplingInput);
        void setPIDInput(PIDParameter_t *pid, float feedback);

        float getPIDinput(PIDParameter_t *pid);
        float getTimeSampling(PIDParameter_t *pid);
        float getSetPoint(PIDParameter_t *pid);

        float PIDControl(PIDParameter_t *pid);


};

void PID::setConstrain(PIDParameter_t *pid, float min, float max){
    pid->min = min;
    pid->max = max;
}

void PID::setPID(PIDParameter_t *pid, float KP, float KI, float KD){
    pid->KP = KP;
    pid->KI = KI;
    pid->KD = KD;
}

void PID::setSetPoint(PIDParameter_t *pid, float setPoint){\
    pid->SetPoint = setPoint;
}

void PID::setTimeSampling(PIDParameter_t *pid, float TimeSampling){
    pid->TimeSampling = TimeSampling;
}

void PID::setPIDInput(PIDParameter_t *pid, float feedback){
    pid->feedback = feedback;
}

float PID::getPIDinput(PIDParameter_t *pid){ return pid->feedback; };
float PID::getSetPoint(PIDParameter_t *pid){ return pid->SetPoint; };
float PID::getTimeSampling(PIDParameter_t *pid){ return pid->TimeSampling; };

float PID::PIDControl(PIDParameter_t *pid){

    pid->error = pid->SetPoint - pid->feedback;

    pid->totalError += pid->error;

    if(pid->totalError <= pid->min)
        pid->totalError = pid->min;
    else if(pid->totalError >= pid->max)
        pid->totalError = pid->max;

    pid->deltaError = pid->error - pid->lastError;

    pid->controlSignal = ( (pid->KP * pid->error) + (pid->KI * pid->totalError * pid->TimeSampling) + ( (pid->KD / pid->TimeSampling) * pid->deltaError) );
        
    if(pid->controlSignal <= pid->min)
        pid->controlSignal = pid->min;
    else if(pid->totalError >= pid->max)
        pid->controlSignal = pid->max;

    pid->outputValue = pid->controlSignal;
        
    pid->lastError = pid->error;

};

#endif