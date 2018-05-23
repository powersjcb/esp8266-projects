#define BUFFER_SIZE 20
#define DEFAULT_TIME 1
#define DEFAULT_VALUE 1

class LinearPredictor {
public:
    LinearPredictor(void);
    bool finishedTraining(void);
    void observe(float time, float value);
    float predict(float time);
    static const int bufferSize = BUFFER_SIZE;
    int observationCount = 0;
    float bufferTime[BUFFER_SIZE] = { 0 };
    float bufferValue[BUFFER_SIZE] = { 0 };
    float sumX = DEFAULT_TIME * BUFFER_SIZE;
    float sumX2 = DEFAULT_TIME * DEFAULT_TIME * BUFFER_SIZE;
    float sumY = DEFAULT_VALUE * BUFFER_SIZE;
    float sumY2 = DEFAULT_VALUE * DEFAULT_VALUE * BUFFER_SIZE;
    float sumXY = DEFAULT_VALUE * DEFAULT_TIME * BUFFER_SIZE;
};

