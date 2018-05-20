struct RollingLinearRegression {
    bool finishedTraining(void);
    int bufferSize;
    int observationCount = 0;
    float bufferTime[];
    float bufferValue[];
    float sumX;
    float sumX2;
    float sumY;
    float sumY2;
    float sumXY;
    RollingLinearRegression(int size);
    void observe(unsigned long t, float value);
    float predict(unsigned long t);

};
