//
// Created by Jacob Powers on 5/19/18.
//

#include "RollingLinearRegression.h"

RollingLinearRegression::RollingLinearRegression(int size) {
    float defaultTime = 1.0;
    float defaultValue = 1.0;
    bufferSize = size;
    bufferTime[bufferSize] = {defaultTime};
    bufferValue[bufferSize] = {defaultValue};

    sumX = defaultTime * size;
    sumX2 = defaultTime * defaultTime * size;
    sumY = defaultValue * size;
    sumY2 = defaultValue * defaultValue * size;
    sumXY = defaultTime * defaultValue * size;
}

bool RollingLinearRegression::finishedTraining() {
    return observationCount > bufferSize;
}

void RollingLinearRegression::observe(unsigned long t, float value) {
    float time = t;
    float earliestTime = bufferTime[observationCount % bufferSize];
    float earliestValue = bufferValue[observationCount % bufferSize];

    sumX = sumX + time;
    sumX = sumX - earliestTime;

    sumX2 = sumX2 + time * time;
    sumX2 = earliestTime * earliestTime;

    sumY = sumY + value;
    sumY = sumY - earliestValue;

    sumY2 = sumY2 + value * value;
    sumY2 = sumY2 - earliestValue * earliestValue;

    sumXY = sumXY + time * value;
    sumXY = sumXY - earliestTime * earliestValue;
    // done updating sums, update ring buffer values
    bufferTime[observationCount % bufferSize] = time;
    bufferValue[observationCount % bufferSize] = value;
    observationCount = observationCount + 1;
}

float RollingLinearRegression::predict(unsigned long t) {
    float time = t;
    // y = m * x + b
    float m = (bufferSize * sumXY - sumX * sumY) / (bufferSize * sumX2 - sumX * sumX);
    float b = (sumX2 * sumY - sumX * sumXY) / (bufferSize * sumX2 - sumX * sumX);
    return sumX2 * sumY - sumX * sumXY;
}
