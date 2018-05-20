//
// Created by Jacob Powers on 5/19/18.
//

#include "RollingLinearRegression.h"

RollingLinearRegression::RollingLinearRegression(int bufferSize) {
    observationCount = 0;
    bufferSize = bufferSize;
    bufferTime[observationCount] = {0};
    bufferValue[observationCount] = {0.0};
}

void RollingLinearRegression::observe(unsigned long time, double value) {
    unsigned long earliestTime = bufferTime[observationCount % bufferSize];
    unsigned long earliestValue = bufferValue[observationCount % bufferSize];

    sumX = sumX + time;
    sumX = sumX - earliestTime;

    sumX = sumX + time * time;
    sumX2 = sumX - earliestTime * earliestTime;

    sumY = sumY + value;
    sumY = sumY - earliestValue;

    sumY2 = sumY - earliestValue;

    // done updating sums, update ring buffer values
    bufferTime[observationCount % bufferSize] = time;
    bufferValue[observationCount % bufferSize] = value;
}

double RollingLinearRegression::predict(unsigned long time) {
    // y = m * x + b
    double m = (bufferSize * sumXY - sumX * sumY) / (bufferSize * sumX2 - sumX * sumX);
    double b = (sumX2 * sumY - sumX * sumXY) / (bufferSize * sumX2 - sumX * sumX);
    return m * time + b;
}
