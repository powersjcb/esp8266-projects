#include "LinearPredictor.h"

LinearPredictor::LinearPredictor(void) {
    for (int i = 0; i < BUFFER_SIZE; i++) {
        bufferTime[i] = DEFAULT_TIME;
    }
    for (int i = 0; i < BUFFER_SIZE; i++) {
        bufferValue[i] = DEFAULT_VALUE;
    }
}

bool LinearPredictor::finishedTraining(void) {
    return observationCount >= bufferSize;
}

void LinearPredictor::observe(float time, float value) {
    float earliestTime = bufferTime[observationCount % bufferSize];
    float earliestValue = bufferValue[observationCount % bufferSize];
    sumX += time;
    sumX -= earliestTime;
    sumX2 += time * time;
    sumX2 -= earliestTime * earliestTime;
    sumY += value;
    sumY -= earliestValue;
    sumY2 += value * value;
    sumY2 -= earliestValue * earliestValue;
    sumXY += time * value;
    sumXY -= earliestTime * earliestValue;
    bufferTime[observationCount % bufferSize] = time;
    bufferValue[observationCount % bufferSize] = value;
    observationCount += 1;
}

float LinearPredictor::predict(float time) {
    // m = velocity
    // x = time
    // b =
    // y = m * x + b
    float m;
    float mNum = (sumXY / bufferSize - sumX * sumY / bufferSize);
    float mDenom = (sumY / bufferSize - sumX * sumX / bufferSize);
    // handle edge cases that trigger nan/inf values
    if (mNum == 0) {
        m = 0;
    } else if (mDenom == 0) {
        m = 10e25;
    } else {
        m = mNum / mDenom;
    }

    float b = sumY / bufferSize - (m * sumX / bufferSize);
    return m * time + b;
}

#ifdef COMPILE_UNIT_TESTS
#include <iostream>     // std::cout
#include <cmath>        // std::abs

int testPredictor(void) {
    LinearPredictor predictor;
    for (int i; i < 50; i++) {
        predictor.observe(i, i);
    }

    if (predictor.observationCount != 50) {
        std::cout << "\nfailed: testPredictor\n";
        std::cout << predictor.observationCount;
        std::cout << "\n  - observationCount not incrementing\n";
        return 1;
    }

    if (!predictor.finishedTraining()) {
        std::cout << "\nfailed: testPredictor\n";
        std::cout << "  - did not finish training\n";
        return 1;
    }
    unsigned long time = 100;
    float prediction = predictor.predict(time);
    int diff = std::abs(prediction - time);
    if (diff > time * 0.05) {
        std::cout << "\nfailed: testPredictor\n";
        std::cout << "  - prediction too innacurate\n";
        std::cout << "  expected:\n";
        std::cout << time;
        std::cout << "\n  got:\n";
        std::cout << prediction;
        return 1;
    }
    std::cout << "\npassed: testPredictor\n";
    return 0;
}

int testPredictorEmpty(void) {
    LinearPredictor predictor;
    if (predictor.predict(1) != 1) {
        std::cout << "\nfailed: testPredictorEmpty\n";
        std::cout << predictor.predict(1);
        return 1;
    }
    std::cout << "\npassed: testPredictorEmpty\n";
    return 0;
}

int main() {
    return testPredictor() + testPredictorEmpty();
};
#endif
