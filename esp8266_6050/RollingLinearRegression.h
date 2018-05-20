class RollingLinearRegression {
    public:
        RollingLinearRegression(int bufferSize);
        void observe(unsigned long time, double value);
        double predict(unsigned long time);
    private:
        int bufferSize;
        int observationCount;
        unsigned long bufferTime[];
        double bufferValue[];
        unsigned long sumX = 0;
        unsigned long sumX2 = 0;
        double sumY = 0;
        double sumY2 = 0;
        double sumXY = 0;
};
