//reference: https://wenku.baidu.com/view/3c42b7733186bceb18e8bb29.html
//put this file in Arduino's library folder

#include <math.h>

class Kalman{

    private:

        float X[2] = {0, 0};//angle, bias

        const float Q[2][2] = {{0.25, 0}, {0, 0.75}};//VarAngle 0; 0, VarBias

        float P[2][2] = {{0, 0}, {0, 0}};//cov matrix of X

        float K[2] = {0, 0};

        const float H[2] = {1, 0};

        const float R = 0.03;//variance of angle from acc

    public:

        float getAngle(float gyro, float acc, float dt){
            
            //1: X(k|k-1) = AX(k-1|k-1) + BU(k)
            //angle += (gyro - bias)*dt
            X[0] += (gyro - X[1]) * dt;

            //2: P(k|k-1) = AP(k-1|k-1)AT+ Q
            P[0][0] += Q[0][0] - (P[1][0] + P[0][1] - P[1][1] * dt) * dt;
            P[0][1] -= P[1][1] * dt;
            P[1][0] -= P[1][1] * dt;
            P[1][1] += Q[1][1];

            //3: Kg(k) = P(k|k-1)HT / (HP(k|k-1)HT + R), kalman gain; H = |1 0|
            K[0] = P[0][0] / (P[0][0] + R);
            K[1] = P[1][0] / (P[0][0] + R);

            //4: X(k|k) = X(k|k-1) + Kg(k)(Z(k) - HX(k|k-1))
            X[0] += K[0] * (acc - X[0]);
            X[1] += K[1] * (acc - X[0]);

            //5: P(k|k) = (I - Kg(k)H)P(k|k-1)
            P[0][0] -= K[0] * P[0][0];
            P[0][1] -= K[0] * P[0][1];
            P[1][0] -= K[1] * P[0][0];
            P[1][1] -= K[1] * P[0][1];

            return X[0];
        }

};


