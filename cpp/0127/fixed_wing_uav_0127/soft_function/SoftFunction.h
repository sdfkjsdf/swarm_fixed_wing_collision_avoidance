#ifndef SOFTFUNCTION
#define SOFTFUNCTION
#include<cmath>


class SoftFunction

{
    public:
         SoftFunction()=default;
        ~SoftFunction()=default;

       static inline double softplus_stable(double z)
        {
            // log(1+exp(z))를 overflow 없이 동일값으로 계산
            if (z > 0.0) {
                return z + std::log1p(std::exp(-z));
            } else {
                return std::log1p(std::exp(z));
            }
        }

        static inline double sigmoid_stable(double z)
    {
        if (z >= 0.0) {
            return 1.0 / (1.0 + std::exp(-z));
        } else {
            const double e = std::exp(z);
            return e / (1.0 + e);
        }
    }

    double symmetric_softsat(const double x, const double A, const double k)
        {
            const double s0 = std::tanh(0.5 * k * A);
            const double gamma = 1.0 / (s0 + 1e-6);

            const double z1 = k * (gamma * x + A);
            const double z2 = k * (gamma * x - A);

            const double term1 = softplus_stable(z1) / k;
            const double term2 = softplus_stable(z2) / k;

            return -A + term1 - term2;
        }

    double asym_softsat(double u, double L, double U, double k)
    {   
        const double eps = 1e-6;
        // u0=(L+U)/2
        const double u0 = 0.5 * (L + U);

        // us=u-u0, Ls=L-u0, Us=U-u0
        const double us = u - u0;
        const double Ls = L - u0;
        const double Us = U - u0;

        // v0 = (1/k) * ( log(1+exp(k*Ls)) - log(1+exp(-k*Us)) )
        const double v0 = (softplus_stable(k * Ls) - softplus_stable(-k * Us)) / k;

        // s0 = sigmoid(k*( -v0 - Ls )) - sigmoid(k*( -v0 - Us ))
        const double s0 = sigmoid_stable(k * (-v0 - Ls)) - sigmoid_stable(k * (-v0 - Us));

        // gamma = 1/(s0 + eps)
        const double gamma = 1.0 / (s0 + eps);

        // out_s = Ls + softplus(k*(gamma*us - v0 - Ls))/k - softplus(k*(gamma*us - v0 - Us))/k
        const double a = k * (gamma * us - v0 - Ls);
        const double b = k * (gamma * us - v0 - Us);

        const double out_s = Ls + softplus_stable(a) / k - softplus_stable(b) / k;

        // out = out_s + u0
        return out_s + u0;
 
    }
};





















#endif