#include "PID.h"
#include <stdio.h>
#include <numeric>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double dKp, double dKi, double dKd, double tolerance) {

    Kpid.push_back(Kp);
    Kpid.push_back(Ki);
    Kpid.push_back(Kd);

    dKpid.push_back(dKp);
    dKpid.push_back(dKi);
    dKpid.push_back(dKd);
    tol = tolerance;

    p_error = 0;
    i_error = 0;
    d_error = 0;
}

void PID::UpdateError(double cte) {
    double pre_cte = p_error;
    p_error = cte;
    i_error += cte;
    d_error = cte - pre_cte;
}

double PID::TotalError() {
    return Kpid[0]*p_error + Kpid[1]*i_error + Kpid[2]*d_error;
}

#define TWIDDLE_N 100
void PID::Twiddle(double cte){

    static unsigned int cnt = 0;
    static double best_err = 0;
    static double err = 0;
    static bool initialized = false;
    static unsigned int i = 0;
    static unsigned int state = 0;

    cnt++;

    if(cnt < TWIDDLE_N){
        return;
    }
    else if(cnt >= TWIDDLE_N && cnt < 2*TWIDDLE_N){
        err += cte*cte;
        return;
    }
    else if(cnt >= 2*TWIDDLE_N){
        cnt = 0;
        err /= TWIDDLE_N;
    }

    if( false == initialized ){
        best_err = err;
        initialized = true;
        return;
    }

    double sum_dp = std::accumulate(std::begin(dKpid), std::end(dKpid), 0.0);
    if(sum_dp > tol){

        if(0 == state){
            Kpid[i] += dKpid[i];
            state ++;
            return;
        }
        else if(1 == state){
            if(err < best_err){
                best_err = err;
                dKpid[i] *= 1.1;
            }
            else{
                Kpid[i] -= 2 * dKpid[i];
                state ++;
                return;
            }
        }
        else if(2 == state){
            if(err < best_err){
                best_err = err;
                dKpid[i] *= 1.1;
            }
            else{
                Kpid[i] += dKpid[i];
                dKpid[i] *= 0.9;
            }
        }

        state = 0;
        i ++;
        i = i>=Kpid.size() ? 0 : i;
    }

    if(sum_dp <= tol){
        printf("final Kpid %f %f %f\n", Kpid[0], Kpid[1], Kpid[2]);
    }
}