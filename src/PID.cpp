#include "PID.h"
#include <stdio.h>
#include <numeric>
#include <iostream>
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

#define PREPARE_N   (250)
#define MEASURE_PERIOD_N (1000)
#define MEASURE_CYCLE_N (PREPARE_N+MEASURE_PERIOD_N)
void PID::Twiddle(double cte){

    static unsigned int cnt = 0;
    static double best_err = 0;
    static double err = 0;
    static bool err_initialized = false;
    static unsigned int i = 0;
    static unsigned int state = 0;
    static bool i_next = false;
    static bool first_in = true;

    cnt++;

    if(first_in){

        if(1==cnt){
            printf("%4d", cnt );
            fflush(stdout);
        }
        else{
            printf("\b\b\b\b%4d", cnt);
            fflush(stdout);
        }

        if(cnt > 400){
            first_in = false;
            cnt = 0;
            printf("\nfirst in initialized.\n");
        }
        return;
    }

    if(false == i_next){

        if(1==cnt){
            printf("%4d", cnt );
            fflush(stdout);
        }
        else{
            printf("\b\b\b\b%4d", cnt);
            fflush(stdout);
        }
        //printf("cnt: %d Kpid: %f %f %f dKpid: %f %f %f cte: %f\n", cnt, Kpid[0], Kpid[1], Kpid[2], dKpid[0], dKpid[1], dKpid[2], cte);
        if(cnt < PREPARE_N){
            return;
        }
        else if(cnt >= PREPARE_N && cnt < MEASURE_CYCLE_N){
            err += cte*cte;
            return;
        }
        else if(cnt >= MEASURE_CYCLE_N){
            cnt = 0;
            err /= MEASURE_PERIOD_N;
        }

        if( false == err_initialized ){
            best_err = err;
            err_initialized = true;
            printf("\nerr_initialized.\n");
        }

        printf("\nBest error: %f Current error: %f\n", best_err, err);
        printf("Best error: %f Current error: %f\n", best_err, err);
        printf("Best error: %f Current error: %f\n", best_err, err);
    }

    double sum_dp = std::accumulate(std::begin(dKpid), std::end(dKpid), 0.0);
    if(sum_dp <= tol){
        printf("final Kpid: %f %f %f dKpid: %f %f %f cte: %f\n", \
                        Kpid[0], Kpid[1], Kpid[2], dKpid[0], dKpid[1], dKpid[2], cte);
        exit(0);
    }


    if(sum_dp > tol){

        if(0 == state){
            Kpid[i] += dKpid[i];
            state ++;
            err = 0;
            i_next = false;
            printf("INC attempt.\n");
            printf("INC attempt.\n");
            printf("INC attempt.\n");
            printf("parameter: %d Kpid: %f %f %f dKpid: %f %f %f\n", \
                        i, Kpid[0], Kpid[1], Kpid[2], dKpid[0], dKpid[1], dKpid[2]);
            return;
        }
        else if(1 == state){
            if(err < best_err){
                best_err = err;
                dKpid[i] *= 1.1;
                printf("parameter INC accepted.\n");
                printf("parameter INC accepted.\n");
                printf("parameter INC accepted.\n");
            }
            else{
                Kpid[i] -= 2 * dKpid[i];
                state ++;
                err = 0;
                i_next = false;
                printf("DEC attempt.\n");
                printf("DEC attempt.\n");
                printf("DEC attempt.\n");
                printf("parameter: %d Kpid: %f %f %f dKpid: %f %f %f\n", \
                        i, Kpid[0], Kpid[1], Kpid[2], dKpid[0], dKpid[1], dKpid[2]);
                return;
            }
        }
        else if(2 == state){
            if(err < best_err){
                best_err = err;
                dKpid[i] *= 1.1;
                printf("parameter DEC accepted.\n");
                printf("parameter DEC accepted.\n");
                printf("parameter DEC accepted.\n");
            }
            else{
                Kpid[i] += dKpid[i];
                dKpid[i] *= 0.8;
                printf("Converge\n");
                printf("Converge\n");
                printf("Converge\n");
            }
        }

        err = 0;
        state = 0;
        i ++;
        i = i>=Kpid.size() ? 0 : i;
        if(1==i) i=2; // skip I parameter, since it looks like this system has no systematic bias.
        i_next = true;
        printf("next parameter.\n");
    }
}