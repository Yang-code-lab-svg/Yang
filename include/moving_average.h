#ifndef MOVING_AVERAGE_H
#define MOVING_AVERAGE_H

#include "config.h"

typedef struct {
    double samples[FILTER_WINDOW];
    double sum;
    int count;
    int next_index;
} MovingAverage;

void moving_average_init(MovingAverage *filter);
double moving_average_update(MovingAverage *filter, double sample);

#endif
