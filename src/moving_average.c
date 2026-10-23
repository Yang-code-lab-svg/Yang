#include "moving_average.h"

void moving_average_init(MovingAverage *filter)
{
    int i;
    for (i = 0; i < FILTER_WINDOW; ++i) {
        filter->samples[i] = 0.0;
    }
    filter->sum = 0.0;
    filter->count = 0;
    filter->next_index = 0;
}

double moving_average_update(MovingAverage *filter, double sample)
{
    /* 第1~4拍只对已有样本求均值，避免补零导致启动偏差。 */
    if (filter->count < FILTER_WINDOW) {
        ++filter->count;
    } else {
        filter->sum -= filter->samples[filter->next_index];
    }
    filter->samples[filter->next_index] = sample;
    filter->sum += sample;
    filter->next_index = (filter->next_index + 1) % FILTER_WINDOW;
    return filter->sum / (double)filter->count;
}
