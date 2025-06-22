#ifndef SIMPLE_FILTER_H
#define SIMPLE_FILTER_H

typedef struct {
    float alpha;
    float y_kminus1;
} SimpleFilter;

float simple_filter_next(float u, SimpleFilter* filter);

#endif

