
#ifndef bucketer_h
#define bucketer_h

typedef struct {
    int size;
    int buckets;
    int *indices;
} Bucketer_t;

Bucketer_t* NewBucketer(int size, int buckets, float f_min, float f_max);
void Bucket(Bucketer_t *b, float *frame, float *output);

#endif
