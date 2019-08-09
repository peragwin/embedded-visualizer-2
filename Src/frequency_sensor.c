#include "frequency_sensor.h"
#include "arm_math.h"
#include "bucketer.h"

Filter_t* NewFilter(int size, float *params) {
    float *values = (float*)calloc(size, sizeof(float));
    float *fparams = (float*)malloc(2*sizeof(float));
    memcpy(fparams, params, 2*sizeof(float));

    Filter_t *f = (Filter_t*)malloc(sizeof(Filter_t));
    f->params = params;
    f->size = size;
    f->values = values;
    
    return f;
}

FS_Drivers_t* NewDrivers(int size, int columns) {
    float **amp = (float**)malloc(columns * sizeof(float*));
    for (int i = 0; i < columns; i++) {
        float *amp_vals = (float*)calloc(size, sizeof(float));
        amp[i] = amp_vals;
    }

    float *diff = (float*)calloc(size, sizeof(float));
    float *energy = (float*)calloc(size, sizeof(float));
    
    FS_Drivers_t *d = (FS_Drivers_t*)malloc(sizeof(FS_Drivers_t));
    d->amp = amp;
    d->diff = diff;
    d->energy = energy;
    d->bass = 0;
    
    return d;
}

FS_GainController_t* NewGainController(int size, float *filterParams, float kp, float kd) {
    Filter_t *filter = NewFilter(size, filterParams); 

    float *gain = (float*)calloc(size, sizeof(float));
    float *err = (float*)calloc(size, sizeof(float));
    
    FS_GainController_t* gc = (FS_GainController_t*)malloc(sizeof(FS_GainController_t));
    gc->size = size;
    gc->filter = filter;
    gc->gain = gain;
    gc->err = err;
    gc->kp = kp;
    gc->kd = kd;

    return gc;
}

FS_Module_t* NewFrequencySensor(int size, int columns) {

    FS_Config_t *config = (FS_Config_t*)malloc(sizeof(FS_Config_t));
    config->offset = 0;
    config->gain = 2;
    config->diffGain = 1e-3;
    config->sync = 1e-2;
    config->mode = 1;
    config->preemph = 2;

    float gainParams[2] = {
        0.20, 0.80,
    };
    Filter_t *gainFilter = NewFilter(size, gainParams);

    float gainFeedbackP[2] = {
        -0.005, 0.995,
    };
    Filter_t *gainFeedback = NewFilter(size, gainFeedbackP);

    float diffParams[2] = {
        0.263, .737,
    };
    Filter_t *diffFilter = NewFilter(size, diffParams);

    float diffFeedbackP[2] = {
        -0.0028, 0.2272,
    };
    Filter_t *diffFeedback = NewFilter(size, diffFeedbackP);

    float gainControllerParams[2] = {
        0.005, 0.995,
    };
    float kp = 0.001;
    float kd = 0.005;
    FS_GainController_t *gc = NewGainController(size, gainControllerParams, kp, kd);
    
    FS_Drivers_t *drivers = NewDrivers(size, columns);

    FS_Module_t *fs = (FS_Module_t*)malloc(sizeof(FS_Module_t));
    fs->size = size;
    fs->columns = columns;
    fs->config = config;
    fs->gainFilter = gainFilter;
    fs->gainFeedback = gainFeedback;
    fs->diffFilter = diffFilter;
    fs->diffFeedback = diffFeedback;
    fs->gc = gc;
    fs->drivers = drivers;
    fs->renderLock = 0;

    return fs;
}

static void init_blackman_window(float *window, int size) {
  for (int i = 0; i < size; i++) {
    // blackman-harris window
    float c1 = arm_cos_f32(2.0 * PI * (float)i / (float)(size - 1));
    float c2 = arm_cos_f32(4.0 * PI * (float)i / (float)(size - 1));
    float c3 = arm_cos_f32(6.0 * PI * (float)i / (float)(size - 1));
    window[i] = 0.35875 - 0.48829 * c1 + 0.14128 * c2 - 0.01168 * c3;
    window[i] /= 32768; // 2**15 to get a range of [-1, 1]
  }
}


Audio_Processor_t* NewAudioProcessor(int size, int buckets, int columns) {
    Audio_Processor_t *ap = (Audio_Processor_t*)malloc(sizeof(Audio_Processor_t));
    ap->size = size;

    float *window = (float*)malloc(size*sizeof(float));
    init_blackman_window(window, size);
    ap->window = window;
    
    ap->bucketer = NewBucketer(size, buckets, 64, 16000);

    arm_rfft_fast_instance_f32 *fft = (arm_rfft_fast_instance_f32*)malloc(sizeof(arm_rfft_fast_instance_f32));
    arm_rfft_fast_init_f32(fft, size);
    ap->fft = fft;

    ap->fs = NewFrequencySensor(buckets, columns);

    return ap;
}

// converts 24bit right shifted 2s complement into signed ints
void convert_to_signed(int *input, int *output, int size) {
    for (int i = 0; i < size; i++) {
        output[i] = (input[i] << 8) >> 8;
    }
}

// removes the dc component by subtracting the average
void remove_dc_component(int *input, int *output, int size) {
    long sum = 0;
    for (int i = 0; i < size; i++) {
        sum += input[i];
    }
    sum /= size;
    for (int i = 0; i < size; i++) {
        output[i] = input[i] - size;
    }
}

void convert_to_float(int *input, float *output, int size) {
    for (int i = 0; i < size; i++) {
        output[i] = (float)input[i];
    }
}

void apply_window(float *input, float *window, int size) {
    arm_mult_f32(window, input, input, size);
}

// fast_log just returns the floating point's exponent!
float fast_log(float x) {
    unsigned int v = *((int*)&x);
    return (float)((v >> 23) - 127);
}

void power_spectrum(arm_rfft_fast_instance_f32 *fft, float *input, float *output) {
    int size = fft->fftLenRFFT;
    arm_rfft_fast_f32(fft, input, output, 0);
    // arm_abs_f32(output, output, size);
    for (int i = 0; i < size; i++) {
        output[i] = fast_log(output[i]);
    }
}

void Audio_Process(Audio_Processor_t *a, int *input) {
    convert_to_signed(input, input, a->size);
    remove_dc_component(input, input, a->size);
    
    float frame[a->size];
    float fftFrame[a->size];
    float bucketFrame[a->bucketer->buckets];

    convert_to_float(input, frame, a->size);
    apply_window(frame, a->window, a->size);

    power_spectrum(a->fft, frame, fftFrame);

    Bucket(a->bucketer, fftFrame, bucketFrame);

    FS_Process(a->fs, bucketFrame);
}

void FS_Process(FS_Module_t *f, float *input) {
    return;
}