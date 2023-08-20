typedef enum {
    SAMPLE_RATE_NONE,
    SAMPLE_RATE_8KHZ,
    SAMPLE_RATE_16KHZ,
} sample_rate_t;

void adc_transport_initialize();
void adc_initialize();
void adc_deinit();