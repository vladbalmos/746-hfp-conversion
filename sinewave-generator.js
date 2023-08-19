'use strict';
/**
 * Generate a PCM sine wave C header file for testing DAC output
 */
const fs = require('fs');

const SAMPLE_RATE = process.env['SAMPLE_RATE'] || 16000;
const MAX_SAMPLES = process.env['MAX_SAMPLES'] || 120;
const CYCLES = process.env['CYCLES'] || 5
const AMPLITUDE = 15000;  // for 16-bit PCM

function computeFrequency(sampleCount) {
    const period = sampleCount / SAMPLE_RATE;
    return Math.floor(1 / period);
}

function generateSineWave() {
    const sampleCount = MAX_SAMPLES / CYCLES;
    
    if (!Number.isInteger(sampleCount)) {
        throw new Error('MAX_SAMPLES / CYCLES must yield an integer');
    }

    const frequency = computeFrequency(sampleCount);
    const samples = [];
    
    console.log('Frequency', frequency, 'hz');
    console.log('Samples', sampleCount);

    for (let i = 0; i < sampleCount; i++) {
        let t = i / SAMPLE_RATE;  // time in seconds
        samples[i] = Math.round(AMPLITUDE * Math.sin(2 * Math.PI * frequency * t));
    }

    return [frequency, samples]
}

// Generate the sine wave samples
const [frequency, samples] = generateSineWave();
const wavetable = [];

for (let i = 0; i < CYCLES; i++) {
    wavetable.push(...samples);
}

const filename = `sine_${SAMPLE_RATE}_${frequency}.h`;
const output = [
    `#include <inttypes.h>`,
    `int16_t sinewave_${SAMPLE_RATE}[${MAX_SAMPLES}] = {${wavetable.join(', ')}};`,
    ''
];
        
fs.writeFileSync(filename, output.join('\n'));
console.log(`Wrote ${filename}`);
        