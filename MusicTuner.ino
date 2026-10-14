#include <Arduino.h>
#include <driver/i2s.h>
#include <math.h>
#include <string.h>
#include <Adafruit_NeoPixel.h>  //version 1.12.3 
// using ESP32 by Espressif Systems V2.0.17

// ============================================================
// ESP32 UNIVERSAL SAX / CLARINET TUNER
// Mark Donners aka The Electronic Engineer
// Created for Element 14 Presents
// Instruments:
//   - Tenor saxophone
//   - Alto saxophone
//   - Soprano saxophone
//   - Bb clarinet
//
// Fast live pitch tracking
//
// INMP441:
//   VDD -> 3.3V
//   GND -> GND
//   SCK -> GPIO26
//   WS  -> GPIO25
//   SD  -> GPIO33
//   L/R -> GND
// ============================================================


// ============================================================
// I2S
// ============================================================

#define I2S_PORT I2S_NUM_0

#define I2S_BCK 26
#define I2S_WS 25
#define I2S_DIN 33
#define LED_PIN 27
#define LED_COUNT 42

Adafruit_NeoPixel leds(
  LED_COUNT,
  LED_PIN,
  NEO_GRB + NEO_KHZ800);

// ============================================================
// AUDIO SETTINGS
// ============================================================

#define SAMPLE_RATE 16000

// Analysis window
#define BUFFER_SIZE 1024

// 50% overlap
#define HOP_SIZE 512

// Number of samples to read each time
#define READ_SAMPLES HOP_SIZE


// ============================================================
// FREQUENCY RANGE
// ============================================================

// Covers:
//   tenor sax
//   alto sax
//   soprano sax
//   Bb clarinet
//
// with room for high register / altissimo.

#define MIN_FREQUENCY 80.0f
#define MAX_FREQUENCY 2200.0f


// Corresponding lag limits:
//
// 16000 / 2200 = 7.27
// 16000 / 80   = 200

#define MIN_LAG 7
#define MAX_LAG 200


// ============================================================
// PITCH DETECTION SETTINGS
// ============================================================

// Minimum normalized peak strength.
//
// Higher = more selective
// Lower  = detects weaker notes but more false positives

#define PITCH_THRESHOLD 0.72f


// RMS threshold for silence

#define MIN_RMS 8000.0f


// ============================================================
// PITCH STABILITY
// ============================================================

// We deliberately keep this small.
//
// We want fast response for jazz playing.

#define STABILITY_HISTORY 3

float frequencyHistory[STABILITY_HISTORY];

int historyCount = 0;
int historyIndex = 0;


// ============================================================
// AUDIO BUFFER
// ============================================================

float audioBuffer[BUFFER_SIZE];


// Temporary buffer for incoming stereo I2S data

int32_t i2sBuffer[READ_SAMPLES * 2];


// ============================================================
// NSDF ARRAYS
// ============================================================

float nsdf[MAX_LAG + 1];


// ============================================================
// NOTE NAMES
// ============================================================

const char* noteNames[12] = {
  "C",
  "C#",
  "D",
  "D#",
  "E",
  "F",
  "F#",
  "G",
  "G#",
  "A",
  "A#",
  "B"
};


// ============================================================
// I2S SETUP
// ============================================================

void setupI2S() {
  i2s_config_t config = {};

  config.mode =
    (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);

  config.sample_rate =
    SAMPLE_RATE;

  config.bits_per_sample =
    I2S_BITS_PER_SAMPLE_32BIT;

  // IMPORTANT:
  //
  // This is the configuration proven to work with
  // our INMP441.
  //
  // L/R = GND -> LEFT channel.

  config.channel_format =
    I2S_CHANNEL_FMT_RIGHT_LEFT;

  config.communication_format =
    I2S_COMM_FORMAT_STAND_I2S;

  config.intr_alloc_flags =
    ESP_INTR_FLAG_LEVEL1;

  config.dma_buf_count =
    8;

  config.dma_buf_len =
    256;

  config.use_apll =
    false;

  config.tx_desc_auto_clear =
    false;

  config.fixed_mclk =
    0;


  // ----------------------------------------------------------
  // Install driver
  // ----------------------------------------------------------

  esp_err_t err =
    i2s_driver_install(
      I2S_PORT,
      &config,
      0,
      NULL);


  if (err != ESP_OK) {
    Serial.print(
      "I2S driver error: ");

    Serial.println(
      esp_err_to_name(err));

    while (true) {
      delay(1000);
    }
  }


  // ----------------------------------------------------------
  // Pin configuration
  // ----------------------------------------------------------

  i2s_pin_config_t pins = {};

  pins.bck_io_num =
    I2S_BCK;

  pins.ws_io_num =
    I2S_WS;

  pins.data_out_num =
    I2S_PIN_NO_CHANGE;

  pins.data_in_num =
    I2S_DIN;


  err =
    i2s_set_pin(
      I2S_PORT,
      &pins);


  if (err != ESP_OK) {
    Serial.print(
      "I2S pin error: ");

    Serial.println(
      esp_err_to_name(err));

    while (true) {
      delay(1000);
    }
  }


  Serial.println(
    "I2S initialized.");
}


// ============================================================
// INITIAL AUDIO BUFFER
// ============================================================
//
// Fill the complete 1024 sample window once at startup.
//
// ============================================================

void fillInitialBuffer() {
  size_t bytesRead = 0;


  i2s_read(
    I2S_PORT,
    i2sBuffer,
    sizeof(i2sBuffer),
    &bytesRead,
    portMAX_DELAY);


  for (int i = 0; i < READ_SAMPLES; i++) {
    int32_t left =
      i2sBuffer[i * 2];


    audioBuffer[i] =
      (float)(left >> 8);
  }


  // We need another 512 samples
  // to complete the 1024 sample buffer.

  i2s_read(
    I2S_PORT,
    i2sBuffer,
    sizeof(i2sBuffer),
    &bytesRead,
    portMAX_DELAY);


  for (int i = 0; i < READ_SAMPLES; i++) {
    int32_t left =
      i2sBuffer[i * 2];


    audioBuffer[i + READ_SAMPLES] =
      (float)(left >> 8);
  }
}


// ============================================================
// READ NEXT 512 SAMPLES
// ============================================================
//
// Keep the second half of the previous window.
//
// Then append 512 new samples.
//
// This gives 50% overlap.
//
// ============================================================

void readNextBlock() {
  // ----------------------------------------------------------
  // Move second half to first half
  // ----------------------------------------------------------

  memmove(
    audioBuffer,
    audioBuffer + HOP_SIZE,
    sizeof(float) * HOP_SIZE);


  // ----------------------------------------------------------
  // Read 512 new stereo samples
  // ----------------------------------------------------------

  size_t bytesRead = 0;


  i2s_read(
    I2S_PORT,
    i2sBuffer,
    sizeof(i2sBuffer),
    &bytesRead,
    portMAX_DELAY);


  // ----------------------------------------------------------
  // Extract LEFT channel
  // ----------------------------------------------------------

  for (int i = 0; i < READ_SAMPLES; i++) {
    int32_t left =
      i2sBuffer[i * 2];


    audioBuffer[HOP_SIZE + i] =
      (float)(left >> 8);
  }
}


// ============================================================
// REMOVE DC + CALCULATE RMS
// ============================================================

float prepareAudio() {
  float sum = 0.0f;


  // ----------------------------------------------------------
  // Calculate average
  // ----------------------------------------------------------

  for (int i = 0; i < BUFFER_SIZE; i++) {
    sum += audioBuffer[i];
  }


  float average =
    sum / BUFFER_SIZE;


  // ----------------------------------------------------------
  // Remove DC
  // ----------------------------------------------------------

  float sumSquares = 0.0f;


  for (int i = 0; i < BUFFER_SIZE; i++) {
    float value =
      audioBuffer[i] - average;


    audioBuffer[i] =
      value;


    sumSquares +=
      value * value;
  }


  float rms =
    sqrtf(
      sumSquares / BUFFER_SIZE);


  return rms;
}


// ============================================================
// CALCULATE NSDF
// ============================================================
//
// Normalized Square Difference Function.
//
// This is similar in principle to the McLeod Pitch Method.
//
// A strong periodic signal produces a strong positive peak.
//
// ============================================================

void calculateNSDF() {
  // ----------------------------------------------------------
  // For every possible period
  // ----------------------------------------------------------

  for (
    int tau = MIN_LAG;
    tau <= MAX_LAG;
    tau++) {
    float acf = 0.0f;

    float energy1 = 0.0f;

    float energy2 = 0.0f;


    int limit =
      BUFFER_SIZE - tau;


    for (
      int i = 0;
      i < limit;
      i++) {
      float x =
        audioBuffer[i];

      float y =
        audioBuffer[i + tau];


      acf +=
        x * y;

      energy1 +=
        x * x;

      energy2 +=
        y * y;
    }


    float denominator =
      energy1 + energy2;


    if (denominator > 0.0f) {
      nsdf[tau] =
        2.0f * acf / denominator;
    } else {
      nsdf[tau] =
        0.0f;
    }
  }
}


// ============================================================
// FIND PITCH PEAK
// ============================================================
//
// We look for local maxima.
//
// Important:
//
// We prefer the FIRST strong periodic peak rather than
// simply the highest peak. This helps reduce octave errors.
//
// ============================================================

int findPitchPeak(
  float& peakValue) {
  peakValue =
    0.0f;


  int bestTau =
    -1;


  // ----------------------------------------------------------
  // First find local maxima
  // ----------------------------------------------------------

  for (
    int tau = MIN_LAG + 1;
    tau < MAX_LAG - 1;
    tau++) {
    float value =
      nsdf[tau];


    // Local maximum?

    if (
      value > nsdf[tau - 1] && value >= nsdf[tau + 1]) {
      if (
        value >= PITCH_THRESHOLD) {
        // ----------------------------------------------------
        // We have found a strong periodic candidate.
        //
        // Prefer the first strong peak.
        // ----------------------------------------------------

        bestTau =
          tau;

        peakValue =
          value;

        break;
      }
    }
  }


  // ----------------------------------------------------------
  // If no strong peak was found,
  // find the strongest peak.
  // ----------------------------------------------------------

  if (bestTau < 0) {
    for (
      int tau = MIN_LAG + 1;
      tau < MAX_LAG - 1;
      tau++) {
      float value =
        nsdf[tau];


      if (
        value > peakValue && value > nsdf[tau - 1] && value >= nsdf[tau + 1]) {
        peakValue =
          value;

        bestTau =
          tau;
      }
    }
  }


  return bestTau;
}


// ============================================================
// PARABOLIC PEAK INTERPOLATION
// ============================================================

float refinePeak(
  int tau) {
  if (
    tau <= MIN_LAG || tau >= MAX_LAG) {
    return (float)tau;
  }


  float y1 =
    nsdf[tau - 1];

  float y2 =
    nsdf[tau];

  float y3 =
    nsdf[tau + 1];


  float denominator =
    y1 - 2.0f * y2 + y3;


  if (
    fabsf(denominator) < 0.000001f) {
    return (float)tau;
  }


  float correction =
    0.5f * (y1 - y3) / denominator;


  return (float)tau + correction;
}


// ============================================================
// OCTAVE / HARMONIC VALIDATION
// ============================================================
//
// If we detect:
//
//   440 Hz
//
// but 220 Hz also has a strong periodic peak,
// we prefer 220 Hz.
//
// This is particularly useful for saxophone and clarinet.
//
// ============================================================

float correctOctave(
  float frequency,
  int tau) {
  float lowerFrequency =
    frequency / 2.0f;


  // Lower octave outside our range?
  if (
    lowerFrequency < MIN_FREQUENCY) {
    return frequency;
  }


  int lowerTau =
    (int)roundf(
      (float)SAMPLE_RATE / lowerFrequency);


  if (
    lowerTau < MIN_LAG || lowerTau > MAX_LAG) {
    return frequency;
  }


  float currentScore =
    nsdf[tau];

  float lowerScore =
    nsdf[lowerTau];


  // ----------------------------------------------------------
  // IMPORTANT:
  //
  // Only choose the lower octave if it is actually STRONGER
  // than the detected frequency.
  //
  // We do NOT accept "almost as strong".
  //
  // This prevents a 440 Hz tone from becoming 220 Hz simply
  // because 220 Hz is also periodic.
  // ----------------------------------------------------------

  if (
    lowerScore > currentScore + 0.005f) {
    return lowerFrequency;
  }


  return frequency;
}


// ============================================================
// DETECT PITCH
// ============================================================
//
// Returns frequency in Hz.
//
// Returns NAN if no reliable pitch.
//
// ============================================================

float detectPitch(
  float& confidence) {
  confidence =
    0.0f;


  // ----------------------------------------------------------
  // Calculate NSDF
  // ----------------------------------------------------------

  calculateNSDF();

  //Serial.print("NSDF 440=");
  //Serial.print(nsdf[36], 3);

  //Serial.print("  NSDF 220=");
  //Serial.println(nsdf[73], 3);
  // ----------------------------------------------------------
  // Find pitch peak
  // ----------------------------------------------------------

  float peakValue =
    0.0f;


  int tau =
    findPitchPeak(
      peakValue);


  if (tau < 0) {
    return NAN;
  }


  // ----------------------------------------------------------
  // Confidence
  // ----------------------------------------------------------

  confidence =
    constrain(
      peakValue,
      0.0f,
      1.0f);


  // Reject weak signals

  if (
    peakValue < 0.55f) {
    return NAN;
  }


  // ----------------------------------------------------------
  // Refine period
  // ----------------------------------------------------------

  float refinedTau =
    refinePeak(
      tau);


  if (
    refinedTau <= 0.0f) {
    return NAN;
  }


  // ----------------------------------------------------------
  // Convert period to frequency
  // ----------------------------------------------------------

  float frequency =
    (float)SAMPLE_RATE / refinedTau;


  // ----------------------------------------------------------
  // Octave correction
  // ----------------------------------------------------------

  frequency =
    correctOctave(
      frequency,
      tau);


  // ----------------------------------------------------------
  // Check range
  // ----------------------------------------------------------

  if (
    frequency < MIN_FREQUENCY || frequency > MAX_FREQUENCY) {
    return NAN;
  }


  return frequency;
}


// ============================================================
// FREQUENCY HISTORY
// ============================================================

void clearHistory() {
  historyCount =
    0;

  historyIndex =
    0;


  for (
    int i = 0;
    i < STABILITY_HISTORY;
    i++) {
    frequencyHistory[i] =
      0.0f;
  }
}


// ============================================================
// ADD FREQUENCY
// ============================================================

void addFrequency(
  float frequency) {
  frequencyHistory[historyIndex] =
    frequency;


  historyIndex++;


  if (
    historyIndex >= STABILITY_HISTORY) {
    historyIndex =
      0;
  }


  if (
    historyCount < STABILITY_HISTORY) {
    historyCount++;
  }
}


// ============================================================
// GET STABLE FREQUENCY
// ============================================================
//
// With only 3 samples this is intentionally lightweight.
//
// For a new note we don't wait for a long history.
//
// ============================================================

float getStableFrequency() {
  if (
    historyCount <= 0) {
    return NAN;
  }


  float values[STABILITY_HISTORY];


  for (
    int i = 0;
    i < historyCount;
    i++) {
    values[i] =
      frequencyHistory[i];
  }


  // Sort

  for (
    int i = 0;
    i < historyCount - 1;
    i++) {
    for (
      int j = i + 1;
      j < historyCount;
      j++) {
      if (
        values[j] < values[i]) {
        float temp =
          values[i];

        values[i] =
          values[j];

        values[j] =
          temp;
      }
    }
  }


  // Median

  if (
    historyCount == 1) {
    return values[0];
  }


  if (
    historyCount == 2) {
    return (
             values[0] + values[1])
           * 0.5f;
  }


  return values[1];
}


// ============================================================
// FREQUENCY -> MIDI
// ============================================================

int frequencyToMidi(
  float frequency) {
  return (int)roundf(
    12.0f * log2f(frequency / 440.0f) + 69.0f);
}


// ============================================================
// DISPLAY NOTE
// ============================================================

void displayNote(
  float frequency,
  float confidence) {
  int midi =
    frequencyToMidi(
      frequency);


  int note =
    midi % 12;


  if (note < 0) {
    note += 12;
  }


  int octave =
    (midi / 12) - 1;


  // ----------------------------------------------------------
  // Target note frequency
  // ----------------------------------------------------------

  float targetFrequency =
    440.0f * powf(2.0f, ((float)midi - 69.0f) / 12.0f);


  // ----------------------------------------------------------
  // Cents
  // ----------------------------------------------------------

  float cents =
    1200.0f * log2f(frequency / targetFrequency);


  // ----------------------------------------------------------
  // Output
  // ----------------------------------------------------------

  Serial.print(
    "NOTE=");

  Serial.print(
    noteNames[note]);

  Serial.print(
    octave);


  Serial.print(
    "  FREQ=");

  Serial.print(
    frequency,
    2);

  Serial.print(
    "Hz");


  Serial.print(
    "  CENTS=");


  if (cents >= 0.0f) {
    Serial.print("+");
  }


  Serial.print(
    cents,
    1);


  Serial.print(
    "  CONF=");

  Serial.print(
    confidence * 100.0f,
    0);

  Serial.print(
    "%");


  // ----------------------------------------------------------
  // Tuning status
  // ----------------------------------------------------------

  if (cents < -10.0f) {
    Serial.println("  TOO LOW");

    // RED
    setLEDs(255, 0, 0);
  } else if (cents <= 10.0f) {
    Serial.println("  IN TUNE");

    // GREEN
    setLEDs(0, 255, 0);
  } else {
    Serial.println("  TOO HIGH");

    // ORANGE
    setLEDs(255, 80, 0);
  }
}


// ============================================================
// SETUP
// ============================================================

void setup() {
  // Faster Serial output.
  //
  // Use 230400 in Serial Monitor.

  Serial.begin(
    230400);


  delay(500);


  Serial.println();
  Serial.println(
    "==========================================");

  Serial.println(
    " ESP32 UNIVERSAL WOODWIND TUNER V2");

  Serial.println(
    " Fast live pitch tracking");

  Serial.println(
    " Saxophone + Bb Clarinet");

  Serial.println(
    "==========================================");

  Serial.println();


  Serial.print(
    "Sample rate : ");

  Serial.print(
    SAMPLE_RATE);

  Serial.println(
    " Hz");


  Serial.print(
    "Buffer      : ");

  Serial.print(
    BUFFER_SIZE);

  Serial.println(
    " samples");


  Serial.print(
    "Hop         : ");

  Serial.print(
    HOP_SIZE);

  Serial.println(
    " samples");


  Serial.print(
    "Analysis    : ");

  Serial.print(
    1000.0f * HOP_SIZE / SAMPLE_RATE,
    1);

  Serial.println(
    " ms");


  Serial.print(
    "Range       : ");

  Serial.print(
    MIN_FREQUENCY,
    0);

  Serial.print(
    " - ");

  Serial.print(
    MAX_FREQUENCY,
    0);

  Serial.println(
    " Hz");


  Serial.println();


  setupI2S();


  Serial.println();
  Serial.println(
    "Filling audio buffer...");


  // Initial 1024 samples

  fillInitialBuffer();


  Serial.println(
    "Ready.");

  Serial.println();
  leds.begin();
  leds.setBrightness(80);
  leds.clear();
  leds.show();
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
  // ----------------------------------------------------------
  // Get 512 new samples
  //
  // Previous 512 samples remain in buffer.
  // ----------------------------------------------------------

  readNextBlock();


  // ----------------------------------------------------------
  // Remove DC and calculate RMS
  // ----------------------------------------------------------

  float rms =
    prepareAudio();


  // ----------------------------------------------------------
  // Silence detection
  // ----------------------------------------------------------

  if (
    rms < MIN_RMS) {
    clearHistory();

    Serial.print(
      "NO SIGNAL  RMS=");

    Serial.println(
      rms,
      0);
    setLEDs(0, 0, 0);
    return;
  }


  // ----------------------------------------------------------
  // Pitch detection
  // ----------------------------------------------------------

  float confidence =
    0.0f;


  float frequency =
    detectPitch(
      confidence);


  // ----------------------------------------------------------
  // No reliable pitch
  // ----------------------------------------------------------

  if (
    isnan(frequency)) {
    Serial.print(
      "NO PITCH   RMS=");

    Serial.print(
      rms,
      0);

    Serial.print(
      "  CONF=");

    Serial.println(
      confidence * 100.0f,
      0);
    setLEDs(0, 0, 0);
    return;
  }


  // ----------------------------------------------------------
  // Add frequency to very short history
  // ----------------------------------------------------------

  addFrequency(
    frequency);


  // ----------------------------------------------------------
  // Get stable frequency
  // ----------------------------------------------------------

  float stableFrequency =
    getStableFrequency();


  // ----------------------------------------------------------
  // Display
  // ----------------------------------------------------------

  Serial.print(
    "RMS=");

  Serial.print(
    rms,
    0);

  Serial.print(
    "  RAW=");

  Serial.print(
    frequency,
    2);

  Serial.print(
    "Hz  ");


  displayNote(
    stableFrequency,
    confidence);
}


void setLEDs(uint8_t red, uint8_t green, uint8_t blue) {
  for (int i = 0; i < LED_COUNT; i++) {
    leds.setPixelColor(
      i,
      leds.Color(red, green, blue));
  }

  leds.show();
}
