#include "fftcalc.h"
#include "appglobal.h"
#include "logging.h"

#include <cstring>

fftCalc::fftCalc() {
  plan = nullptr;
  out = nullptr;
  dataBuffer = nullptr;
  dataBufferWindowed = nullptr;
}

fftCalc::~fftCalc() {
  if (plan) fftw_destroy_plan(plan);
  if (out) fftw_free(out);
  if (dataBuffer) fftw_free(dataBuffer);
}

void fftCalc::init(int length, int nblocks, int isamplingrate) {
  int i;
  windowSize = length;
  fftLength = windowSize * nblocks;
  numBlocks = nblocks;
  blockIndex = 0;
  createHamming();
  samplingrate = isamplingrate;
  // prepare fft
  if (plan) fftw_destroy_plan(plan);
  if (out) fftw_free(out);
  if (dataBuffer) delete[] dataBuffer;

  dataBuffer = new double[fftLength];
  for (i = 0; i < fftLength; i++) {
    dataBuffer[i] = 0.;
  }
  if (dataBufferWindowed) fftw_free(dataBufferWindowed);
  out = static_cast<double*>(fftw_malloc(fftLength * sizeof(double)));
  dataBufferWindowed = static_cast<double*>(fftw_malloc(fftLength * sizeof(double)));
  // create the fftw plan
  addToLog("fftw_plan fftcalc start", LOGFFT);
  plan = fftw_plan_r2r_1d(fftLength, dataBufferWindowed, out, FFTW_R2HC, FFTW_ESTIMATE);
  addToLog("fftw_plan fftcalc stop", LOGFFT);
}

void fftCalc::createHamming() {
  int i;
  hammingBuffer = new double[fftLength];
  for (i = 0; i < fftLength; i++) {
    hammingBuffer[i] = 0.54 - (0.46 * cos(2 * M_PI * (static_cast<double>(i) / (static_cast<double>(fftLength - 1)))));
  }
}

void fftCalc::realFFT(double* data) {
  int i;
  memmove(&dataBuffer[0], &dataBuffer[windowSize], sizeof(double) * windowSize * (numBlocks - 1));
  memmove(&dataBuffer[windowSize * (numBlocks - 1)], data, sizeof(double) * windowSize);
  for (i = 0; i < fftLength; i++) {
    dataBufferWindowed[i] = dataBuffer[i] * hammingBuffer[i];
  }
  fftw_execute(plan);
}
