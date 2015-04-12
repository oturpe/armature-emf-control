#include "config.h"

#include "AveragingDataSet.h"

AveragingDataSet::AveragingDataSet(uint16_t initialValue)
    : cursor(0) {
  for (int i = 0; i < AVG_WINDOW; i++) {
    buffer[i] = initialValue;
  }
}

int16_t AveragingDataSet::add(int16_t value, bool returnNew) {
  buffer[cursor] = value;
  cursor = (cursor + 1) % AVG_WINDOW;

  return returnNew ? average() : 0;
}

int16_t AveragingDataSet::average() {
  int32_t sum = 0;
  for (int i = 0; i < AVG_WINDOW; i++) {
    sum += buffer[i];
  }

  return sum / AVG_WINDOW;
}