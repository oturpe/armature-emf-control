#include <inttypes.h>

/// \class AveragingDataSet
///
/// Buffers number of readings and returns rolling average values. Length of
/// window to average is given by macŕo AVG_WINDOW
///
class AveragingDataSet {
public:
  /// Initializes store by filling the whole buffer with given value.
  ///
  /// \param initialValue
  ///    Initial value
  AveragingDataSet(uint16_t initialValue);

  /// Adds given value to dataset. Optionally can return the new rolling
  /// average.
  ///
  /// If return value is not requested, it contains zero.
  ///
  /// \param value
  ///    New value
  ///
  /// \param returnNew
  ///    If new average is returned
  ///
  /// \return
  ///    New rolling average value
  int16_t add(int16_t value, bool returnNew);

  /// Returns current rolling average.
  ///
  /// \return
  ///    Rolling average value
  int16_t average();

private:
  int16_t buffer[AVG_WINDOW];
  uint16_t cursor;
};
