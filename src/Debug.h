#include "config.h"

#include <inttypes.h>

class Debug {

public:
  /// Initializes debug instance.
  ///
  /// \param reportingFrequency
  ///    How often calls to printInfo(uint16_t) should actually report something
  Debug(uint16_t reportingFrequency);

  /// Prints debug information
  ///
  /// This crude implementation flashes a led connected to pin D5. This is only
  /// done once every reportingFrequency calls to avoid spending all the time
  /// flashing the led.
  ///
  /// \param value
  ///     Value to report
  void printInfo(uint16_t value);

private:
	// Frequency for reporting
	uint16_t reportingFrequency;

	// Info printing interval counter
	uint16_t counter;
};
