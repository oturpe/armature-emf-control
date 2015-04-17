enum Direction {
  D_CLOCKWISE,
  D_COUNTER_CLOCKWISE,
  D_NONE,
  D_BOTH
};

/// \class LimitSensor
///
/// Manages sensing limits of movement.
class LimitSensor {

public:

  /// Returns the opposite direction.
  ///
  /// \param direction
  ///    Direction
  ///
  /// \return
  ///    Opposite direction
  static Direction opposite(Direction direction);

  /// Initializes a new limit sensor with given sensing interval.
  ///
  /// \param sensingInterval
  ///    How ofter sensors are actually read
  LimitSensor(uint16_t sensingInterval);

  /// Reads analog sensor readings of rotation sensor. If one of the values
  /// exceeds the threshold, returns the direction corresponding to the sensor.
  ///
  /// This function only actually reads the sensor every sensingIntercal
  /// invocations. Other times D_NONE is returned.
  ///
  /// \return
  ///    The sensor that has a reading, or D_NONE if sensors were not read.
  Direction senseLimit();

private:
  uint16_t counter;
  uint16_t sensingInterval;
};