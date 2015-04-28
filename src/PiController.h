#include <inttypes.h>

class PiController {
public:
  /// Initializes a new pi controller.
  ///
  /// \param target
  ///    Controller target value
  ///
  /// \param deltaCoeff
  ///    Position coefficient. Larger value means less effect.
  ///
  /// \param integralCoeff
  ///    Integral coefficient. Larger value means less effect.
  PiController(int16_t target,
               int16_t positionCoeff,
               int16_t integralCoeff,
               int16_t resolution);

  /// Sets the target value. A new value is only actually set if requested new
  /// value differs from the currently set value by at least amount of
  /// resulution set for controller. In case the value is actually set, integral
  /// is reset to 0 as the old value becomes stale.
  ///
  /// \param newTarget
  ///    New target value.
  void setTarget(int16_t newTarget);

  //debug
  //int16_t getTarget() {return target;}

  /// Request new control value based on input.
  ///
  /// \param input
  ///    New input value
  ///
  /// \return
  ///    Output value
  int16_t control(int16_t input);

private:
  /// Target value for the controller.
  int16_t target;

  /// Coefficient of position value in pi controller equation.
  int16_t positionCoeff;

  /// Coefficient of integral value in pi controller equation.
  int16_t integralDivisor;

  /// Resolution for changing target.
  int16_t resolution;

  /// Controller's current integral value.
  int16_t integral;
};