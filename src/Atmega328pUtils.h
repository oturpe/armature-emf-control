// Helpers for working with Atmega328p.

// Cleaner setting of bits
#define BV(x) (1<<x)

namespace Atmega328p {

/// Possible prescaler values for timer 0.
enum Timer0PrescalerValue {
  PSV_1,
  PSV_8,
  PSV_64,
  PSV_256,
  PSV_1024,
};

/// Sets timer 0 prescaler to requested value.
///
/// This function assumes that Clock Select bits have not been touched yet.
///
/// \param value
///   Requested prescaler value
void setTimer0Prescaler(Timer0PrescalerValue value);

// Possible voltage references
enum VoltageReference {
  // Aref pin
  VREF_AREF,
  // Vcc with external capacitor
  VREF_VCC,
  // Internal 1.1 V with external capacitor
  VREF_INTERNAL_1_1V
};

// Sets analog voltage reference to requested type.
//
// This function assumes that Reference Selection bits have not been touched
// yet.
//
// \param reference
//    Requested voltage reference
void setVoltageReference(VoltageReference reference);

// Possible analog-to-digital converter prescaler values
enum AdcPrescalerValue {
  ADC_PSV_2,
  ADC_PSV_4,
  ADC_PSV_8,
  ADC_PSV_16,
  ADC_PSV_32,
  ADC_PSV_64,
  ADC_PSV_128
};

// Sets analog-to-digital converter prescaler value.
//
// This function assumes that ADV Prescaler Selection bits have not been touched
// yet.
//
// \param value
//    Requested prescaler value
void setAdcPrescalerValue(AdcPrescalerValue value);

} // namespace