#include <Arduino.h>
#include "imxrt.h"
#include "kalman_four_state/kalman_jerk.cpp"
#include "encoder/digital_encoder.cpp"
#include "lib/generic/encoder_sample_validator.cpp"
#include "TeensyTimerTool.h"

using namespace TeensyTimerTool;

/**
 * FourStateTeensy40AS5147PRotarySensor
 *
 * Class to log out the four state (displacement, velocity, acceleration and jerk) computed by the extended Kalman method for a
 * AS5147P rotary encoder on the teensy40 platform.
 */
namespace kaepek
{
  class FourStateTeensy40AS5147PRotarySensor : public EncoderSampleValidator
  {
  public:
    // Default constuctor.
    FourStateTeensy40AS5147PRotarySensor() : EncoderSampleValidator()
    {
    }

    // Constructor with parameters.
    FourStateTeensy40AS5147PRotarySensor(DigitalEncoderSPI encoder, float sample_period_microseconds) : EncoderSampleValidator(encoder, sample_period_microseconds)
    {
    }

    void post_sample_logic(uint32_t encoder_value)
    {
    }

    void post_fault_logic(EncoderSampleValidator::Fault fault_code)
    {
      // Stop logging.
      Serial.println("A fault occured. Check your encoder connection.")
    }
  };
}

// Define encoder pin config struct.
kaepek::DigitalEncoderPinsSPI enc_pins = kaepek::DigitalEncoderPinsSPI();
// Define the encoder.
kaepek::DigitalEncoderSPI enc;
// Define the encoder sampler.
kaepek::FourStateTeensy40AS5147PRotarySensor sampler;
// Define bool for knowing if the sampler started is a good state.
bool started_ok = false;

// Method to print Kalman state via the serial port.
void print_kalman_flat(double *kalman_vec)
{
  Serial.print(kalman_vec[0]);
  Serial.print(",");
  Serial.print(kalman_vec[1]);
  Serial.print(",");
  Serial.print(kalman_vec[2]);
  Serial.print(",");
  Serial.print(kalman_vec[3]);
}

// Method to print Kalman state via the serial port with option to ignore printing the displacement.
void print_kalman_flat(double *kalman_vec, bool dont_print_disp)
{
  if (dont_print_disp == false)
  {
    Serial.print(kalman_vec[0]);
    Serial.print(",");
  }
  Serial.print(kalman_vec[1]);
  Serial.print(",");
  Serial.print(kalman_vec[2]);
  Serial.print(",");
  Serial.print(kalman_vec[3]);
}

// Method to print the Eular state via the serial port.
void print_eular_flat(double *eular_vec)
{
  Serial.print(eular_vec[0]);
  Serial.print(",");
  Serial.print(eular_vec[1]);
  Serial.print(",");
  Serial.print(eular_vec[2]);
  Serial.print(",");
  Serial.print(eular_vec[3]);
  Serial.print(",");
  Serial.print(eular_vec[4]);
}

// Method to print both the Eular and Kalman state of the rotary sensor from the cache.
void print_k()
{
  print_eular_flat(eular_vec_store);
  Serial.print(",");
  print_kalman_flat(kalman_vec_store, false);
  Serial.print("\n");
}

// Rotary encoder Kalman/Eular state storage.
kaepek::Dbl4x1 kalman_vec_store = {};
kaepek::Dbl4x1 eular_vec_store = {};

// Create a timer to allow for periodic logging to the serial port with the rotary sensor's Kalman and Eular state.
PeriodicTimer logging_timer(GPT2);

void setup()
{

  // Setup the encoder pin configuration.
  enc_pins.csn = 10;
  enc_pins.miso = 12;
  enc_pins.mosi = 11;
  enc_pins.sck = 22;

  // Initalise the encoder with giving it the pin configuration.
  enc = kaepek::DigitalEncoderSPI(enc_pins);

  // Initalise the encoder sampler.
  sampler = kaepek::FourStateTeensy40AS5147PRotarySensor(enc, 2.0); // 2us (micro) sample period

  // Allow skipping ahead a maximum value of 4.0, in terms of the read encoder value measurement, before a skip is detected.
  sampler.set_skip_tolerance(4.0);
  // Only allow skipping ahead twice before faulting.
  sampler.set_skip_threshold(2);

  // To disable direction enforcement.
  sampler.set_direction_enforcement(false);

  // Run setup procedure of the sampler. Note this will invoke the encoder's setup method and therefore it is unnecessary to do it explicitly on the encoder instance.
  sampler.setup();

  // Start sampling.
  started_ok = sampler.start();

  if (started_ok == true)
  {
    // Begin loggin the Kalman state.
    logging_timer.begin(print_k, 10'000);
  }
}

// Create a 1D four state Kalman filter: 
double alpha = 50000.0;
double angular_resolution_error = 40.0;
double process_noise = 0.000000000001;
// note KalmanJerk1D really needs a default constructor
kaepek::KalmanJerk1D filter = kaepek::KalmanJerk1D(alpha, angular_resolution_error, process_noise, true, 16384.0); // constructor with relative time and mod limit of 16384;


void loop()
{
  if (started_ok == true)
  {
    // Check the encoder has a new sample.
    if (sampler.has_new_sample() == true)
    {
      // Define variables to store the sampled encoder value and the number of elapsed microseconds since the last samples retrieval.
      uint32_t encoder_value;
      uint32_t elapsed_micros_since_last_sample;
      // Fetch the stored values from the buffer.
      sampler.get_sample_and_elapsed_time(encoder_value, elapsed_micros_since_last_sample);
      // Convert microseconds to seconds.
      double seconds_since_last = (double) elapsed_micros_since_last_sample * (double) 1e-6;
      // Perform one kalman step with the data.
      filter.step(seconds_since_last, encoder_value);
      // Extract state values.
      double *kalman_vec = filter.get_kalman_vector();
      double *eular_vec = filter.get_eular_vector();
      // Store state in cache ready for printing.
      cli();
      kalman_vec_store[0] = kalman_vec[0];
      kalman_vec_store[1] = kalman_vec[1];
      kalman_vec_store[2] = kalman_vec[2];
      kalman_vec_store[3] = kalman_vec[3];
      eular_vec_store[0] = eular_vec[0];
      eular_vec_store[1] = eular_vec[1];
      eular_vec_store[2] = eular_vec[2];
      eular_vec_store[3] = eular_vec[3];
      eular_vec_store[4] = eular_vec[4];
      sei();
    }
  }
  else
  {
    // If the sampler did not start in a good state, then print the configuration issues out via the serial port, by invoking the print_configuration_issues method of the sampler.
    sampler.print_configuration_issues();
    delayMicroseconds(10'000'000);
  }
}
