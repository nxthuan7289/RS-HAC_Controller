#include <Arduino.h>
#include <Encoder.h>
#include "state_derivatives.h"         // Model header file
#include "HAC_swingUp_controller.h"
#include <ContinuousStepper.h>
#include <ContinuousStepper/Tickers/TeensyTimerTool.hpp>

#define STEP_PIN   8
#define DIR_PIN    9
#define ENABLE_PIN 10
#define SPEED_MAX  6563
#define MY_LED     25

static state_derivatives state_diff;        // Instance of model class
static HAC_swingUp_controller controllerObj;         // Instance of model class

TeensyTimerTool::PeriodicTimer timer;

// If using a stepper driver, with STEP and DIR pins:
ContinuousStepper<StepperDriver, TeensyTimerToolTicker> stepper(timer);

// Encoder
Encoder enc_pend(5, 6);
Encoder enc_cart(2, 3);
long old_pend_count  = -9999;
long old_cart_count = -9999;

// misc
double R = 0.0194;  // radius of the pulley
double pend_angle = PI;
double pend_angle_wrapped = PI;
double cart_position = 0;

double pend_ang_vel = 0;
double cart_pos_vel = 0;

double ref_signal[4] = {0, 0, 0, 0};  // x, x_d, q, q_d
double linearAccel = 0.0;

int stepPerRound = 200;  // step
double diameter = 0.0388;  // meter

const float pos_max = 0.43;

// functions templates
// void spi_send_accel();
int sign_accel(double accel);
void moveByAccel(void);
void differentiator_oneStep(void);
void controller_oneStep(void);

// Measure controller_oneStep() with the ARM DWT cycle counter on Teensy 4.1.
// Serial output is outside the timed region.
#define TIMING_N 1000
static uint32_t t_min_cyc = UINT32_MAX;
static uint32_t t_max_cyc = 0;
static uint64_t t_sum_cyc = 0;
static uint32_t t_count = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  digitalWrite(LED_BUILTIN, LOW);

  stepper.begin(STEP_PIN, DIR_PIN);

  delay(10);
  Serial.begin(115200);
  const uint32_t serial_wait_start = millis();
  while (!Serial && (millis() - serial_wait_start) < 10000U) {
    delay(10);
  }

  stepper.setEnablePin(ENABLE_PIN, LOW);

  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

  // init controller objs
  state_diff.initialize();
  controllerObj.initialize();
  stepper.powerOff();
  delay(3000);
  stepper.powerOn();

  stepper.setAcceleration(0);
  stepper.spin(0);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("TRIAL_START");

}

elapsedMicros stopWatch = 0;  // has to be here

void loop() {

  if (stopWatch < 1000) {  // stepTime = 1 ms
    return;
  }

  elapsedMicros stopWatchInner = 0;

  // check position limit
  if (abs(cart_position) > pos_max) {
    stepper.stop();
    stepper.powerOff();
    stopWatch = stopWatchInner;
    return;
  }

  // enc reading
  long new_pend_count, new_cart_count;

  new_pend_count = enc_pend.read();
  new_cart_count = enc_cart.read();

  pend_angle = ((new_pend_count + 4000.0) / 4000.0) * PI;  // radian
  cart_position = (new_cart_count / 4000.0) * PI * R;  // meter

  pend_angle_wrapped = atan2(sin(pend_angle), cos(pend_angle));

  differentiator_oneStep();  // differentiate to get x_dot and q_dot
  ARM_DWT_CYCCNT = 0;
  controller_oneStep();  // step the controller to get accel command to the motor
  const uint32_t t_cyc = ARM_DWT_CYCCNT;

  if (t_cyc < t_min_cyc) t_min_cyc = t_cyc;
  if (t_cyc > t_max_cyc) t_max_cyc = t_cyc;
  t_sum_cyc += t_cyc;
  t_count++;

  moveByAccel();
  const uint32_t sample_ms = millis();
  Serial.print(cart_position, 4);
  Serial.print(",");
  Serial.print(cart_pos_vel, 4);
  Serial.print(",");
  Serial.print(pend_angle, 4);
  Serial.print(",");
  Serial.print(pend_ang_vel, 4);
  Serial.print(",");
  Serial.print(linearAccel, 4);
  Serial.print(",");
  Serial.print(sample_ms);
  Serial.println();

  if (t_count >= TIMING_N) {
    const float mhz = (float)F_CPU / 1.0e6f;
    Serial.print("TIMING us: min=");
    Serial.print(t_min_cyc / mhz, 3);
    Serial.print(" mean=");
    Serial.print((float)(t_sum_cyc / t_count) / mhz, 3);
    Serial.print(" max=");
    Serial.print(t_max_cyc / mhz, 3);
    Serial.print(" n=");
    Serial.println(t_count);
    t_min_cyc = UINT32_MAX;
    t_max_cyc = 0;
    t_sum_cyc = 0;
    t_count = 0;
  }
  

  old_pend_count = new_pend_count;
  old_cart_count = new_cart_count;

  // Serial.println(stepper.speed());

  stopWatch = stopWatchInner;
}

void differentiator_oneStep(void)
{
  static boolean_T OverrunFlag{ false };

  // Disable interrupts here

  // Check for overrun
  if (OverrunFlag) {
    return;
  }

  OverrunFlag = true;

  // Save FPU context here (if necessary)
  // Re-enable timer or interrupt here
  // Set model inputs here
  state_diff.rtU.x_in = cart_position;
  state_diff.rtU.q_in = pend_angle;

  // Step the model
  state_diff.step();

  // Get model outputs here
  cart_pos_vel = state_diff.rtY.x_dot;
  pend_ang_vel = state_diff.rtY.q_dot;

  // Indicate task complete
  OverrunFlag = false;

  // Disable interrupts here
  // Restore FPU context here (if necessary)
  // Enable interrupts here
}

void controller_oneStep(void)
{
  static boolean_T OverrunFlag{ false };

  // Disable interrupts here

  // Check for overrun
  if (OverrunFlag) {
    return;
  }

  OverrunFlag = true;

  // Save FPU context here (if necessary)
  // Re-enable timer or interrupt here
  // Set model inputs here
  controllerObj.rtU.X_f[0] = cart_position;
  controllerObj.rtU.X_f[1] = cart_pos_vel;
  controllerObj.rtU.X_f[2] = pend_angle_wrapped;
  controllerObj.rtU.X_f[3] = pend_ang_vel;

  controllerObj.rtU.error[0] = cart_position - ref_signal[0];
  controllerObj.rtU.error[1] = cart_pos_vel - ref_signal[1];
  controllerObj.rtU.error[2] = pend_angle_wrapped - ref_signal[2];
  controllerObj.rtU.error[3] = pend_ang_vel - ref_signal[3];

  // Step the model
  controllerObj.step();

  // Get model outputs here
  linearAccel = controllerObj.rtY.u;  // m/s2

  // Indicate task complete
  OverrunFlag = false;

  // Disable interrupts here
  // Restore FPU context here (if necessary)
  // Enable interrupts here
}

int sign_accel(double accel){
  int sign = 0;

  if (accel > 0) {
    sign = 1;
  }
  else if (accel == 0) {
    sign = 0;
  }
  else {
    sign = -1;
  }
  return sign;
}

void moveByAccel(){
  int dir = sign_accel(linearAccel); 
  float speed = dir * SPEED_MAX;
  float accelStep = 0.0;

  accelStep = (linearAccel) * stepPerRound / (PI * diameter);

  stepper.setAcceleration(abs(accelStep));
  if (dir != 0)
  {
    stepper.spin(speed);
  }
}
