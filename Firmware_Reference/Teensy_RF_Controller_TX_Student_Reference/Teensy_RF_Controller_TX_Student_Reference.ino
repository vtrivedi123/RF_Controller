/*
  Teensy_RF_Controller_TX

  Pin-redacted reference firmware for the ECE 426 Lab 7 design exercise.
  NOT READY TO FLASH: choose pins for your own schematic and PCB first.
  This is the instructor transmitter's behavior, not its MCU pin map.

  Hardware:
  - Teensy 4.0
  - nRF24L01+PA+LNA; choose CE, CSN, and a compatible Teensy SPI bus
  - Two KY-023 joystick modules powered from 3.3 V
  - SSD1306 128x64 I2C OLED at 0x3C by default
  - Optional RC-style debug pulse outputs; not required for Lab 7

  Packet format is intentionally compact for the nRF24L01 32-byte payload limit:
  byte 0-1   uint16_t sequenceNumber
  byte 2-3   int16_t  rollCommand      (-1000 to +1000)
  byte 4-5   int16_t  pitchCommand     (-1000 to +1000)
  byte 6-7   int16_t  yawCommand       (-1000 to +1000)
  byte 8-9   int16_t  throttleCommand  (1000 to 2000 microseconds)
  byte 10    uint8_t  buttons          (bit 0 = right joystick arm switch;
                                        bit 1 = left joystick IMU-cal request;
                                        bit 7 = fail-closed tuner STOP request)
  byte 11    uint8_t  flags            (bits 0-1 = throttle mode: hold/up/down)
  byte 12-13 uint16_t checksum         (CRC-16/CCITT over bytes 0-11)

  Arming and IMU calibration live on the receiver. This sketch reports the
  right and left joystick switches in button bits 0 and 1. Bit 7 is a tuner STOP that explicitly
  disarms the receiver before normal control packets are inhibited. The packet
  layout is unchanged.
*/

// The stick positions now travel in the $T telemetry row, so the old TX,
// debug line is redundant. Anything not starting with '$' is ignored by the
// tuner and merely logged, so turning this back on is harmless but noisy.
#define ENABLE_SERIAL_DEBUG 0
#define ENABLE_RECEIVER_TELEMETRY 1
#define ENABLE_DEBUG_PWM 0
#define ENABLE_OLED 0
#define ENABLE_TX_BATTERY_SENSE 0
#define ENABLE_RF_STATUS_LED 0

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <stddef.h>
#include "NonBlockingUsb.h"

NonBlockingUsb usbProtocol(Serial);

#if ENABLE_DEBUG_PWM
#include <Servo.h>
#endif

// -----------------------------
// Teensy 4.0 pin definitions — intentionally left unassigned for Lab 7.
// Match every selected pin to your own schematic, PCB, and Teensy pinout.
// Then remove this #error only after you have filled and reviewed the map.
// -----------------------------
#error "Lab 7 reference only: assign your own MCU pins and divider values before compiling."
constexpr uint8_t UNASSIGNED_PIN = 255;
constexpr uint8_t NRF_CE_PIN = UNASSIGNED_PIN;
constexpr uint8_t NRF_CSN_PIN = UNASSIGNED_PIN;

constexpr uint8_t ROLL_ADC_PIN = UNASSIGNED_PIN;
constexpr uint8_t PITCH_ADC_PIN = UNASSIGNED_PIN;
constexpr uint8_t THROTTLE_ADC_PIN = UNASSIGNED_PIN;
constexpr uint8_t YAW_ADC_PIN = UNASSIGNED_PIN;
constexpr uint8_t ARM_SW_PIN = UNASSIGNED_PIN;  // right joystick press switch
constexpr uint8_t CAL_SW_PIN = UNASSIGNED_PIN;  // left joystick press switch
constexpr uint8_t TX_BAT_ADC_PIN = UNASSIGNED_PIN;  // optional battery sensing
constexpr uint8_t RF_STATUS_LED_PIN = UNASSIGNED_PIN;  // optional status LED

constexpr uint8_t OLED_SDA_PIN = UNASSIGNED_PIN;  // optional display
constexpr uint8_t OLED_SCL_PIN = UNASSIGNED_PIN;

constexpr uint8_t PWM_THROTTLE_PIN = UNASSIGNED_PIN;  // optional debug only
constexpr uint8_t PWM_YAW_PIN = UNASSIGNED_PIN;
constexpr uint8_t PWM_PITCH_PIN = UNASSIGNED_PIN;
constexpr uint8_t PWM_ROLL_PIN = UNASSIGNED_PIN;

// Use the actual resistor values from your chosen battery-sense divider.
// If you omit that optional block, remove or disable its firmware feature.
constexpr float BATTERY_DIVIDER_TOP_OHMS = 0.0f;
constexpr float BATTERY_DIVIDER_BOTTOM_OHMS = 0.0f;

static_assert(NRF_CE_PIN != UNASSIGNED_PIN && NRF_CSN_PIN != UNASSIGNED_PIN,
              "Choose radio control pins for your own PCB");
static_assert(ROLL_ADC_PIN != UNASSIGNED_PIN && PITCH_ADC_PIN != UNASSIGNED_PIN &&
              YAW_ADC_PIN != UNASSIGNED_PIN && THROTTLE_ADC_PIN != UNASSIGNED_PIN,
              "Choose four analog-capable joystick pins for your own PCB");
static_assert(ARM_SW_PIN != UNASSIGNED_PIN && CAL_SW_PIN != UNASSIGNED_PIN,
              "Choose both joystick-switch pins for your own PCB");
#if ENABLE_OLED
static_assert(OLED_SDA_PIN != UNASSIGNED_PIN && OLED_SCL_PIN != UNASSIGNED_PIN,
              "Choose a compatible I2C pin pair for your display");
#endif
#if ENABLE_TX_BATTERY_SENSE
static_assert(TX_BAT_ADC_PIN != UNASSIGNED_PIN && BATTERY_DIVIDER_TOP_OHMS > 0.0f &&
              BATTERY_DIVIDER_BOTTOM_OHMS > 0.0f,
              "Set your battery-sense ADC pin and actual divider values");
#endif
#if ENABLE_RF_STATUS_LED
static_assert(RF_STATUS_LED_PIN != UNASSIGNED_PIN,
              "Choose your status-LED output pin");
#endif

// -----------------------------
// ADC and joystick calibration
// -----------------------------
constexpr uint8_t ADC_BITS = 12;
constexpr uint16_t ADC_MAX_VALUE = (1u << ADC_BITS) - 1u;  // 4095 at 12-bit
constexpr uint16_t ADC_CENTER_VALUE = ADC_MAX_VALUE / 2u;  // 2047 nominal
constexpr uint8_t ADC_AVERAGING_SAMPLES = 4;
constexpr uint8_t JOYSTICK_CALIBRATION_SAMPLES = 64;
constexpr uint16_t JOYSTICK_CALIBRATION_SETTLE_MS = 750;
constexpr uint16_t JOYSTICK_CALIBRATION_MAX_SPAN = 150;
constexpr uint16_t JOYSTICK_CENTER_RAIL_MARGIN = 300;

constexpr int16_t STICK_COMMAND_LIMIT = 1000;  // roll/pitch/yaw output range
constexpr uint16_t AXIS_DEADBAND_COUNTS = 80;
constexpr uint16_t THROTTLE_RATE_DEADBAND_COUNTS = 110;

struct AxisCalibration {
  uint16_t rawMin;
  uint16_t rawCenter;
  uint16_t rawMax;
  bool reversed;
  uint16_t deadbandCounts;
};

// Tune these after reading actual joystick values on usbProtocol/OLED.
// Set reversed=true if a channel moves the opposite direction from what you expect.
AxisCalibration ROLL_AXIS = {
  0, ADC_CENTER_VALUE, ADC_MAX_VALUE, false, AXIS_DEADBAND_COUNTS
};

AxisCalibration PITCH_AXIS = {
  0, ADC_CENTER_VALUE, ADC_MAX_VALUE, false, AXIS_DEADBAND_COUNTS
};

// Confirm the left joystick's physical yaw direction on your assembled board.
AxisCalibration YAW_AXIS = {
  0, ADC_CENTER_VALUE, ADC_MAX_VALUE, false, AXIS_DEADBAND_COUNTS
};

// Confirm each joystick's physical orientation and axis direction on your board.
AxisCalibration THROTTLE_RATE_AXIS = {
  0, ADC_CENTER_VALUE, ADC_MAX_VALUE, false, THROTTLE_RATE_DEADBAND_COUNTS
};

// -----------------------------
// Throttle Option B behavior
// -----------------------------
constexpr int16_t THROTTLE_MIN_US = 1000;
constexpr int16_t THROTTLE_MID_US = 1500;
constexpr int16_t THROTTLE_MAX_US = 2000;
constexpr int16_t INITIAL_THROTTLE_US = THROTTLE_MIN_US;
constexpr float THROTTLE_RAMP_US_PER_SECOND = 300.0f;

// -----------------------------
// Radio configuration
// -----------------------------
constexpr bool RF24_AUTO_ACK_ENABLED = true;
constexpr uint8_t RADIO_CHANNEL = 76;
constexpr uint8_t RF24_RETRY_DELAY = 3;
constexpr uint8_t RF24_RETRY_COUNT = 5;

// Receiver code must use the same address, channel, data rate, PA level, and packet.
const uint8_t RADIO_ADDRESS[6] = "D426T";

// If RF24_2MBPS is unstable, change this to RF24_1MBPS or RF24_250KBPS on both ends.
constexpr rf24_datarate_e RADIO_DATA_RATE = RF24_2MBPS;

// PA+LNA modules can brown out at high power without solid local decoupling.
// For bench tests use RF24_PA_LOW. For range testing try RF24_PA_HIGH or RF24_PA_MAX.
constexpr rf24_pa_dbm_e RADIO_PA_LEVEL = RF24_PA_LOW;

constexpr uint16_t RADIO_UPDATE_HZ = 100;
constexpr uint32_t RADIO_PERIOD_US = 1000000UL / RADIO_UPDATE_HZ;
constexpr uint8_t CONTROL_BUTTON_ARM_MASK = 0x01u;
constexpr uint8_t CONTROL_BUTTON_CAL_MASK = 0x02u;
constexpr uint8_t CONTROL_BUTTON_REMOTE_STOP_MASK = 0x80u;
constexpr uint8_t REMOTE_STOP_SEND_ATTEMPTS = 3;
constexpr uint8_t TELEMETRY_PROTOCOL_VERSION = 1;
constexpr uint16_t TELEMETRY_SERIAL_RATE_HZ = 25;
constexpr uint32_t TELEMETRY_SERIAL_PERIOD_MS = 1000UL / TELEMETRY_SERIAL_RATE_HZ;
constexpr uint32_t SERIAL_BAUD = 500000;

// -----------------------------
// OLED configuration
// -----------------------------
constexpr uint8_t OLED_I2C_ADDRESS = 0x3C;  // Change to 0x3D if your display needs it.
constexpr int8_t OLED_RESET_PIN = -1;
constexpr uint8_t OLED_WIDTH = 128;
constexpr uint8_t OLED_HEIGHT = 64;
constexpr uint16_t OLED_UPDATE_INTERVAL_MS = 150;

// -----------------------------
// Debug PWM configuration
// -----------------------------
constexpr uint16_t DEBUG_PWM_PERIOD_MS = 20;

// -----------------------------
// Runtime state
// -----------------------------
enum ThrottleMode : uint8_t {
  THROTTLE_HOLD = 0,
  THROTTLE_UP = 1,
  THROTTLE_DOWN = 2
};

struct ControlState {
  uint16_t rawRoll = ADC_CENTER_VALUE;
  uint16_t rawPitch = ADC_CENTER_VALUE;
  uint16_t rawYaw = ADC_CENTER_VALUE;
  uint16_t rawThrottle = ADC_CENTER_VALUE;

  int16_t rollCommand = 0;
  int16_t pitchCommand = 0;
  int16_t yawCommand = 0;
  int16_t throttleRateCommand = 0;

  bool joyButtonPressed = false;
  bool calButtonPressed = false;
  ThrottleMode throttleMode = THROTTLE_HOLD;
};

struct __attribute__((packed)) TxPacket {
  uint16_t sequenceNumber;
  int16_t rollCommand;
  int16_t pitchCommand;
  int16_t yawCommand;
  int16_t throttleCommand;
  uint8_t buttons;
  uint8_t flags;
  uint16_t checksum;
};

struct __attribute__((packed)) TelemetryPacket {
  uint8_t protocolVersion;
  uint8_t flags;
  uint16_t telemetrySequence;
  int16_t rollCentideg;
  int16_t pitchCentideg;
  int16_t gyroXDeciDps;
  int16_t gyroYDeciDps;
  int16_t gyroZDeciDps;
  int16_t rateSetpointDeciDps[3];
  int16_t angleSetpointCentideg[2];
  uint8_t motorQuarterUs[4];
  uint16_t batteryMillivolts;
  uint16_t checksum;
};

// Byte-identical to the flight controller's dedicated wireless attitude
// packet. Its unique 26-byte size keeps size-based nRF dispatch unambiguous.
constexpr uint8_t IMU_TELEMETRY_MAGIC = 0x49;
constexpr uint8_t IMU_TELEMETRY_PROTOCOL_VERSION = 1;
struct __attribute__((packed)) ImuTelemetryPacket {
  uint8_t magic;
  uint8_t protocolVersion;
  uint16_t sequenceNumber;
  int16_t rollCentideg;
  int16_t pitchCentideg;
  int16_t yawCentideg;
  int16_t gyroXDeciDps;
  int16_t gyroYDeciDps;
  int16_t gyroZDeciDps;
  int16_t accelXMilliG;
  int16_t accelYMilliG;
  int16_t accelZMilliG;
  uint16_t flags;
  uint16_t checksum;
};

enum TelemetryFlags : uint8_t {
  TELEMETRY_RADIO_OK = 1u << 0,
  TELEMETRY_IMU_OK = 1u << 1,
  TELEMETRY_TIMER_OK = 1u << 2,
  TELEMETRY_FAILSAFE_ACTIVE = 1u << 3,
  TELEMETRY_STARTUP_SAFETY_CLEARED = 1u << 4,
  TELEMETRY_BATTERY_LOW = 1u << 5,
  TELEMETRY_BATTERY_CRITICAL = 1u << 6,
  TELEMETRY_ANGLE_MODE = 1u << 7
};


// -----------------------------
// Live PID tuning protocol
// -----------------------------
constexpr uint8_t PID_UPDATE_MAGIC = 0xA5;
constexpr uint8_t PID_STATUS_MAGIC = 0x5A;

enum PidCommand : uint8_t {
  PID_COMMAND_SET = 1,
  PID_COMMAND_GET = 2,
  PID_COMMAND_RESET = 3
};

enum PidControllerId : uint8_t {
  PID_CONTROLLER_ROLL = 0,
  PID_CONTROLLER_PITCH = 1,
  PID_CONTROLLER_YAW = 2,
  PID_CONTROLLER_ROLL_ANGLE = 3,
  PID_CONTROLLER_PITCH_ANGLE = 4,
  // Not a controller: carries per-motor trim through the same three float
  // slots. The transmitter only relays it; the receiver does the work.
  PID_CONTROLLER_MOTOR_TRIM = 5,
  PID_CONTROLLER_LEVEL_TRIM = 6
};

enum PidStatusCode : uint8_t {
  PID_STATUS_APPLIED = 0,
  PID_STATUS_CURRENT = 1,
  PID_STATUS_RESET_TO_DEFAULT = 2,
  PID_STATUS_INVALID_PACKET = 3,
  PID_STATUS_INVALID_VALUES = 4,
  PID_STATUS_UNKNOWN_CONTROLLER = 5,
  PID_STATUS_UNKNOWN_COMMAND = 6
};

struct __attribute__((packed)) PidUpdatePacket {
  uint8_t magic;
  uint8_t command;
  uint8_t controllerId;
  uint8_t reserved;
  uint16_t sequenceNumber;
  float kp;
  float ki;
  float kd;
  uint16_t checksum;
};

struct __attribute__((packed)) PidStatusPacket {
  uint8_t magic;
  uint8_t status;
  uint8_t controllerId;
  uint8_t command;
  uint16_t sequenceNumber;
  float kp;
  float ki;
  float kd;
  uint16_t checksum;
};

static_assert(sizeof(PidUpdatePacket) == 20, "PID update packet must be 20 bytes");
static_assert(sizeof(PidStatusPacket) == 20, "PID status packet must be 20 bytes");


// -----------------------------
// PID step-response trace
// -----------------------------
// Must stay byte-identical to the receiver's definition. Downlink packets are
// told apart by payload size, so 24 must remain unique among the packet types.
constexpr uint8_t PID_TRACE_MAGIC = 0x3C;

// The receiver reuses the trace packet's former reserved byte. Zero is the
// legacy rate domain; one is the outer-angle domain. Neither value changes the
// packet size or the meaning of any rate-domain field.
enum PidTraceDomain : uint8_t {
  PID_TRACE_DOMAIN_RATE = 0,
  PID_TRACE_DOMAIN_ANGLE = 1
};

struct __attribute__((packed)) PidTracePacket {
  uint8_t magic;
  uint8_t axisId;
  uint8_t flags;
  uint8_t traceDomain;  // PidTraceDomain; formerly reserved, offset 3
  uint16_t captureId;
  uint16_t sampleIndex;
  uint16_t sampleCount;
  int16_t relativeMs;
  int16_t setpointDeciDps;
  int16_t measuredDeciDps;
  int16_t pTermCentiUs;
  int16_t iTermCentiUs;
  int16_t dTermCentiUs;
  uint16_t checksum;
};

static_assert(sizeof(PidTracePacket) == 24, "PID trace packet must be 24 bytes");
static_assert(offsetof(PidTracePacket, traceDomain) == 3,
              "Trace domain must repurpose the reserved byte");

// Must stay byte-identical to the receiver's definition.
constexpr uint8_t PID_IDENTITY_MAGIC = 0x7E;

struct __attribute__((packed)) PidIdentityPacket {
  uint8_t magic;
  uint8_t nameLen;
  uint32_t descriptorHash;
  char name[14];
  char version[6];
  uint16_t checksum;
};

static_assert(sizeof(PidIdentityPacket) == 28, "PID identity packet must be 28 bytes");
static_assert(sizeof(PidIdentityPacket) != sizeof(TxPacket) &&
              sizeof(PidIdentityPacket) != sizeof(PidStatusPacket) &&
              sizeof(PidIdentityPacket) != sizeof(PidTracePacket) &&
              sizeof(PidIdentityPacket) != sizeof(TelemetryPacket),
              "Identity packet size must be unique so size-based dispatch stays unambiguous");

// Descriptor discovery over nRF24. The browser protocol already supports
// chunked $D lines; these two compact binary packets carry those chunks across
// the 32-byte radio link when the descriptor is not cached yet.
constexpr uint8_t PID_DESCRIPTOR_REQUEST_MAGIC = 0xD1;
constexpr uint8_t PID_DESCRIPTOR_CHUNK_MAGIC = 0xD2;
constexpr uint8_t PID_DESCRIPTOR_DATA_BYTES = 19;
constexpr uint16_t PID_DESCRIPTOR_MAX_CHUNKS = 256;

struct __attribute__((packed)) PidDescriptorRequestPacket {
  uint8_t magic;
  uint8_t reserved;
  uint16_t index;
  uint32_t descriptorHash;
  uint16_t checksum;
};

struct __attribute__((packed)) PidDescriptorChunkPacket {
  uint8_t magic;
  uint16_t index;
  uint16_t total;
  uint8_t length;
  uint32_t descriptorHash;
  char data[PID_DESCRIPTOR_DATA_BYTES];
  uint16_t checksum;
};

static_assert(sizeof(PidDescriptorRequestPacket) == 10, "Descriptor request packet must be 10 bytes");
static_assert(sizeof(PidDescriptorChunkPacket) == 31, "Descriptor chunk packet must be 31 bytes");
static_assert(sizeof(PidDescriptorRequestPacket) != sizeof(TxPacket) &&
              sizeof(PidDescriptorRequestPacket) != sizeof(PidStatusPacket) &&
              sizeof(PidDescriptorRequestPacket) != sizeof(PidTracePacket) &&
              sizeof(PidDescriptorRequestPacket) != sizeof(PidIdentityPacket) &&
              sizeof(PidDescriptorRequestPacket) != sizeof(TelemetryPacket),
              "Descriptor request size must stay unique");
static_assert(sizeof(PidDescriptorChunkPacket) != sizeof(TxPacket) &&
              sizeof(PidDescriptorChunkPacket) != sizeof(PidStatusPacket) &&
              sizeof(PidDescriptorChunkPacket) != sizeof(PidTracePacket) &&
              sizeof(PidDescriptorChunkPacket) != sizeof(PidIdentityPacket) &&
              sizeof(PidDescriptorChunkPacket) != sizeof(TelemetryPacket),
              "Descriptor chunk size must stay unique");

// The commanded rates the flight controller derives from the sticks. The
// transmitter reproduces them so telemetry can carry a setpoint next to each
// measurement, which is what step-response analysis needs.
// MUST MATCH the receiver's MAX_*_RATE_DPS.
constexpr float MAX_ROLL_RATE_DPS = 180.0f;
constexpr float MAX_PITCH_RATE_DPS = 180.0f;
constexpr float MAX_YAW_RATE_DPS = 160.0f;

static_assert(sizeof(TxPacket) <= 32, "TxPacket must fit in the nRF24L01 32-byte payload");
static_assert(sizeof(TxPacket) == 14, "Unexpected TxPacket padding");
static_assert(sizeof(TelemetryPacket) == 32, "TelemetryPacket must be exactly one nRF24L01 payload");
static_assert(sizeof(ImuTelemetryPacket) == 26, "IMU telemetry packet must be 26 bytes");
static_assert(sizeof(ImuTelemetryPacket) != sizeof(TxPacket) &&
              sizeof(ImuTelemetryPacket) != sizeof(PidStatusPacket) &&
              sizeof(ImuTelemetryPacket) != sizeof(PidTracePacket) &&
              sizeof(ImuTelemetryPacket) != sizeof(PidIdentityPacket) &&
              sizeof(ImuTelemetryPacket) != sizeof(PidDescriptorChunkPacket) &&
              sizeof(ImuTelemetryPacket) != sizeof(TelemetryPacket),
              "IMU telemetry size must stay unique for nRF dispatch");
static_assert(sizeof(PidTracePacket) != sizeof(TxPacket) &&
              sizeof(PidTracePacket) != sizeof(PidStatusPacket) &&
              sizeof(PidTracePacket) != sizeof(TelemetryPacket),
              "Trace packet size must be unique so the size-based dispatch stays unambiguous");

RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET_PIN);


bool descriptorTransferActive = false;
// While discovery is active and control transmission is permitted, alternate
// one descriptor request with one ordinary control packet. At the nominal
// 100 Hz radio rate this bounds the control-packet gap to 20 ms, well below the
// receiver's 150 ms failsafe, while still advancing descriptor discovery.
bool auxiliarySentLastSlot = false;
uint32_t descriptorTransferHash = 0;
uint16_t descriptorRequestedIndex = 0;
uint16_t descriptorTransferTotal = 0;
uint32_t descriptorLastProgressMs = 0;
// A descriptor that stops arriving must not strand the transmitter in
// discovery: without this the aircraft dropping out mid-transfer left control
// packets permanently halted, recoverable only by a power cycle.
constexpr uint32_t DESCRIPTOR_STALL_MS = 5000;

#if ENABLE_DEBUG_PWM
Servo pwmThrottle;
Servo pwmYaw;
Servo pwmPitch;
Servo pwmRoll;
#endif

ControlState controls;
TxPacket packet;
TelemetryPacket latestTelemetry = {};
PidUpdatePacket pendingPidPacket = {};
bool pidPacketPending = false;
// Keep a request alive until its matching application response arrives; a
// hardware ACK only confirms RF delivery, not that the receiver processed it.
bool pidResponsePending = false;
uint8_t pidSendAttempts = 0;
uint32_t pidFirstSendMs = 0;
uint32_t pidLastSendMs = 0;
constexpr uint8_t PID_MAX_SEND_ATTEMPTS = 4;
constexpr uint32_t PID_RETRY_INTERVAL_MS = 250;
constexpr uint32_t PID_RESPONSE_TIMEOUT_MS = 1500;
bool pidStatusReplyPending = false;
PidStatusPacket deferredPidStatus = {};
bool tunerStateReplyPending = false;
bool identityReplyPending = false;
bool pidTimeoutReplyPending = false;
uint32_t usbBusyRejectedCount = 0;
uint32_t droppedTraceLines = 0;
uint16_t nextPidSequenceNumber = 1;
String serialCommandBuffer;
PidIdentityPacket lastIdentity = {};
bool haveIdentity = false;

// OUT,STOP first sends an explicit fail-closed control packet, then inhibits
// normal traffic. The receiver failsafe remains the delivery fallback.
bool transmitInhibited = false;
bool remoteStopAcknowledged = false;
uint32_t telemetrySerialPeriodMs = 1000UL / 25;

bool radioOk = false;
bool oledOk = false;
bool lastAckOk = false;
bool haveValidTelemetry = false;
bool telemetryUpdatedSincePrint = false;
bool joystickCalibrationOk = false;
float txBatteryVoltage = 0.0f;

uint16_t nextSequenceNumber = 0;
uint32_t packetCount = 0;
uint32_t failedPacketCount = 0;
uint32_t validTelemetryCount = 0;
uint32_t invalidTelemetryCount = 0;

int16_t throttleCommandUs = INITIAL_THROTTLE_US;
float throttleCommandFloat = static_cast<float>(INITIAL_THROTTLE_US);

uint32_t lastRadioUpdateUs = 0;
uint32_t lastOledUpdateMs = 0;
uint32_t lastBatteryUpdateMs = 0;
uint32_t lastDebugPwmUpdateMs = 0;
uint32_t lastThrottleUpdateUs = 0;
uint32_t lastTelemetrySerialMs = 0;
bool telemetryHeaderPrinted = false;

#if ENABLE_SERIAL_DEBUG
uint32_t lastSerialDebugMs = 0;
bool transmitterHeaderPrinted = false;
constexpr uint16_t TRANSMITTER_SERIAL_RATE_HZ = 10;
constexpr uint16_t SERIAL_DEBUG_INTERVAL_MS =
    1000U / TRANSMITTER_SERIAL_RATE_HZ;
#endif

// -----------------------------
// Utility helpers
// -----------------------------
int32_t clampInt32(int32_t value, int32_t low, int32_t high) {
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

float clampFloat(float value, float low, float high) {
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

int32_t mapLinearClamped(int32_t value, int32_t inMin, int32_t inMax, int32_t outMin, int32_t outMax) {
  if (inMin == inMax) {
    return outMin;
  }

  value = clampInt32(value, min(inMin, inMax), max(inMin, inMax));
  return outMin + ((value - inMin) * (outMax - outMin)) / (inMax - inMin);
}

uint16_t crc16Ccitt(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;

  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if ((crc & 0x8000u) != 0) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021u);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }

  return crc;
}

const char *throttleModeLabel(ThrottleMode mode) {
  switch (mode) {
    case THROTTLE_UP:
      return "UP";
    case THROTTLE_DOWN:
      return "DOWN";
    case THROTTLE_HOLD:
    default:
      return "HOLD";
  }
}

const char *dataRateLabel() {
  switch (RADIO_DATA_RATE) {
    case RF24_2MBPS:
      return "2M";
    case RF24_1MBPS:
      return "1M";
    case RF24_250KBPS:
      return "250K";
    default:
      return "?";
  }
}

const char *paLevelLabel() {
  switch (RADIO_PA_LEVEL) {
    case RF24_PA_MIN:
      return "MIN";
    case RF24_PA_LOW:
      return "LOW";
    case RF24_PA_HIGH:
      return "HIGH";
    case RF24_PA_MAX:
      return "MAX";
    default:
      return "?";
  }
}

// -----------------------------
// Input processing
// -----------------------------
uint16_t readAxisRaw(uint8_t analogPin) {
  return static_cast<uint16_t>(analogRead(analogPin));
}

bool captureJoystickCenters() {
  if (oledOk) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 16);
    display.println(F("RELEASE BOTH STICKS"));
    display.println(F("Calibrating centers"));
    display.display();
  }

  if (usbProtocol) {
    usbProtocol.println(F("CAL,release both sticks"));
  }

  delay(JOYSTICK_CALIBRATION_SETTLE_MS);

  uint32_t sums[4] = {};
  uint16_t minimums[4] = {
      ADC_MAX_VALUE, ADC_MAX_VALUE, ADC_MAX_VALUE, ADC_MAX_VALUE};
  uint16_t maximums[4] = {};

  for (uint8_t sample = 0; sample < JOYSTICK_CALIBRATION_SAMPLES; ++sample) {
    // Logical axis order: confirm this mapping against your own joystick orientation.
    const uint16_t readings[4] = {
        readAxisRaw(ROLL_ADC_PIN),
        readAxisRaw(PITCH_ADC_PIN),
        readAxisRaw(YAW_ADC_PIN),
        readAxisRaw(THROTTLE_ADC_PIN),
    };

    for (uint8_t axis = 0; axis < 4; ++axis) {
      sums[axis] += readings[axis];
      minimums[axis] = min(minimums[axis], readings[axis]);
      maximums[axis] = max(maximums[axis], readings[axis]);
    }
    delay(2);
  }

  ROLL_AXIS.rawCenter =
      static_cast<uint16_t>(sums[0] / JOYSTICK_CALIBRATION_SAMPLES);
  PITCH_AXIS.rawCenter =
      static_cast<uint16_t>(sums[1] / JOYSTICK_CALIBRATION_SAMPLES);
  YAW_AXIS.rawCenter =
      static_cast<uint16_t>(sums[2] / JOYSTICK_CALIBRATION_SAMPLES);
  THROTTLE_RATE_AXIS.rawCenter =
      static_cast<uint16_t>(sums[3] / JOYSTICK_CALIBRATION_SAMPLES);

  const uint16_t centers[4] = {
      ROLL_AXIS.rawCenter,
      PITCH_AXIS.rawCenter,
      YAW_AXIS.rawCenter,
      THROTTLE_RATE_AXIS.rawCenter,
  };

  bool valid = true;
  for (uint8_t axis = 0; axis < 4; ++axis) {
    const uint16_t span = maximums[axis] - minimums[axis];
    if (span > JOYSTICK_CALIBRATION_MAX_SPAN ||
        centers[axis] < JOYSTICK_CENTER_RAIL_MARGIN ||
        centers[axis] > ADC_MAX_VALUE - JOYSTICK_CENTER_RAIL_MARGIN) {
      valid = false;
    }
  }

  if (usbProtocol) {
    usbProtocol.print(F("CAL_RESULT,"));
    usbProtocol.print(valid ? F("OK") : F("FAIL"));
    usbProtocol.print(F(",centers,"));
    usbProtocol.print(centers[0]);
    usbProtocol.print(',');
    usbProtocol.print(centers[1]);
    usbProtocol.print(',');
    usbProtocol.print(centers[2]);
    usbProtocol.print(',');
    usbProtocol.print(centers[3]);
    usbProtocol.print(F(",spans,"));
    usbProtocol.print(maximums[0] - minimums[0]);
    usbProtocol.print(',');
    usbProtocol.print(maximums[1] - minimums[1]);
    usbProtocol.print(',');
    usbProtocol.print(maximums[2] - minimums[2]);
    usbProtocol.print(',');
    usbProtocol.println(maximums[3] - minimums[3]);
  }

  return valid;
}

int16_t normalizeAxisRaw(uint16_t raw, const AxisCalibration &axis) {
  const uint16_t lowerDeadbandEdge =
      (axis.rawCenter > axis.deadbandCounts) ? axis.rawCenter - axis.deadbandCounts : axis.rawMin;
  const uint16_t upperDeadbandEdge =
      (axis.rawCenter + axis.deadbandCounts < axis.rawMax) ? axis.rawCenter + axis.deadbandCounts : axis.rawMax;

  int16_t command = 0;

  if (raw > upperDeadbandEdge) {
    command = static_cast<int16_t>(
        mapLinearClamped(raw, upperDeadbandEdge, axis.rawMax, 0, STICK_COMMAND_LIMIT));
  } else if (raw < lowerDeadbandEdge) {
    command = static_cast<int16_t>(
        mapLinearClamped(raw, axis.rawMin, lowerDeadbandEdge, -STICK_COMMAND_LIMIT, 0));
  }

  if (axis.reversed) {
    command = static_cast<int16_t>(-command);
  }

  return static_cast<int16_t>(clampInt32(command, -STICK_COMMAND_LIMIT, STICK_COMMAND_LIMIT));
}

void readControls() {
  controls.rawRoll = readAxisRaw(ROLL_ADC_PIN);
  controls.rawPitch = readAxisRaw(PITCH_ADC_PIN);
  // Match these logical names to the nets and pins in your own design.
  controls.rawYaw = readAxisRaw(YAW_ADC_PIN);
  controls.rawThrottle = readAxisRaw(THROTTLE_ADC_PIN);

  if (!joystickCalibrationOk) {
    controls.rollCommand = 0;
    controls.pitchCommand = 0;
    controls.yawCommand = 0;
    controls.throttleRateCommand = 0;
    controls.throttleMode = THROTTLE_HOLD;
    controls.joyButtonPressed = false;
    controls.calButtonPressed = false;
    return;
  }

  controls.rollCommand = normalizeAxisRaw(controls.rawRoll, ROLL_AXIS);
  controls.pitchCommand = normalizeAxisRaw(controls.rawPitch, PITCH_AXIS);
  controls.yawCommand = normalizeAxisRaw(controls.rawYaw, YAW_AXIS);
  controls.throttleRateCommand = normalizeAxisRaw(controls.rawThrottle, THROTTLE_RATE_AXIS);

  controls.joyButtonPressed = (digitalRead(ARM_SW_PIN) == LOW);
  controls.calButtonPressed = (digitalRead(CAL_SW_PIN) == LOW);
}

void updateThrottleCommand(uint32_t nowUs) {
  if (!joystickCalibrationOk || transmitInhibited) {
    throttleCommandFloat = static_cast<float>(THROTTLE_MIN_US);
    throttleCommandUs = THROTTLE_MIN_US;
    lastThrottleUpdateUs = nowUs;
    return;
  }
  if (lastThrottleUpdateUs == 0) {
    lastThrottleUpdateUs = nowUs;
  }

  const uint32_t elapsedUs = nowUs - lastThrottleUpdateUs;
  lastThrottleUpdateUs = nowUs;

  if (controls.throttleRateCommand > 0) {
    controls.throttleMode = THROTTLE_UP;
  } else if (controls.throttleRateCommand < 0) {
    controls.throttleMode = THROTTLE_DOWN;
  } else {
    controls.throttleMode = THROTTLE_HOLD;
  }

  const float rateScale =
      static_cast<float>(controls.throttleRateCommand) / static_cast<float>(STICK_COMMAND_LIMIT);
  const float elapsedSeconds = static_cast<float>(elapsedUs) / 1000000.0f;

  throttleCommandFloat += rateScale * THROTTLE_RAMP_US_PER_SECOND * elapsedSeconds;
  throttleCommandFloat = clampFloat(throttleCommandFloat, THROTTLE_MIN_US, THROTTLE_MAX_US);
  throttleCommandUs = static_cast<int16_t>(lroundf(throttleCommandFloat));
}


const char *pidControllerLabel(uint8_t controllerId) {
  switch (controllerId) {
    case PID_CONTROLLER_ROLL: return "ROLL";
    case PID_CONTROLLER_PITCH: return "PITCH";
    case PID_CONTROLLER_YAW: return "YAW";
    case PID_CONTROLLER_ROLL_ANGLE: return "ROLL_ANGLE";
    case PID_CONTROLLER_PITCH_ANGLE: return "PITCH_ANGLE";
    case PID_CONTROLLER_MOTOR_TRIM: return "MOTOR_TRIM";
    case PID_CONTROLLER_LEVEL_TRIM: return "LEVEL_TRIM";
    default: return "UNKNOWN";
  }
}

bool parsePidController(const String &text, uint8_t &controllerId) {
  if (text.equalsIgnoreCase("ROLL")) controllerId = PID_CONTROLLER_ROLL;
  else if (text.equalsIgnoreCase("PITCH")) controllerId = PID_CONTROLLER_PITCH;
  else if (text.equalsIgnoreCase("YAW")) controllerId = PID_CONTROLLER_YAW;
  else if (text.equalsIgnoreCase("ROLL_ANGLE")) controllerId = PID_CONTROLLER_ROLL_ANGLE;
  else if (text.equalsIgnoreCase("PITCH_ANGLE")) controllerId = PID_CONTROLLER_PITCH_ANGLE;
  else if (text.equalsIgnoreCase("MOTOR_TRIM")) controllerId = PID_CONTROLLER_MOTOR_TRIM;
  else if (text.equalsIgnoreCase("LEVEL_TRIM")) controllerId = PID_CONTROLLER_LEVEL_TRIM;
  else return false;
  return true;
}

// No default arguments here. The Arduino builder copies defaults into the
// prototype it generates and inserts above this definition, and the two
// specifications then clash: "default argument given for parameter 3".
// GET and RESET ignore the gains, so they pass explicit zeros instead.
void preparePidPacket(uint8_t command, uint8_t controllerId,
                      float kp, float ki, float kd) {
  pendingPidPacket.magic = PID_UPDATE_MAGIC;
  pendingPidPacket.command = command;
  pendingPidPacket.controllerId = controllerId;
  pendingPidPacket.reserved = 0;
  pendingPidPacket.sequenceNumber = nextPidSequenceNumber++;
  pendingPidPacket.kp = kp;
  pendingPidPacket.ki = ki;
  pendingPidPacket.kd = kd;
  pendingPidPacket.checksum = 0;
  pendingPidPacket.checksum = crc16Ccitt(
      reinterpret_cast<const uint8_t *>(&pendingPidPacket),
      sizeof(PidUpdatePacket) - sizeof(pendingPidPacket.checksum));
  pidPacketPending = true;
  pidResponsePending = true;
  pidSendAttempts = 0;
}

/* Screen the text before converting: String::toFloat() returns 0.0 for junk,
   which would turn a typo into "set this gain to zero". */
bool parseNumberField(const String &field, float &out) {
  String t = field;
  t.trim();
  if (t.length() == 0) return false;
  char *end = nullptr;
  out = strtof(t.c_str(), &end);
  return end != t.c_str() && *end == '\0' && isfinite(out);
}

/* PID Tuner Protocol v1 over USB. The transmitter is a relay: it does not hold
   the descriptor, because that belongs to the flight controller and is read
   over that board's own USB port. Here it only forwards commands and turns
   radio packets back into protocol lines. */
void printTunerOutputState() {
  if (!usbProtocol.canQueue(512)) {
    tunerStateReplyPending = true;
    return;
  }
  tunerStateReplyPending = false;
  if (transmitInhibited && remoteStopAcknowledged)
    usbProtocol.println(F("$S,STATE,stopped=1,can_resume=1,reason=receiver acknowledged explicit STOP; release arm button before a fresh low-throttle arm"));
  else if (transmitInhibited)
    usbProtocol.println(F("$S,STATE,stopped=1,can_resume=1,reason=STOP not acknowledged; receiver failsafe is the fallback and re-arm still requires release"));
  else if (descriptorTransferActive)
    usbProtocol.println(F("$S,STATE,stopped=0,can_resume=0,reason=reading descriptor over radio; control packets remain active"));
  else
    usbProtocol.println(F("$S,STATE,stopped=0,can_resume=0,reason="));
}

/* Discovery shares radio slots with ordinary controls. It never changes the
   explicit transmit-inhibit state: starting discovery while flying therefore
   cannot manufacture a receiver failsafe, while starting it after OUT,STOP
   cannot release that stop. */
void beginDescriptorTransfer(uint32_t hash) {
  descriptorTransferActive = true;
  // Keep the existing control/auxiliary slot reservation across discovery.
  descriptorTransferHash = hash;
  descriptorRequestedIndex = 0;
  descriptorTransferTotal = 0;
  descriptorLastProgressMs = millis();
}

void endDescriptorTransfer() {
  descriptorTransferActive = false;
}

bool sendRemoteStopPacket();

void processUsbCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  String f[8];
  uint8_t n = 0;
  int start = 0;
  while (n < 8) {
    const int comma = line.indexOf(',', start);
    if (comma < 0) { f[n++] = line.substring(start); break; }
    f[n++] = line.substring(start, comma);
    start = comma + 1;
  }

  // STOP must remain actionable even if the host has stopped reading. Other
  // commands reserve reply space before they can change any state. Rejections
  // are counted and reported when the host drains the queue again.
  const bool isStop = f[0].equalsIgnoreCase("OUT") && n >= 2 &&
                      f[1].equalsIgnoreCase("STOP");
  if (!isStop && !usbProtocol.canQueue(6144)) {
    ++usbBusyRejectedCount;
    return;
  }

  if (f[0].equalsIgnoreCase("SYS")) {
    if (n >= 2 && f[1].equalsIgnoreCase("ID")) {
      if (haveIdentity) printIdentity(lastIdentity);
      else usbProtocol.println(F("$E,NO_IDENTITY_YET_WAITING_FOR_AIRCRAFT"));
      printTunerOutputState();
      return;
    }
    if (n >= 2 && f[1].equalsIgnoreCase("DESCRIBE")) {
      if (!haveIdentity) {
        usbProtocol.println(F("$E,NO_IDENTITY_YET_WAITING_FOR_AIRCRAFT"));
        printTunerOutputState();
        return;
      }
      if (descriptorTransferActive && descriptorTransferHash == lastIdentity.descriptorHash) {
        usbProtocol.println(F("$K,DESCRIPTOR,RADIO,IN_PROGRESS"));
        printTunerOutputState();
        return;
      }
      beginDescriptorTransfer(lastIdentity.descriptorHash);
      usbProtocol.println(F("$K,DESCRIPTOR,RADIO,START"));
      printTunerOutputState();
      return;
    }
    if (n >= 2 && f[1].equalsIgnoreCase("PING")) { usbProtocol.println(F("$K,PONG")); return; }
    usbProtocol.println(F("$E,BAD_SYS"));
    return;
  }

  if (f[0].equalsIgnoreCase("PID")) {
    if (n < 3) { usbProtocol.println(F("$E,BAD_PID")); return; }
    if (pidPacketPending || pidResponsePending || pidStatusReplyPending) {
      // Retain the request until the receiver's application-level reply, not
      // merely its RF delivery ACK. Never overwrite an unconfirmed command.
      usbProtocol.println(F("$E,BUSY_PREVIOUS_COMMAND_NOT_CONFIRMED_YET"));
      return;
    }
    uint8_t controllerId = 0;
    if (!parsePidController(f[2], controllerId)) { usbProtocol.println(F("$E,UNKNOWN_CONTROLLER")); return; }

    if (f[1].equalsIgnoreCase("GET")) { preparePidPacket(PID_COMMAND_GET, controllerId, 0.0f, 0.0f, 0.0f); return; }
    if (f[1].equalsIgnoreCase("RESET")) { preparePidPacket(PID_COMMAND_RESET, controllerId, 0.0f, 0.0f, 0.0f); return; }
    if (f[1].equalsIgnoreCase("SET")) {
      // Named param=value pairs, in any order, so adding a parameter later
      // cannot break an older tuner.
      float kp = NAN, ki = NAN, kd = NAN;
      for (uint8_t i = 3; i < n; i++) {
        const int eq = f[i].indexOf('=');
        if (eq < 0) continue;
        const String key = f[i].substring(0, eq);
        float v;
        if (!parseNumberField(f[i].substring(eq + 1), v)) {
          usbProtocol.println(F("$E,MALFORMED_VALUE"));
          return;
        }
        if (key.equalsIgnoreCase("kp")) kp = v;
        else if (key.equalsIgnoreCase("ki")) ki = v;
        else if (key.equalsIgnoreCase("kd")) kd = v;
      }
      // Level trim has roll and pitch only. Its reserved third payload value is
      // always zero; all actual PID controllers still require a full triple.
      if (controllerId == PID_CONTROLLER_LEVEL_TRIM && isnan(kd)) kd = 0.0f;
      if (isnan(kp) || isnan(ki) || isnan(kd)) {
        // The radio packet carries all three together, so a partial set cannot
        // be expressed. Ask the tuner to send the full triple.
        usbProtocol.println(F("$E,SET_REQUIRES_KP_KI_KD"));
        return;
      }
      preparePidPacket(PID_COMMAND_SET, controllerId, kp, ki, kd);
      return;
    }
    usbProtocol.println(F("$E,BAD_PID"));
    return;
  }

  if (f[0].equalsIgnoreCase("CAL")) {
    // Calibration needs the aircraft level, still and disarmed, which is a
    // bench job on the flight controller's own USB port. Refusing clearly beats
    // relaying something that cannot be done safely in flight.
    usbProtocol.println(F("$C,FAIL,NOT_SUPPORTED"));
    usbProtocol.println(F("$E,CALIBRATE_OVER_USB_ON_THE_FLIGHT_CONTROLLER"));
    return;
  }

  if (f[0].equalsIgnoreCase("OUT") && n >= 2 && f[1].equalsIgnoreCase("STOP")) {
    // Abandon discovery, explicitly disarm the receiver, then inhibit normal
    // control packets. If RF delivery fails, the receiver's failsafe is the
    // backup and also latches a required arm-button release.
    if (descriptorTransferActive) endDescriptorTransfer();
    throttleCommandFloat = static_cast<float>(THROTTLE_MIN_US);
    throttleCommandUs = THROTTLE_MIN_US;
    remoteStopAcknowledged = sendRemoteStopPacket();
    transmitInhibited = true;
    printTunerOutputState();
    return;
  }
  if (f[0].equalsIgnoreCase("OUT") && n >= 2 && f[1].equalsIgnoreCase("RESUME")) {
    // Resuming during discovery would clear the inhibit while the radio is
    // still busy, leaving the reported state disagreeing with reality.
    if (descriptorTransferActive) endDescriptorTransfer();
    lastThrottleUpdateUs = micros();
    transmitInhibited = false;
    remoteStopAcknowledged = false;
    printTunerOutputState();
    return;
  }

  if (f[0].equalsIgnoreCase("TELEM") && n >= 3 && f[1].equalsIgnoreCase("RATE")) {
    float hz;
    if (!parseNumberField(f[2], hz)) { usbProtocol.println(F("$E,BAD_RATE")); return; }
    if (hz < 1.0f || hz > 100.0f || floorf(hz) != hz) { usbProtocol.println(F("$E,RATE_REJECTED")); return; }
    const uint32_t requested = static_cast<uint32_t>(hz);
    if (requested == 0 || requested > 100) { usbProtocol.println(F("$E,RATE_REJECTED")); return; }
    telemetrySerialPeriodMs = 1000UL / requested;
    usbProtocol.print(F("$K,TELEM,RATE,"));
    usbProtocol.println(requested);
    return;
  }

  usbProtocol.println(F("$E,UNKNOWN_COMMAND"));
}

void updateUsbCommandInput() {
  static bool discardLine = false;
  uint16_t budget = 64;
  while (budget-- > 0 && usbProtocol.available() > 0) {
    const char c = static_cast<char>(usbProtocol.read());
    if (discardLine) {
      if (c == '\n' || c == '\r') discardLine = false;
      continue;
    }
    if (c == '\n' || c == '\r') {
      if (serialCommandBuffer.length() > 0) {
        processUsbCommand(serialCommandBuffer);
        serialCommandBuffer = "";
      }
    } else if (serialCommandBuffer.length() < 120) {
      serialCommandBuffer += c;
    } else {
      serialCommandBuffer = "";
      discardLine = true;
      usbProtocol.println(F("PID_ERROR,LINE_TOO_LONG"));
    }
  }
}

bool validatePidStatusPacket(const PidStatusPacket &candidate) {
  if (candidate.magic != PID_STATUS_MAGIC) return false;
  PidStatusPacket copy = candidate;
  const uint16_t receivedChecksum = copy.checksum;
  copy.checksum = 0;
  return receivedChecksum == crc16Ccitt(
      reinterpret_cast<const uint8_t *>(&copy),
      sizeof(PidStatusPacket) - sizeof(copy.checksum));
}

bool validatePidIdentityPacket(const PidIdentityPacket &candidate) {
  if (candidate.magic != PID_IDENTITY_MAGIC) return false;
  PidIdentityPacket copy = candidate;
  const uint16_t received = copy.checksum;
  copy.checksum = 0;
  return received == crc16Ccitt(reinterpret_cast<const uint8_t *>(&copy),
                                sizeof(PidIdentityPacket) - sizeof(copy.checksum));
}


bool validateDescriptorChunkPacket(const PidDescriptorChunkPacket &candidate) {
  if (candidate.magic != PID_DESCRIPTOR_CHUNK_MAGIC) return false;
  if (candidate.length > PID_DESCRIPTOR_DATA_BYTES) return false;
  if (candidate.total == 0 || candidate.total > PID_DESCRIPTOR_MAX_CHUNKS) return false;
  if (candidate.index >= candidate.total) return false;
  PidDescriptorChunkPacket copy = candidate;
  const uint16_t received = copy.checksum;
  copy.checksum = 0;
  return received == crc16Ccitt(reinterpret_cast<const uint8_t *>(&copy),
                                sizeof(PidDescriptorChunkPacket) - sizeof(copy.checksum));
}

void handleDescriptorChunk(const PidDescriptorChunkPacket &chunk) {
  if (!descriptorTransferActive) return;
  if (chunk.descriptorHash != descriptorTransferHash) return;
  if (chunk.index != descriptorRequestedIndex) return;
  // Leave the index unchanged under pressure so the next request retrieves
  // this same chunk. Descriptor bytes are never silently skipped.
  if (!usbProtocol.canQueue(512)) return;

  if (descriptorTransferTotal == 0) descriptorTransferTotal = chunk.total;
  if (chunk.total != descriptorTransferTotal) {
    usbProtocol.println(F("$E,DESCRIPTOR_TOTAL_CHANGED"));
    endDescriptorTransfer();
    printTunerOutputState();
    return;
  }

  usbProtocol.print(F("$D,"));
  usbProtocol.print(chunk.index);
  usbProtocol.print(',');
  usbProtocol.print(chunk.total);
  usbProtocol.print(',');
  usbProtocol.write(reinterpret_cast<const uint8_t *>(chunk.data), chunk.length);
  usbProtocol.println();

  descriptorLastProgressMs = millis();
  ++descriptorRequestedIndex;
  if (descriptorRequestedIndex >= descriptorTransferTotal) {
    endDescriptorTransfer();
    usbProtocol.println(F("$K,DESCRIPTOR,RADIO,COMPLETE"));
    printTunerOutputState();
  }
}

void printIdentity(const PidIdentityPacket &id) {
  lastIdentity = id;
  haveIdentity = true;
  if (!usbProtocol.canQueue(256)) {
    identityReplyPending = true;
    return;
  }
  identityReplyPending = false;
  char name[sizeof(id.name) + 1] = {0}, version[sizeof(id.version) + 1] = {0};
  memcpy(name, id.name, sizeof(id.name));
  memcpy(version, id.version, sizeof(id.version));
  char hex[9];
  snprintf(hex, sizeof(hex), "%08lx", (unsigned long)id.descriptorHash);
  usbProtocol.print(F("$I,"));
  usbProtocol.print(name);
  usbProtocol.print(',');
  usbProtocol.print(version);
  usbProtocol.print(',');
  usbProtocol.println(hex);
}

bool validatePidTracePacket(const PidTracePacket &candidate) {
  if (candidate.magic != PID_TRACE_MAGIC) return false;
  if (candidate.traceDomain != PID_TRACE_DOMAIN_RATE &&
      candidate.traceDomain != PID_TRACE_DOMAIN_ANGLE) return false;
  if ((candidate.traceDomain == PID_TRACE_DOMAIN_RATE &&
       candidate.axisId > PID_CONTROLLER_YAW) ||
      (candidate.traceDomain == PID_TRACE_DOMAIN_ANGLE &&
       (candidate.axisId < PID_CONTROLLER_ROLL_ANGLE ||
        candidate.axisId > PID_CONTROLLER_PITCH_ANGLE))) return false;
  PidTracePacket copy = candidate;
  const uint16_t receivedChecksum = copy.checksum;
  copy.checksum = 0;
  return receivedChecksum == crc16Ccitt(
      reinterpret_cast<const uint8_t *>(&copy),
      sizeof(PidTracePacket) - sizeof(copy.checksum));
}

void printPidTrace(const PidTracePacket &trace) {
  if (!usbProtocol || usbProtocol.availableForWrite() < 256) {
    ++droppedTraceLines;
    return;
  }
  // PID_TRACE,captureId,axis,index,count,relativeMs,setpoint,measured,p,i,d,flags
  // Keep the established 12-field line shape. The axis name identifies the
  // angle domain, while the domain byte has already been validated on RF.
  const bool angleDomain = trace.traceDomain == PID_TRACE_DOMAIN_ANGLE;
  usbProtocol.print(F("PID_TRACE,"));
  usbProtocol.print(trace.captureId);
  usbProtocol.print(',');
  usbProtocol.print(pidControllerLabel(trace.axisId));
  usbProtocol.print(',');
  usbProtocol.print(trace.sampleIndex);
  usbProtocol.print(',');
  usbProtocol.print(trace.sampleCount);
  usbProtocol.print(',');
  usbProtocol.print(trace.relativeMs);
  usbProtocol.print(',');
  usbProtocol.print(static_cast<float>(trace.setpointDeciDps) /
                   (angleDomain ? 100.0f : 10.0f),
               angleDomain ? 2 : 1);
  usbProtocol.print(',');
  usbProtocol.print(static_cast<float>(trace.measuredDeciDps) /
                   (angleDomain ? 100.0f : 10.0f),
               angleDomain ? 2 : 1);
  usbProtocol.print(',');
  usbProtocol.print(static_cast<float>(trace.pTermCentiUs) /
                   (angleDomain ? 10.0f : 100.0f),
               2);
  usbProtocol.print(',');
  usbProtocol.print(static_cast<float>(trace.iTermCentiUs) /
                   (angleDomain ? 10.0f : 100.0f),
               2);
  usbProtocol.print(',');
  usbProtocol.print(static_cast<float>(trace.dTermCentiUs) /
                   (angleDomain ? 10.0f : 100.0f),
               2);
  usbProtocol.print(',');
  usbProtocol.println(trace.flags);
}

/* Lowercase ids, matching the controller ids in the flight controller's
   descriptor. The uppercase labels above are for human-readable logging. */
const char *pidControllerId(uint8_t controllerId) {
  switch (controllerId) {
    case PID_CONTROLLER_ROLL: return "roll";
    case PID_CONTROLLER_PITCH: return "pitch";
    case PID_CONTROLLER_YAW: return "yaw";
    case PID_CONTROLLER_ROLL_ANGLE: return "roll_angle";
    case PID_CONTROLLER_PITCH_ANGLE: return "pitch_angle";
    // Must match the descriptor id exactly. The tuner matches $P replies to
    // controller cards by this string, so a miss leaves the card waiting for a
    // confirmation that never arrives.
    case PID_CONTROLLER_MOTOR_TRIM: return "motor_trim";
    case PID_CONTROLLER_LEVEL_TRIM: return "level_trim";
    default: return "unknown";
  }
}

void printPidStatus(const PidStatusPacket &status) {
  if (!usbProtocol.canQueue(256)) {
    deferredPidStatus = status;
    pidStatusReplyPending = true;
    return;
  }
  pidStatusReplyPending = false;
  // $P,<controller>,<code>,<param>=<value>...
  usbProtocol.print(F("$P,"));
  usbProtocol.print(pidControllerId(status.controllerId));
  usbProtocol.print(',');
  usbProtocol.print(status.status);
  usbProtocol.print(F(",kp="));
  usbProtocol.print(status.kp, 6);
  usbProtocol.print(F(",ki="));
  usbProtocol.print(status.ki, 6);
  usbProtocol.print(F(",kd="));
  usbProtocol.println(status.kd, 6);
}

// -----------------------------
// Radio and packet handling
// -----------------------------
bool initRadio() {
  SPI.begin();

  if (!radio.begin()) {
    return false;
  }

  radio.setChannel(RADIO_CHANNEL);
  radio.setAddressWidth(5);
  radio.setAutoAck(RF24_AUTO_ACK_ENABLED);
  radio.setRetries(RF24_RETRY_DELAY, RF24_RETRY_COUNT);
  radio.setDataRate(RADIO_DATA_RATE);
  radio.setPALevel(RADIO_PA_LEVEL);
  radio.setCRCLength(RF24_CRC_16);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.openWritingPipe(RADIO_ADDRESS);
  radio.stopListening();

  return radio.isChipConnected();
}

void fillTransmitPacket(TxPacket &outPacket) {
  outPacket.sequenceNumber = nextSequenceNumber++;
  outPacket.rollCommand = controls.rollCommand;
  outPacket.pitchCommand = controls.pitchCommand;
  outPacket.yawCommand = controls.yawCommand;
  outPacket.throttleCommand = throttleCommandUs;
  outPacket.buttons = (controls.joyButtonPressed ? CONTROL_BUTTON_ARM_MASK : 0x00u) |
                      (controls.calButtonPressed ? CONTROL_BUTTON_CAL_MASK : 0x00u);
  outPacket.flags = static_cast<uint8_t>(controls.throttleMode) & 0x03u;
  outPacket.checksum = 0;

  outPacket.checksum = crc16Ccitt(
      reinterpret_cast<const uint8_t *>(&outPacket),
      sizeof(TxPacket) - sizeof(outPacket.checksum));
}

bool validateTelemetryPacket(const TelemetryPacket &candidate) {
  if (candidate.protocolVersion != TELEMETRY_PROTOCOL_VERSION) {
    return false;
  }

  const uint16_t expectedChecksum = crc16Ccitt(
      reinterpret_cast<const uint8_t *>(&candidate),
      sizeof(TelemetryPacket) - sizeof(candidate.checksum));
  return candidate.checksum == expectedChecksum;
}

bool validateImuTelemetryPacket(const ImuTelemetryPacket &candidate) {
  if (candidate.magic != IMU_TELEMETRY_MAGIC ||
      candidate.protocolVersion != IMU_TELEMETRY_PROTOCOL_VERSION) return false;
  const uint16_t expectedChecksum = crc16Ccitt(
      reinterpret_cast<const uint8_t *>(&candidate),
      sizeof(ImuTelemetryPacket) - sizeof(candidate.checksum));
  return candidate.checksum == expectedChecksum;
}

void printImuTelemetry(const ImuTelemetryPacket &sample) {
  if (!usbProtocol || usbProtocol.availableForWrite() < 256) return;
  constexpr float STANDARD_GRAVITY_MPS2 = 9.80665f;
  usbProtocol.print(F("IMU,NRF_TX,"));
  usbProtocol.print(sample.rollCentideg / 100.0f, 2);
  usbProtocol.print(','); usbProtocol.print(sample.pitchCentideg / 100.0f, 2);
  usbProtocol.print(','); usbProtocol.print(sample.yawCentideg / 100.0f, 2);
  usbProtocol.print(','); usbProtocol.print(sample.gyroXDeciDps / 10.0f, 1);
  usbProtocol.print(','); usbProtocol.print(sample.gyroYDeciDps / 10.0f, 1);
  usbProtocol.print(','); usbProtocol.print(sample.gyroZDeciDps / 10.0f, 1);
  usbProtocol.print(','); usbProtocol.print(sample.accelXMilliG * STANDARD_GRAVITY_MPS2 / 1000.0f, 3);
  usbProtocol.print(','); usbProtocol.print(sample.accelYMilliG * STANDARD_GRAVITY_MPS2 / 1000.0f, 3);
  usbProtocol.print(','); usbProtocol.println(sample.accelZMilliG * STANDARD_GRAVITY_MPS2 / 1000.0f, 3);
}

void handlePidStatus(const PidStatusPacket &status) {
  // Internal GET polls clock out ACK payloads while STOP inhibits pilot
  // packets. Their CURRENT replies must not masquerade as confirmation
  // of an outstanding SET/RESET. Late duplicate replies are ignored.
  if (!pidResponsePending ||
      status.sequenceNumber != pendingPidPacket.sequenceNumber ||
      status.controllerId != pendingPidPacket.controllerId ||
      status.command != pendingPidPacket.command) return;
  pidResponsePending = false;
  pidPacketPending = false;
  printPidStatus(status);
}

void readTelemetryAckPayload() {
  while (radio.available()) {
    const uint8_t payloadSize = radio.getDynamicPayloadSize();
    if (payloadSize == 0 || payloadSize > 32) {
      radio.flush_rx();
      ++invalidTelemetryCount;
      return;
    }

    if (payloadSize == sizeof(TelemetryPacket)) {
      TelemetryPacket candidate;
      radio.read(&candidate, sizeof(candidate));
      if (!validateTelemetryPacket(candidate)) {
        ++invalidTelemetryCount;
        continue;
      }
      latestTelemetry = candidate;
      haveValidTelemetry = true;
      telemetryUpdatedSincePrint = true;
      ++validTelemetryCount;
    } else if (payloadSize == sizeof(ImuTelemetryPacket)) {
      ImuTelemetryPacket sample = {};
      radio.read(&sample, sizeof(sample));
      if (validateImuTelemetryPacket(sample)) {
        printImuTelemetry(sample);
      } else {
        ++invalidTelemetryCount;
      }
    } else if (payloadSize == sizeof(PidStatusPacket)) {
      PidStatusPacket status = {};
      radio.read(&status, sizeof(status));
      if (validatePidStatusPacket(status)) {
        handlePidStatus(status);
      } else {
        usbProtocol.println(F("PID_ERROR,BAD_STATUS_CRC"));
      }
    } else if (payloadSize == sizeof(PidTracePacket)) {
      PidTracePacket trace = {};
      radio.read(&trace, sizeof(trace));
      if (validatePidTracePacket(trace)) {
        printPidTrace(trace);
      } else {
        usbProtocol.println(F("PID_ERROR,BAD_TRACE_CRC"));
      }
    } else if (payloadSize == sizeof(PidIdentityPacket)) {
      PidIdentityPacket id = {};
      radio.read(&id, sizeof(id));
      if (validatePidIdentityPacket(id)) {
        printIdentity(id);
      }
    } else if (payloadSize == sizeof(PidDescriptorChunkPacket)) {
      PidDescriptorChunkPacket chunk = {};
      radio.read(&chunk, sizeof(chunk));
      if (validateDescriptorChunkPacket(chunk)) {
        handleDescriptorChunk(chunk);
      } else {
        usbProtocol.println(F("$E,BAD_DESCRIPTOR_CHUNK"));
      }
    } else {
      uint8_t discardedPayload[32];
      radio.read(discardedPayload, payloadSize);
      ++invalidTelemetryCount;
    }
  }
}

bool sendPacket(const TxPacket &outPacket) {
  if (!radioOk) {
    lastAckOk = false;
    ++packetCount;
    ++failedPacketCount;
    return false;
  }

  const bool ok = radio.write(&outPacket, sizeof(outPacket));
  lastAckOk = ok;
  ++packetCount;

  if (!ok) {
    ++failedPacketCount;
  } else {
    readTelemetryAckPayload();
  }

  return ok;
}

bool sendRemoteStopPacket() {
  TxPacket stopPacket = {};
  fillTransmitPacket(stopPacket);
  stopPacket.rollCommand = 0;
  stopPacket.pitchCommand = 0;
  stopPacket.yawCommand = 0;
  stopPacket.throttleCommand = THROTTLE_MIN_US;

  // ARM stays high in the STOP frame. The receiver cannot mistake this frame
  // for the required post-STOP button release; only a later ordinary frame
  // reflecting the physical switch in its released state can clear the latch.
  stopPacket.buttons = CONTROL_BUTTON_ARM_MASK | CONTROL_BUTTON_REMOTE_STOP_MASK;
  stopPacket.flags = static_cast<uint8_t>(THROTTLE_HOLD) & 0x03u;
  stopPacket.checksum = 0;
  stopPacket.checksum = crc16Ccitt(
      reinterpret_cast<const uint8_t *>(&stopPacket),
      sizeof(TxPacket) - sizeof(stopPacket.checksum));

  for (uint8_t attempt = 0; attempt < REMOTE_STOP_SEND_ATTEMPTS; ++attempt) {
    if (sendPacket(stopPacket)) return true;
  }
  return false;
}

bool sendPidPacket(const PidUpdatePacket &outPacket) {
  if (!radioOk) {
    usbProtocol.print(F("PID_TX,"));
    usbProtocol.print(outPacket.sequenceNumber);
    usbProtocol.println(F(",NO_RADIO"));
    return false;
  }

  const bool ok = radio.write(&outPacket, sizeof(outPacket));
  lastAckOk = ok;
  ++packetCount;
  if (!ok) {
    ++failedPacketCount;
  } else {
    readTelemetryAckPayload();
  }

  usbProtocol.print(F("PID_TX,"));
  usbProtocol.print(outPacket.sequenceNumber);
  usbProtocol.print(',');
  usbProtocol.println(ok ? F("OK") : F("MISS"));
  return ok;
}

void servicePidRequestTimeout() {
  if (!pidResponsePending || pidSendAttempts == 0) return;
  const uint32_t nowMs = millis();
  if (nowMs - pidFirstSendMs >= PID_RESPONSE_TIMEOUT_MS) {
    pidPacketPending = false;
    pidResponsePending = false;
    pidTimeoutReplyPending = true;
    return;
  }
  if (!pidPacketPending && pidSendAttempts < PID_MAX_SEND_ATTEMPTS &&
      nowMs - pidLastSendMs >= PID_RETRY_INTERVAL_MS) {
    pidPacketPending = true;
  }
}

void notePidSendAttempt() {
  const uint32_t nowMs = millis();
  if (pidSendAttempts == 0) pidFirstSendMs = nowMs;
  pidLastSendMs = nowMs;
  ++pidSendAttempts;
}

bool sendStoppedPidPoll() {
  if (!radioOk || !transmitInhibited || !pidResponsePending) return false;
  // GET is a read-only uplink understood by every existing RX, including the
  // unchanged I2C build. It neither arms nor refreshes the control failsafe.
  PidUpdatePacket poll = {};
  poll.magic = PID_UPDATE_MAGIC;
  poll.command = PID_COMMAND_GET;
  poll.controllerId = pendingPidPacket.controllerId;
  poll.sequenceNumber = pendingPidPacket.sequenceNumber;
  poll.checksum = crc16Ccitt(reinterpret_cast<const uint8_t *>(&poll),
                            sizeof(poll) - sizeof(poll.checksum));
  const bool ok = radio.write(&poll, sizeof(poll));
  lastAckOk = ok;
  ++packetCount;
  if (!ok) ++failedPacketCount;
  else readTelemetryAckPayload();
  return ok;
}

void serviceDeferredUsbReplies() {
  if (!usbProtocol) return;
  if (pidStatusReplyPending && usbProtocol.canQueue(256)) {
    printPidStatus(deferredPidStatus);
  }
  if (pidTimeoutReplyPending && usbProtocol.canQueue(256)) {
    usbProtocol.println(F("$E,PID_RESPONSE_TIMEOUT"));
    pidTimeoutReplyPending = false;
  }
  if (tunerStateReplyPending && usbProtocol.canQueue(512)) printTunerOutputState();
  if (identityReplyPending && usbProtocol.canQueue(256)) printIdentity(lastIdentity);
  if (usbBusyRejectedCount != 0 && usbProtocol.canQueue(512)) {
    usbProtocol.print(F("$E,USB_BUSY_RETRY,rejected="));
    usbProtocol.println(usbBusyRejectedCount);
    usbBusyRejectedCount = 0;
  }
  if (droppedTraceLines != 0 && usbProtocol.availableForWrite() >= 256) {
    // A missing sample index already prevents complete-capture analysis; make
    // the source of that missing trace explicit in the app's technical log.
    usbProtocol.print(F("PID_TRACE_DROPPED,USB_BACKPRESSURE,count="));
    usbProtocol.println(droppedTraceLines);
    droppedTraceLines = 0;
  }
}


bool sendDescriptorRequest() {
  if (!descriptorTransferActive) return false;

  // Checked before the radio guard on purpose. A link that has gone down is
  // exactly when discovery must be able to give up, and testing this after
  // "radioOk" would make the one case that strands the transmitter the one
  // case the timeout never sees.
  if ((uint32_t)(millis() - descriptorLastProgressMs) > DESCRIPTOR_STALL_MS) {
    // Abandoning discovery is what lets control packets - and so the aircraft
    // - come back. The browser retries on its own schedule.
    usbProtocol.println(F("$E,DESCRIPTOR_RADIO_TIMEOUT_ABORTED"));
    endDescriptorTransfer();
    printTunerOutputState();
    return false;
  }
  if (!radioOk) return false;

  PidDescriptorRequestPacket request = {};
  request.magic = PID_DESCRIPTOR_REQUEST_MAGIC;
  request.index = descriptorRequestedIndex;
  request.descriptorHash = descriptorTransferHash;
  request.checksum = 0;
  request.checksum = crc16Ccitt(reinterpret_cast<const uint8_t *>(&request),
                                sizeof(PidDescriptorRequestPacket) - sizeof(request.checksum));

  const bool ok = radio.write(&request, sizeof(request));
  lastAckOk = ok;
  ++packetCount;
  if (!ok) ++failedPacketCount;
  else readTelemetryAckPayload();
  return ok;
}

// -----------------------------
// OLED display
// -----------------------------
bool initOled() {
  Wire.setSDA(OLED_SDA_PIN);
  Wire.setSCL(OLED_SCL_PIN);
  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS)) {
    return false;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("TX Controller"));
  display.println(F("Teensy RF TX"));
  display.println(F("OLED OK"));
  display.display();

  return true;
}

void printSignedToOled(int16_t value) {
  if (value >= 0) {
    display.print('+');
  }
  display.print(value);
}

uint8_t radioSuccessPercent() {
  const uint32_t goodPackets =
      (packetCount >= failedPacketCount) ? packetCount - failedPacketCount : 0;
  return (packetCount == 0)
             ? 0
             : static_cast<uint8_t>((goodPackets * 100UL) / packetCount);
}

void updateOledDisplay() {
  if (!oledOk) {
    return;
  }

  const uint8_t successPercent = radioSuccessPercent();

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print(joystickCalibrationOk ? F("TX Controller") : F("JOY CAL FAIL"));

  display.setCursor(0, 8);
  display.print(F("RF:"));
  display.print(radioOk ? F("OK ") : F("FAIL "));
  display.print(dataRateLabel());
  display.print(F(" PA:"));
  display.print(paLevelLabel());

  display.setCursor(0, 16);
  display.print(F("ACK:"));
  display.print(lastAckOk ? F("OK ") : F("MISS "));
  display.print(F("Suc:"));
  display.print(successPercent);
  display.print('%');

  display.setCursor(0, 24);
  display.print(F("R:"));
  printSignedToOled(controls.rollCommand);
  display.print(F(" P:"));
  printSignedToOled(controls.pitchCommand);

  display.setCursor(0, 32);
  display.print(F("Y:"));
  printSignedToOled(controls.yawCommand);
  display.print(F(" T:"));
  display.print(throttleCommandUs);

  display.setCursor(0, 40);
  display.print(F("Thr:"));
  display.print(throttleModeLabel(controls.throttleMode));
  display.print(F(" A:"));
  display.print(controls.joyButtonPressed ? F("DN") : F("UP"));
  display.print(F(" C:"));
  display.print(controls.calButtonPressed ? F("DN") : F("UP"));

  display.setCursor(0, 48);
  display.print(F("Bat:"));
  display.print(txBatteryVoltage, 1);
  display.print(F("V P:"));
  display.print(packetCount);

  display.setCursor(0, 56);
  display.print(F("Fail:"));
  display.print(failedPacketCount);
  display.print(F(" Seq:"));
  display.print(packet.sequenceNumber);

  display.display();
}

// -----------------------------
// Optional debug PWM outputs
// -----------------------------
int16_t signedCommandToPulseUs(int16_t command) {
  return static_cast<int16_t>(
      mapLinearClamped(command, -STICK_COMMAND_LIMIT, STICK_COMMAND_LIMIT, THROTTLE_MIN_US, THROTTLE_MAX_US));
}

void initDebugPwm() {
#if ENABLE_DEBUG_PWM
  pwmThrottle.attach(PWM_THROTTLE_PIN, THROTTLE_MIN_US, THROTTLE_MAX_US);
  pwmYaw.attach(PWM_YAW_PIN, THROTTLE_MIN_US, THROTTLE_MAX_US);
  pwmPitch.attach(PWM_PITCH_PIN, THROTTLE_MIN_US, THROTTLE_MAX_US);
  pwmRoll.attach(PWM_ROLL_PIN, THROTTLE_MIN_US, THROTTLE_MAX_US);

  pwmThrottle.writeMicroseconds(throttleCommandUs);
  pwmYaw.writeMicroseconds(THROTTLE_MID_US);
  pwmPitch.writeMicroseconds(THROTTLE_MID_US);
  pwmRoll.writeMicroseconds(THROTTLE_MID_US);
#endif
}

void updateDebugPwmOutputs() {
#if ENABLE_DEBUG_PWM
  pwmThrottle.writeMicroseconds(throttleCommandUs);
  pwmYaw.writeMicroseconds(signedCommandToPulseUs(controls.yawCommand));
  pwmPitch.writeMicroseconds(signedCommandToPulseUs(controls.pitchCommand));
  pwmRoll.writeMicroseconds(signedCommandToPulseUs(controls.rollCommand));
#endif
}

// -----------------------------
// usbProtocol debug
// -----------------------------
void printTransmitterHeader() {
#if ENABLE_SERIAL_DEBUG
  usbProtocol.println(
      F("TX_HEADER,tx_ms,sequence,raw_roll,raw_pitch,raw_yaw,raw_throttle,"
        "center_roll,center_pitch,center_yaw,center_throttle,roll_cmd,pitch_cmd,"
        "yaw_cmd,throttle_us,throttle_mode,button_pressed,joy_cal_ok,radio_ok,"
        "ack_ok,success_pct,packet_count,failed_packets"));
#endif
}

void updateSerialDebug() {
#if ENABLE_SERIAL_DEBUG
  if (!usbProtocol || usbProtocol.availableForWrite() < 512) {
    return;
  }

  const uint32_t nowMs = millis();
  if (nowMs - lastSerialDebugMs < SERIAL_DEBUG_INTERVAL_MS) {
    return;
  }
  lastSerialDebugMs = nowMs;

  if (!transmitterHeaderPrinted) {
    printTransmitterHeader();
    transmitterHeaderPrinted = true;
  }

  usbProtocol.print(F("TX,"));
  usbProtocol.print(nowMs);
  usbProtocol.print(',');
  usbProtocol.print(packet.sequenceNumber);
  usbProtocol.print(',');
  usbProtocol.print(controls.rawRoll);
  usbProtocol.print(',');
  usbProtocol.print(controls.rawPitch);
  usbProtocol.print(',');
  usbProtocol.print(controls.rawYaw);
  usbProtocol.print(',');
  usbProtocol.print(controls.rawThrottle);
  usbProtocol.print(',');
  usbProtocol.print(ROLL_AXIS.rawCenter);
  usbProtocol.print(',');
  usbProtocol.print(PITCH_AXIS.rawCenter);
  usbProtocol.print(',');
  usbProtocol.print(YAW_AXIS.rawCenter);
  usbProtocol.print(',');
  usbProtocol.print(THROTTLE_RATE_AXIS.rawCenter);
  usbProtocol.print(',');
  usbProtocol.print(controls.rollCommand);
  usbProtocol.print(',');
  usbProtocol.print(controls.pitchCommand);
  usbProtocol.print(',');
  usbProtocol.print(controls.yawCommand);
  usbProtocol.print(',');
  usbProtocol.print(throttleCommandUs);
  usbProtocol.print(',');
  usbProtocol.print(throttleModeLabel(controls.throttleMode));
  usbProtocol.print(',');
  usbProtocol.print(controls.joyButtonPressed ? 1 : 0);
  usbProtocol.print(',');
  usbProtocol.print(joystickCalibrationOk ? 1 : 0);
  usbProtocol.print(',');
  usbProtocol.print(radioOk ? 1 : 0);
  usbProtocol.print(',');
  usbProtocol.print(lastAckOk ? 1 : 0);
  usbProtocol.print(',');
  usbProtocol.print(radioSuccessPercent());
  usbProtocol.print(',');
  usbProtocol.print(packetCount);
  usbProtocol.print(',');
  usbProtocol.print(failedPacketCount);
  usbProtocol.println();
#endif
}

// -----------------------------
// Receiver telemetry over USB usbProtocol
// -----------------------------
/* No header line: the tuner learns the field layout from the flight
   controller's descriptor, not from a CSV header. */

void updateTelemetrySerial() {
#if ENABLE_RECEIVER_TELEMETRY
  if (!haveValidTelemetry || !telemetryUpdatedSincePrint || !usbProtocol ||
      usbProtocol.availableForWrite() < 512) {
    return;
  }

  const uint32_t nowMs = millis();
  if (nowMs - lastTelemetrySerialMs < telemetrySerialPeriodMs) {
    return;
  }
  lastTelemetrySerialMs = nowMs;
  telemetryUpdatedSincePrint = false;

  // $T rows, in the exact field order of the flight controller's descriptor.
  //
  // Setpoints come from the flight controller in the ACK telemetry packet. That
  // matters in angle mode, where the rate targets are produced by the outer
  // loop and cannot be reconstructed faithfully from the transmitter sticks.
  const uint8_t flags = latestTelemetry.flags;
  usbProtocol.print(F("$T,"));
  usbProtocol.print(nowMs * 1000UL);                                   // t (us)
  usbProtocol.print(',');
  usbProtocol.print(latestTelemetry.gyroXDeciDps / 10.0f, 1);          // gx
  usbProtocol.print(',');
  usbProtocol.print(latestTelemetry.rateSetpointDeciDps[0] / 10.0f, 1);// spx
  usbProtocol.print(',');
  usbProtocol.print(latestTelemetry.gyroYDeciDps / 10.0f, 1);          // gy
  usbProtocol.print(',');
  usbProtocol.print(latestTelemetry.rateSetpointDeciDps[1] / 10.0f, 1);// spy
  usbProtocol.print(',');
  usbProtocol.print(latestTelemetry.gyroZDeciDps / 10.0f, 1);          // gz
  usbProtocol.print(',');
  usbProtocol.print(latestTelemetry.rateSetpointDeciDps[2] / 10.0f, 1);// spz
  for (uint8_t i = 0; i < 4; ++i) {                               // m1..m4
    usbProtocol.print(',');
    usbProtocol.print(1000u + static_cast<uint16_t>(latestTelemetry.motorQuarterUs[i]) * 4u);
  }
  usbProtocol.print(',');
  usbProtocol.print(controls.rollCommand);                             // sr
  usbProtocol.print(',');
  usbProtocol.print(controls.pitchCommand);                            // sp
  usbProtocol.print(',');
  usbProtocol.print(controls.yawCommand);                              // sy
  usbProtocol.print(',');
  usbProtocol.print(throttleCommandUs);                                // thr
  usbProtocol.print(',');
  usbProtocol.print(latestTelemetry.rollCentideg / 100.0f, 2);         // roll_deg
  usbProtocol.print(',');
  usbProtocol.print(latestTelemetry.angleSetpointCentideg[0] / 100.0f, 2); // roll_deg_sp
  usbProtocol.print(',');
  usbProtocol.print(latestTelemetry.pitchCentideg / 100.0f, 2);        // pitch_deg
  usbProtocol.print(',');
  usbProtocol.print(latestTelemetry.angleSetpointCentideg[1] / 100.0f, 2); // pitch_deg_sp
  usbProtocol.print(',');
  usbProtocol.print((flags & TELEMETRY_ANGLE_MODE) ? 1 : 0);            // angle_mode
  usbProtocol.print(',');
  usbProtocol.print(latestTelemetry.batteryMillivolts / 1000.0f, 2);   // vbat
  usbProtocol.print(',');
  usbProtocol.print((flags & TELEMETRY_FAILSAFE_ACTIVE) ? 1 : 0);      // fs
  usbProtocol.print(',');
  usbProtocol.print((flags & TELEMETRY_BATTERY_LOW) ? 1 : 0);          // lowbat
  usbProtocol.print(',');
  usbProtocol.println((flags & TELEMETRY_RADIO_OK) ? 0 : 1);           // link lost
#endif
}

// -----------------------------
// Arduino lifecycle
// -----------------------------
void setup() {
#if ENABLE_SERIAL_DEBUG || ENABLE_RECEIVER_TELEMETRY
  usbProtocol.begin(SERIAL_BAUD);
#endif

  pinMode(ARM_SW_PIN, INPUT_PULLUP);
  pinMode(CAL_SW_PIN, INPUT_PULLUP);
#if ENABLE_RF_STATUS_LED
  pinMode(RF_STATUS_LED_PIN, OUTPUT);
  digitalWriteFast(RF_STATUS_LED_PIN, LOW);
#endif

  analogReadResolution(ADC_BITS);
  analogReadAveraging(ADC_AVERAGING_SAMPLES);

  initDebugPwm();
#if ENABLE_OLED
  oledOk = initOled();
#else
  oledOk = false;
#endif
  joystickCalibrationOk = captureJoystickCenters();
  radioOk = initRadio();
#if ENABLE_TX_BATTERY_SENSE
  txBatteryVoltage = (static_cast<float>(analogRead(TX_BAT_ADC_PIN)) / ADC_MAX_VALUE) *
                     3.3f * ((BATTERY_DIVIDER_TOP_OHMS + BATTERY_DIVIDER_BOTTOM_OHMS) /
                             BATTERY_DIVIDER_BOTTOM_OHMS);
#endif

  readControls();
  updateThrottleCommand(micros());
  fillTransmitPacket(packet);
  updateDebugPwmOutputs();
  updateOledDisplay();

  lastRadioUpdateUs = micros();
  lastOledUpdateMs = millis();
  lastBatteryUpdateMs = millis();
  lastDebugPwmUpdateMs = millis();
}

void loop() {
  usbProtocol.pump(64);
  serviceDeferredUsbReplies();
  updateUsbCommandInput();
  servicePidRequestTimeout();
  const uint32_t nowUs = micros();

  if (nowUs - lastRadioUpdateUs >= RADIO_PERIOD_US) {
    if (nowUs - lastRadioUpdateUs > RADIO_PERIOD_US * 4UL) {
      lastRadioUpdateUs = nowUs;
    } else {
      lastRadioUpdateUs += RADIO_PERIOD_US;
    }

    readControls();
    updateThrottleCommand(nowUs);
    // Reserve at least every other slot for real controls, even during a
    // continuous stream of PID commands. Auxiliary traffic cannot feed RX's
    // control failsafe, and must never starve the pilot's commands.
    if (!transmitInhibited &&
        (auxiliarySentLastSlot || (!descriptorTransferActive && !pidPacketPending))) {
      fillTransmitPacket(packet);
      sendPacket(packet);
      auxiliarySentLastSlot = false;
    } else if (descriptorTransferActive) {
      sendDescriptorRequest();
      auxiliarySentLastSlot = true;
    } else if (pidPacketPending) {
      const PidUpdatePacket packetToSend = pendingPidPacket;
      pidPacketPending = false;
      notePidSendAttempt();
      sendPidPacket(packetToSend);
      auxiliarySentLastSlot = true;
    } else if (transmitInhibited && pidResponsePending) {
      sendStoppedPidPoll();
      auxiliarySentLastSlot = true;
    } else {
      lastAckOk = false;  // Explicit STOP still inhibits ordinary controls.
      auxiliarySentLastSlot = false;
    }
    updateSerialDebug();
#if ENABLE_RF_STATUS_LED
    digitalWriteFast(RF_STATUS_LED_PIN, radioOk && lastAckOk ? HIGH : LOW);
#endif
  }

  updateTelemetrySerial();

  const uint32_t nowMs = millis();

#if ENABLE_TX_BATTERY_SENSE
  if (nowMs - lastBatteryUpdateMs >= 200) {
    lastBatteryUpdateMs = nowMs;
    txBatteryVoltage = (static_cast<float>(analogRead(TX_BAT_ADC_PIN)) / ADC_MAX_VALUE) *
                       3.3f * ((BATTERY_DIVIDER_TOP_OHMS + BATTERY_DIVIDER_BOTTOM_OHMS) /
                               BATTERY_DIVIDER_BOTTOM_OHMS);
  }
#endif

  if (nowMs - lastDebugPwmUpdateMs >= DEBUG_PWM_PERIOD_MS) {
    lastDebugPwmUpdateMs += DEBUG_PWM_PERIOD_MS;
    updateDebugPwmOutputs();
  }

  if (nowMs - lastOledUpdateMs >= OLED_UPDATE_INTERVAL_MS) {
    lastOledUpdateMs += OLED_UPDATE_INTERVAL_MS;
    updateOledDisplay();
  }
  usbProtocol.pump(64);
}
