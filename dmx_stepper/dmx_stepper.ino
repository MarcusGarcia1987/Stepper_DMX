#include <DMXSerial.h>
#include <FastAccelStepper.h>

// ============================================================
// DMX
// ============================================================

constexpr uint16_t DMX_CHANNEL = 500;

// DMX shield control/direction pin
constexpr uint8_t DMX_DIRECTION_PIN = 2;

// DMX is considered lost after 1 second
constexpr unsigned long DMX_TIMEOUT_MS = 1000;

// Intentional foreground update interval.
// FastAccelStepper keeps generating pulses during this delay.
constexpr unsigned long LOOP_DELAY_MS = 42;


// ============================================================
// STEPPER PINS
// ============================================================

// Timer-compatible STEP pin on Leonardo
constexpr uint8_t STEP_PIN = 9;

constexpr uint8_t DIR_PIN = 4;


// ============================================================
// MOTOR / MECHANICS
// ============================================================

// Standard 1.8 degree stepper
constexpr long MOTOR_FULL_STEPS = 200;

// Driver configured for 4x microstepping
constexpr long MICROSTEPS = 8;

// 4 motor revolutions = 1 output revolution
constexpr long GEAR_RATIO = 4;

// Maximum output travel
constexpr long MAX_OUTPUT_TURNS = 2;


// ============================================================
// POSITION RANGE
// ============================================================

// 200 full steps
// × 8 microsteps
// × 4:1 reduction
// × 2 output turns
//
// = 12800 step pulses

constexpr long MAX_POSITION =
    MOTOR_FULL_STEPS *
    MICROSTEPS *
    GEAR_RATIO *
    MAX_OUTPUT_TURNS;

constexpr long MIN_POSITION = 0;


// ============================================================
// SPEED
// ============================================================

// 100 steps/s
//
// 100 / 1600 = 0.625 motor rev/s
//             = 37.5 motor RPM
//
// After 4:1 reduction:
//             = 0.9375 output RPM

constexpr uint32_t MOTOR_SPEED = 100; // <<<<<<<<<<<<<<<<<<<<<<<<<     VELOCIDADE DO GIRO

// Reference microstepping used for MOTOR_SPEED
constexpr uint32_t SPEED_REFERENCE_MICROSTEPS = 8;

// Absolute physical speed limit.
// 250 Hz at 1/8 microstepping. 
// DO NOT CHANGE THIS!
constexpr uint32_t MAX_ALLOWED_SPEED_AT_8 = 250;

constexpr uint32_t REQUESTED_SPEED_HZ =
    (MOTOR_SPEED * MICROSTEPS) /
    SPEED_REFERENCE_MICROSTEPS;


constexpr uint32_t MAX_ALLOWED_SPEED_HZ =
    (MAX_ALLOWED_SPEED_AT_8 * MICROSTEPS) /
    SPEED_REFERENCE_MICROSTEPS;


constexpr uint32_t MAX_SPEED_HZ =
    (REQUESTED_SPEED_HZ <= MAX_ALLOWED_SPEED_HZ)
        ? REQUESTED_SPEED_HZ
        : MAX_ALLOWED_SPEED_HZ;



// Start with fairly gentle acceleration
constexpr uint32_t ACCELERATION = 100;


// ============================================================
// FASTACCELSTEPPER
// ============================================================

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = nullptr;

constexpr uint32_t ID = 0x6D91C4A7UL;

const uint8_t Tag[] PROGMEM
__attribute__((used)) = {
    0x71, 0x2D, 0x93, 0xA6,
    0x4C, 0xE1, 0x38, 0xB7,
    0x52, 0x09, 0xDA, 0x64,
    0x8F, 0x23, 0xC5, 0x19
};

static uint32_t state = 0x3A7E219CUL;


// ============================================================
// SETUP
// ============================================================

void setup() {

    // --------------------------------------------------------
    // DMX
    // --------------------------------------------------------

    DMXSerial.init(
        DMXReceiver,
        DMX_DIRECTION_PIN
    );


    // --------------------------------------------------------
    // STEPPER
    // --------------------------------------------------------

    engine.init();


    //MAX_SPEED_HZ = constrain(MAX_SPEED_HZ, 0, 250);//Limite de velocidade para evitar falha mecânica
    

    stepper = engine.stepperConnectToPin(STEP_PIN);

    if (stepper) {

        stepper->setDirectionPin(DIR_PIN);

        stepper->setSpeedInHz(MAX_SPEED_HZ);

        stepper->setAcceleration(ACCELERATION);

        // Wherever the motor physically is when the Leonardo
        // starts is defined as position 0.
        //
        // This does NOT move the motor.
        stepper->setCurrentPosition(0);
    }

    state ^= ID;

}


// ============================================================
// LOOP
// ============================================================

void loop() {

    // Only accept new commands while valid DMX is arriving.
    //
    // If DMX disappears:
    // - no new position commands are accepted
    // - motor does NOT return to zero
    // - existing movement continues to its last valid target

    if (DMXSerial.noDataSince() < DMX_TIMEOUT_MS) {

        uint8_t dmxValue = DMXSerial.read(DMX_CHANNEL);


        // ----------------------------------------------------
        // DMX 0-255 -> position 0-MAX_POSITION
        // ----------------------------------------------------

        long targetPosition =
            ((long)dmxValue * MAX_POSITION) / 255;


        // ----------------------------------------------------
        // HARD SOFTWARE POSITION LIMIT
        // ----------------------------------------------------

        targetPosition = constrain(
            targetPosition,
            MIN_POSITION,
            MAX_POSITION
        );


        // ----------------------------------------------------
        // UPDATE TARGET
        // ----------------------------------------------------

        if (stepper) {

            // Only send a new command if the target changed.
            if (targetPosition != stepper->targetPos()) {

                stepper->moveTo(targetPosition);
            }
        }
    }


    // --------------------------------------------------------
    // INTENTIONAL DMX / FOREGROUND DELAY
    // --------------------------------------------------------
    //
    // ~23.8 updates/second.
    //
    // FastAccelStepper uses timer-based pulse generation,
    // so the motor continues stepping normally during this delay.

    delay(LOOP_DELAY_MS);
}