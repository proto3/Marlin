/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 *
 * About Marlin
 *
 * This firmware is a mashup between Sprinter and grbl.
 *  - https://github.com/kliment/Sprinter
 *  - https://github.com/simen/grbl/tree
 *
 * It has preliminary support for Matthew Roberts advance algorithm
 *  - http://reprap.org/pipermail/reprap-dev/2011-May/003323.html
 */

#include "Marlin.h"

#if ENABLED(BEZIER_CURVE_SUPPORT)
  #include "planner_bezier.h"
#endif

#include "ultralcd.h"
#include "planner.h"
#include "stepper.h"
#include "endstops.h"
#include "temperature.h"
#include "cardreader.h"
#include "configuration_store.h"
#include "language.h"
#include "pins_arduino.h"
#include "math.h"
#include "duration_t.h"
#include "types.h"
#include "state.h"
#include "plasma.h"
#include "torch_height_control.h"

#if ENABLED(USE_WATCHDOG)
  #include "watchdog.h"
#endif

#if HAS_SERVOS
  #include "servo.h"
#endif

#if HAS_DIGIPOTSS
  #include <SPI.h>
#endif

#if ENABLED(DAC_STEPPER_CURRENT)
  #include "stepper_dac.h"
#endif

#if ENABLED(EXPERIMENTAL_I2CBUS)
  #include "twibus.h"
#endif

/**
 * Look here for descriptions of G-codes:
 *  - http://linuxcnc.org/handbook/gcode/g-code.html
 *  - http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes
 *
 * Help us document these G-codes online:
 *  - https://github.com/MarlinFirmware/Marlin/wiki/G-Code-in-Marlin
 *  - http://reprap.org/wiki/G-code
 *
 * -----------------
 * Implemented Codes
 * -----------------
 *
 * "G" Codes
 *
 * G0  -> G1
 * G1  - Coordinated Movement X Y Z E
 * G2  - CW ARC
 * G3  - CCW ARC
 * G4  - Dwell S<seconds> or P<milliseconds>
 * G5  - Cubic B-spline with XYZE destination and IJPQ offsets
 * G12 - Clean tool
 * G20 - Set input units to inches
 * G21 - Set input units to millimeters
 * G28 - Home one or more axes
 * G29 - Detailed Z probe, probes the bed at 3 or more points.  Will fail if you haven't homed yet.
 * G31 - Dock sled (Z_PROBE_SLED only)
 * G32 - Undock sled (Z_PROBE_SLED only)
 * G90 - Use Absolute Coordinates
 * G91 - Use Relative Coordinates
 * G92 - Set current position to coordinates given
 *
 * "M" Codes
 *
 * M0   - Unconditional stop - Wait for user to press a button on the LCD (Only if ULTRA_LCD is enabled)
 * M3   - Plasma ignition sequence (probe, arc, pierce, THC on)
 * M5   - Plasma stop sequence (stop and retract)
 * M75  - Start the print job timer
 * M76  - Pause the print job timer
 * M77  - Stop the print job timer
 * M78  - Show statistical information about the print jobs
 * M110 - Set the current line number
 * M117 - Display a message on the controller screen
 * M300 - Play beep sound S<frequency Hz> P<duration ms>
 * M400 - Finish all moves
 */

#if ENABLED(SDSUPPORT)
  CardReader card;
#endif

#if ENABLED(EXPERIMENTAL_I2CBUS)
  TWIBus i2c;
#endif

bool Running = true;

uint8_t marlin_debug_flags = DEBUG_NONE;

float current_position[NUM_AXIS] = { 0.0 };
static float destination[NUM_AXIS] = { 0.0 };
bool axis_known_position[3] = { false };
bool axis_homed[3] = { false };

static long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;

static char command_queue[BUFSIZE][MAX_CMD_SIZE];
static char* cmd, *cmd_args;
static uint8_t cmd_queue_index_r = 0,
               cmd_queue_index_w = 0,
               commands_in_queue = 0;

#if ENABLED(INCH_MODE_SUPPORT)
  float linear_unit_factor = 1.0;
#endif

/**
 * Feed rates are often configured with mm/m
 * but the planner and stepper like mm/s units.
 */
const float homing_feedrate_mm_m[] = {
    HOMING_FEEDRATE_XY, HOMING_FEEDRATE_XY,
  HOMING_FEEDRATE_Z, 0
};
static float feedrate_mm_m = 1500.0, saved_feedrate_mm_m;
int feedrate_percentage = 100, saved_feedrate_percentage;

bool axis_relative_modes[] = AXIS_RELATIVE_MODES;

// The distance that XYZ has been offset by G92. Reset by G28.
float position_shift[3] = { 0 };

// This offset is added to the configured home position.
// Set by M206, M428, or menu item. Saved to EEPROM.
float home_offset[3] = { 0 };

// Software Endstops. Default to configured limits.
float sw_endstop_min[3] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };
float sw_endstop_max[3] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };

// Relative Mode. Enable with G91, disable with G90.
static bool relative_mode = false;

const char errormagic[] PROGMEM = "Error:";
const char echomagic[] PROGMEM = "echo:";
const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z'};

static int serial_count = 0;

// GCode parameter pointer used by code_seen(), code_value_float(), etc.
static char* seen_pointer;

// Next Immediate GCode Command pointer. NULL if none.
const char* queued_commands_P = NULL;

const int sensitive_pins[] = SENSITIVE_PINS; ///< Sensitive pin list for M42

// Inactivity shutdown
millis_t previous_cmd_ms = 0;
static millis_t max_inactive_time = 0;
static millis_t stepper_inactive_time = (DEFAULT_STEPPER_DEACTIVE_TIME) * 1000UL;

// Print Job Timer
#if ENABLED(PRINTCOUNTER)
  PrintCounter print_job_timer = PrintCounter();
#else
  Stopwatch print_job_timer = Stopwatch();
#endif

// Buzzer - I2C on the LCD or a BEEPER_PIN
#if ENABLED(LCD_USE_I2C_BUZZER)
  #define BUZZ(d,f) lcd_buzz(d, f)
#elif HAS_BUZZER
  Buzzer buzzer;
  #define BUZZ(d,f) buzzer.tone(d, f)
#else
  #define BUZZ(d,f) NOOP
#endif

#define PLANNER_XY_FEEDRATE() (min(planner.max_feedrate_mm_s[X_AXIS], planner.max_feedrate_mm_s[Y_AXIS]))

#if   defined(XY_PROBE_SPEED)
  #define XY_PROBE_FEEDRATE_MM_M XY_PROBE_SPEED
#else
  #define XY_PROBE_FEEDRATE_MM_M MMS_TO_MMM(PLANNER_XY_FEEDRATE())
#endif

#if ENABLED(Z_DUAL_ENDSTOPS) && 1
  float z_endstop_adj = 0;
#endif

#if HAS_Z_SERVO_ENDSTOP
  const int z_servo_angle[2] = Z_SERVO_ANGLES;
#endif



#if ENABLED(ULTIPANEL) && HAS_POWER_SWITCH
  bool powersupply =
    #if ENABLED(PS_DEFAULT_OFF)
      false
    #else
      true
    #endif
  ;
#endif

  static bool home_all_axis = true;






static bool send_ok[BUFSIZE];

#if HAS_SERVOS
  Servo servo[NUM_SERVOS];
  #define MOVE_SERVO(I, P) servo[I].move(P)
  #if HAS_Z_SERVO_ENDSTOP
    #define DEPLOY_Z_SERVO() MOVE_SERVO(Z_ENDSTOP_SERVO_NR, z_servo_angle[0])
    #define STOW_Z_SERVO() MOVE_SERVO(Z_ENDSTOP_SERVO_NR, z_servo_angle[1])
  #endif
#endif

#ifdef CHDK
  millis_t chdkHigh = 0;
  boolean chdkActive = false;
#endif


#if ENABLED(HOST_KEEPALIVE_FEATURE)
  static MarlinBusyState busy_state = NOT_BUSY;
  static millis_t next_busy_signal_ms = 0;
  uint8_t host_keepalive_interval = DEFAULT_KEEPALIVE_INTERVAL;
  #define KEEPALIVE_STATE(n) do{ busy_state = n; }while(0)
#else
  #define host_keepalive() ;
  #define KEEPALIVE_STATE(n) ;
#endif // HOST_KEEPALIVE_FEATURE

/**
 * ***************************************************************************
 * ******************************** FUNCTIONS ********************************
 * ***************************************************************************
 */

void stop();

void process_command(char* cmd);
void prepare_move_to_destination();
void set_current_from_steppers_for_axis(AxisEnum axis);

#if ENABLED(ARC_SUPPORT)
  void plan_arc(float target[NUM_AXIS], float* offset, uint8_t clockwise);
#endif

#if ENABLED(BEZIER_CURVE_SUPPORT)
  void plan_cubic_move(const float offset[4]);
#endif

void serial_echopair_P(const char* s_P, char v)          { serialprintPGM(s_P); SERIAL_CHAR(v); }
void serial_echopair_P(const char* s_P, int v)           { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char* s_P, long v)          { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char* s_P, float v)         { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char* s_P, double v)        { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char* s_P, unsigned long v) { serialprintPGM(s_P); SERIAL_ECHO(v); }

static void report_current_position();

/**
 * sync_plan_position
 * Set planner / stepper positions to the cartesian current_position.
 * The stepper code translates these coordinates into step units.
 * Allows translation between steps and millimeters for cartesian & core robots
 */
inline void sync_plan_position() {
  planner.set_position_mm(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);
}

  #define SYNC_PLAN_POSITION_KINEMATIC() sync_plan_position()

#if ENABLED(SDSUPPORT)
  #include "SdFatUtil.h"
  int freeMemory() { return SdFatUtil::FreeRam(); }
#else
extern "C" {
  extern unsigned int __bss_end;
  extern unsigned int __heap_start;
  extern void* __brkval;

  int freeMemory() {
    int free_memory;
    if ((int)__brkval == 0)
      free_memory = ((int)&free_memory) - ((int)&__bss_end);
    else
      free_memory = ((int)&free_memory) - ((int)__brkval);
    return free_memory;
  }
}
#endif //!SDSUPPORT

#if ENABLED(DIGIPOT_I2C)
  extern void digipot_i2c_set_current(int channel, float current);
  extern void digipot_i2c_init();
#endif

/**
 * Inject the next "immediate" command, when possible.
 * Return true if any immediate commands remain to inject.
 */
static bool drain_queued_commands_P() {
  if (queued_commands_P != NULL) {
    size_t i = 0;
    char c, cmd[30];
    strncpy_P(cmd, queued_commands_P, sizeof(cmd) - 1);
    cmd[sizeof(cmd) - 1] = '\0';
    while ((c = cmd[i]) && c != '\n') i++; // find the end of this gcode command
    cmd[i] = '\0';
    if (enqueue_and_echo_command(cmd)) {   // success?
      if (c)                               // newline char?
        queued_commands_P += i + 1;        // advance to the next command
      else
        queued_commands_P = NULL;          // nul char? no more commands
    }
  }
  return (queued_commands_P != NULL);      // return whether any more remain
}

/**
 * Record one or many commands to run from program memory.
 * Aborts the current queue, if any.
 * Note: drain_queued_commands_P() must be called repeatedly to drain the commands afterwards
 */
void enqueue_and_echo_commands_P(const char* pgcode) {
  queued_commands_P = pgcode;
  drain_queued_commands_P(); // first command executed asap (when possible)
}

void clear_command_queue() {
  cmd_queue_index_r = cmd_queue_index_w;
  commands_in_queue = 0;
}

/**
 * Once a new command is in the ring buffer, call this to commit it
 */
inline void _commit_command(bool say_ok) {
  send_ok[cmd_queue_index_w] = say_ok;
  cmd_queue_index_w = (cmd_queue_index_w + 1) % BUFSIZE;
  commands_in_queue++;
}

/**
 * Copy a command directly into the main command buffer, from RAM.
 * Returns true if successfully adds the command
 */
inline bool _enqueuecommand(const char* cmd, bool say_ok=false) {
  if (*cmd == ';' || commands_in_queue >= BUFSIZE) return false;
  strcpy(command_queue[cmd_queue_index_w], cmd);
  _commit_command(say_ok);
  return true;
}

void enqueue_and_echo_command_now(const char* cmd) {
  while (!enqueue_and_echo_command(cmd)) idle();
}

/**
 * Enqueue with Serial Echo
 */
bool enqueue_and_echo_command(const char* cmd, bool say_ok/*=false*/) {
  if (_enqueuecommand(cmd, say_ok)) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM(MSG_Enqueueing);
    SERIAL_ECHO(cmd);
    SERIAL_ECHOLNPGM("\"");
    return true;
  }
  return false;
}

void setup_killpin() {
  #if HAS_KILL
    SET_INPUT(KILL_PIN);
    WRITE(KILL_PIN, HIGH);
    #if (digitalPinToInterrupt(KILL_PIN) != NOT_AN_INTERRUPT)
      attachInterrupt(digitalPinToInterrupt(KILL_PIN), kill_pin_handler, KILL_PRESSED_ON);
    #endif
  #endif
}


// Set home pin
void setup_homepin(void) {
  #if HAS_HOME
    SET_INPUT(HOME_PIN);
    WRITE(HOME_PIN, HIGH);
  #endif
}


void setup_photpin() {
  #if HAS_PHOTOGRAPH
    OUT_WRITE(PHOTOGRAPH_PIN, LOW);
  #endif
}

void setup_powerhold() {
  #if HAS_SUICIDE
    OUT_WRITE(SUICIDE_PIN, HIGH);
  #endif
  #if HAS_POWER_SWITCH
    #if ENABLED(PS_DEFAULT_OFF)
      OUT_WRITE(PS_ON_PIN, PS_ON_ASLEEP);
    #else
      OUT_WRITE(PS_ON_PIN, PS_ON_AWAKE);
    #endif
  #endif
}

void suicide() {
  #if HAS_SUICIDE
    OUT_WRITE(SUICIDE_PIN, LOW);
  #endif
}

void servo_init() {
  #if NUM_SERVOS >= 1 && HAS_SERVO_0
    servo[0].attach(SERVO0_PIN);
    servo[0].detach(); // Just set up the pin. We don't have a position yet. Don't move to a random position.
  #endif
  #if NUM_SERVOS >= 2 && HAS_SERVO_1
    servo[1].attach(SERVO1_PIN);
    servo[1].detach();
  #endif
  #if NUM_SERVOS >= 3 && HAS_SERVO_2
    servo[2].attach(SERVO2_PIN);
    servo[2].detach();
  #endif
  #if NUM_SERVOS >= 4 && HAS_SERVO_3
    servo[3].attach(SERVO3_PIN);
    servo[3].detach();
  #endif

  #if HAS_Z_SERVO_ENDSTOP
    /**
     * Set position of Z Servo Endstop
     *
     * The servo might be deployed and positioned too low to stow
     * when starting up the machine or rebooting the board.
     * There's no way to know where the nozzle is positioned until
     * homing has been done - no homing with z-probe without init!
     *
     */
    STOW_Z_SERVO();
  #endif

}

/**
 * Stepper Reset (RigidBoard, et.al.)
 */
#if HAS_STEPPER_RESET
  void disableStepperDrivers() {
    pinMode(STEPPER_RESET_PIN, OUTPUT);
    digitalWrite(STEPPER_RESET_PIN, LOW);  // drive it down to hold in reset motor driver chips
  }
  void enableStepperDrivers() { pinMode(STEPPER_RESET_PIN, INPUT); }  // set to input, which allows it to be pulled high by pullups
#endif

/**
 * Marlin entry-point: Set up before the program loop
 *  - Set up the kill pin, power hold
 *  - Start the serial port
 *  - Print startup messages and diagnostics
 *  - Get EEPROM or default settings
 *  - Initialize managers for:
 *    • temperature
 *    • planner
 *    • watchdog
 *    • stepper
 *    • photo pin
 *    • servos
 *    • LCD controller
 *    • Digipot I2C
 *    • Z probe sled
 *    • status LEDs
 */
void setup() {

  #ifdef DISABLE_JTAG
    // Disable JTAG on AT90USB chips to free up pins for IO
    MCUCR = 0x80;
    MCUCR = 0x80;
  #endif


  setup_killpin();

  setup_powerhold();

  #if HAS_STEPPER_RESET
    disableStepperDrivers();
  #endif

  MYSERIAL.begin(BAUDRATE);
  SERIAL_PROTOCOLLNPGM("start");
  SERIAL_ECHO_START;

  // Check startup - does nothing if bootloader sets MCUSR to 0
  byte mcu = MCUSR;
  if (mcu & 1) SERIAL_ECHOLNPGM(MSG_POWERUP);
  if (mcu & 2) SERIAL_ECHOLNPGM(MSG_EXTERNAL_RESET);
  if (mcu & 4) SERIAL_ECHOLNPGM(MSG_BROWNOUT_RESET);
  if (mcu & 8) SERIAL_ECHOLNPGM(MSG_WATCHDOG_RESET);
  if (mcu & 32) SERIAL_ECHOLNPGM(MSG_SOFTWARE_RESET);
  MCUSR = 0;

  SERIAL_ECHOPGM(MSG_MARLIN);
  SERIAL_ECHOLNPGM(" " SHORT_BUILD_VERSION);

  #ifdef STRING_DISTRIBUTION_DATE
    #ifdef STRING_CONFIG_H_AUTHOR
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM(MSG_CONFIGURATION_VER);
      SERIAL_ECHOPGM(STRING_DISTRIBUTION_DATE);
      SERIAL_ECHOPGM(MSG_AUTHOR);
      SERIAL_ECHOLNPGM(STRING_CONFIG_H_AUTHOR);
      SERIAL_ECHOPGM("Compiled: ");
      SERIAL_ECHOLNPGM(__DATE__);
    #endif // STRING_CONFIG_H_AUTHOR
  #endif // STRING_DISTRIBUTION_DATE

  SERIAL_ECHO_START;
  SERIAL_ECHOPGM(MSG_FREE_MEMORY);
  SERIAL_ECHO(freeMemory());
  SERIAL_ECHOPGM(MSG_PLANNER_BUFFER_BYTES);
  SERIAL_ECHOLN((int)sizeof(block_t)*BLOCK_BUFFER_SIZE);

  // Send "ok" after commands by default
  for (int8_t i = 0; i < BUFSIZE; i++) send_ok[i] = true;

  // Load data from EEPROM if available (or use defaults)
  // This also updates variables in the planner, elsewhere
  Config_RetrieveSettings();

  // Initialize current position based on home_offset
  memcpy(current_position, home_offset, sizeof(home_offset));

  // Vital to init stepper/planner equivalent for current_position
  SYNC_PLAN_POSITION_KINEMATIC();

  plasmaManager.init();

  torchHeightController.init();

  thermalManager.init();    // Initialize temperature loop

  #if ENABLED(USE_WATCHDOG)
    watchdog_init();
  #endif

  stepper.init();    // Initialize stepper, this enables interrupts!
  setup_photpin();
  servo_init();

  #if HAS_STEPPER_RESET
    enableStepperDrivers();
  #endif

  #if ENABLED(DIGIPOT_I2C)
    digipot_i2c_init();
  #endif

  #if ENABLED(DAC_STEPPER_CURRENT)
    dac_init();
  #endif


  setup_homepin();

  #ifdef STAT_LED_RED
    pinMode(STAT_LED_RED, OUTPUT);
    digitalWrite(STAT_LED_RED, LOW); // turn it off
  #endif

  #ifdef STAT_LED_BLUE
    pinMode(STAT_LED_BLUE, OUTPUT);
    digitalWrite(STAT_LED_BLUE, LOW); // turn it off
  #endif

  lcd_init();
  #if ENABLED(SHOW_BOOTSCREEN)
    #if ENABLED(DOGLCD)
      safe_delay(BOOTSCREEN_TIMEOUT);
    #elif ENABLED(ULTRA_LCD)
      bootscreen();
      lcd_init();
    #endif
  #endif

}

/**
 * The main Marlin program loop
 *
 *  - Save or log commands to SD
 *  - Process available commands (if not saving)
 *  - Call heater manager
 *  - Call inactivity manager
 *  - Call endstop manager
 *  - Call LCD update
 */
void loop()
{
    while(stateManager.get_state() != waiting_file)
    {
        char cmd[MAX_CMD_SIZE];
        if(card.get_next_command(cmd))
        {
            process_command(cmd);
            endstops.report_state();
        }
        else
        {
            stepper.synchronize();
            if(stateManager.stop())
                lcd_setstatus("Done.");
        }
    }
    endstops.report_state();
    idle();
}

void gcode_line_error(const char* err, bool doFlush = true) {
  SERIAL_ERROR_START;
  serialprintPGM(err);
  SERIAL_ERRORLN(gcode_LastN);
  //Serial.println(gcode_N);
  if (doFlush) FlushSerialRequestResend();
  serial_count = 0;
}

inline void get_serial_commands() {
  static char serial_line_buffer[MAX_CMD_SIZE];
  static boolean serial_comment_mode = false;

  // If the command buffer is empty for too long,
  // send "wait" to indicate Marlin is still waiting.
  #if defined(NO_TIMEOUTS) && NO_TIMEOUTS > 0
    static millis_t last_command_time = 0;
    millis_t ms = millis();
    if (commands_in_queue == 0 && !MYSERIAL.available() && ELAPSED(ms, last_command_time + NO_TIMEOUTS)) {
      SERIAL_ECHOLNPGM(MSG_WAIT);
      last_command_time = ms;
    }
  #endif

  /**
   * Loop while serial characters are incoming and the queue is not full
   */
  while (commands_in_queue < BUFSIZE && MYSERIAL.available() > 0) {

    char serial_char = MYSERIAL.read();

    /**
     * If the character ends the line
     */
    if (serial_char == '\n' || serial_char == '\r') {

      serial_comment_mode = false; // end of line == end of comment

      if (!serial_count) continue; // skip empty lines

      serial_line_buffer[serial_count] = 0; // terminate string
      serial_count = 0; //reset buffer

      char* command = serial_line_buffer;

      while (*command == ' ') command++; // skip any leading spaces
      char* npos = (*command == 'N') ? command : NULL; // Require the N parameter to start the line
      char* apos = strchr(command, '*');

      if (npos) {

        boolean M110 = strstr_P(command, PSTR("M110")) != NULL;

        if (M110) {
          char* n2pos = strchr(command + 4, 'N');
          if (n2pos) npos = n2pos;
        }

        gcode_N = strtol(npos + 1, NULL, 10);

        if (gcode_N != gcode_LastN + 1 && !M110) {
          gcode_line_error(PSTR(MSG_ERR_LINE_NO));
          return;
        }

        if (apos) {
          byte checksum = 0, count = 0;
          while (command[count] != '*') checksum ^= command[count++];

          if (strtol(apos + 1, NULL, 10) != checksum) {
            gcode_line_error(PSTR(MSG_ERR_CHECKSUM_MISMATCH));
            return;
          }
          // if no errors, continue parsing
        }
        else {
          gcode_line_error(PSTR(MSG_ERR_NO_CHECKSUM));
          return;
        }

        gcode_LastN = gcode_N;
        // if no errors, continue parsing
      }
      else if (apos) { // No '*' without 'N'
        gcode_line_error(PSTR(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM), false);
        return;
      }

      // Movement commands alert when stopped
      if (IsStopped()) {
        char* gpos = strchr(command, 'G');
        if (gpos) {
          int codenum = strtol(gpos + 1, NULL, 10);
          switch (codenum) {
            case 0:
            case 1:
            case 2:
            case 3:
              SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
              LCD_MESSAGEPGM(MSG_STOPPED);
              break;
          }
        }
      }

      #if DISABLED(EMERGENCY_PARSER)
        // If command was e-stop process now
        if (strcmp(command, "M112") == 0) kill(PSTR(MSG_KILLED));
        if (strcmp(command, "M410") == 0) { quickstop_stepper(); }
      #endif

      #if defined(NO_TIMEOUTS) && NO_TIMEOUTS > 0
        last_command_time = ms;
      #endif

      // Add the command to the queue
      _enqueuecommand(serial_line_buffer, true);
    }
    else if (serial_count >= MAX_CMD_SIZE - 1) {
      // Keep fetching, but ignore normal characters beyond the max length
      // The command will be injected when EOL is reached
    }
    else if (serial_char == '\\') {  // Handle escapes
      if (MYSERIAL.available() > 0) {
        // if we have one more character, copy it over
        serial_char = MYSERIAL.read();
        if (!serial_comment_mode) serial_line_buffer[serial_count++] = serial_char;
      }
      // otherwise do nothing
    }
    else { // it's not a newline, carriage return or escape char
      if (serial_char == ';') serial_comment_mode = true;
      if (!serial_comment_mode) serial_line_buffer[serial_count++] = serial_char;
    }

  } // queue has space, serial has data
}

inline bool code_has_value() {
  int i = 1;
  char c = seen_pointer[i];
  while (c == ' ') c = seen_pointer[++i];
  if (c == '-' || c == '+') c = seen_pointer[++i];
  if (c == '.') c = seen_pointer[++i];
  return NUMERIC(c);
}

inline float code_value_float() {
  return strtod(seen_pointer + 1, NULL);
}

inline unsigned long code_value_ulong() { return strtoul(seen_pointer + 1, NULL, 10); }

inline long code_value_long() { return strtol(seen_pointer + 1, NULL, 10); }

inline int code_value_int() { return (int)strtol(seen_pointer + 1, NULL, 10); }

inline uint16_t code_value_ushort() { return (uint16_t)strtoul(seen_pointer + 1, NULL, 10); }

inline uint8_t code_value_byte() { return (uint8_t)(constrain(strtol(seen_pointer + 1, NULL, 10), 0, 255)); }

inline bool code_value_bool() { return code_value_byte() > 0; }

#if ENABLED(INCH_MODE_SUPPORT)
  inline void set_input_linear_units(LinearUnit units) {
    switch (units) {
      case LINEARUNIT_INCH:
        linear_unit_factor = 25.4;
        break;
      case LINEARUNIT_MM:
      default:
        linear_unit_factor = 1.0;
        break;
    }
  }

  inline float axis_unit_factor(int axis) {
    return linear_unit_factor;
  }

  inline float code_value_linear_units() { return code_value_float() * linear_unit_factor; }
  inline float code_value_axis_units(int axis) { return code_value_float() * axis_unit_factor(axis); }
  inline float code_value_per_axis_unit(int axis) { return code_value_float() / axis_unit_factor(axis); }

#else

  inline float code_value_linear_units() { return code_value_float(); }
  inline float code_value_axis_units(int axis) { UNUSED(axis); return code_value_float(); }
  inline float code_value_per_axis_unit(int axis) { UNUSED(axis); return code_value_float(); }

#endif

FORCE_INLINE millis_t code_value_millis() { return code_value_ulong(); }
inline millis_t code_value_millis_from_seconds() { return code_value_float() * 1000; }

bool code_seen(char code) {
  seen_pointer = strchr(cmd_args, code);
  return (seen_pointer != NULL); // Return TRUE if the code-letter was found
}

#define DEFINE_PGM_READ_ANY(type, reader)       \
  static inline type pgm_read_any(const type *p)  \
  { return pgm_read_##reader##_near(p); }

DEFINE_PGM_READ_ANY(float,       float);
DEFINE_PGM_READ_ANY(signed char, byte);

#define XYZ_CONSTS_FROM_CONFIG(type, array, CONFIG) \
  static const PROGMEM type array##_P[3] =        \
      { X_##CONFIG, Y_##CONFIG, Z_##CONFIG };     \
  static inline type array(int axis)          \
  { return pgm_read_any(&array##_P[axis]); }

XYZ_CONSTS_FROM_CONFIG(float, base_min_pos,   MIN_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_max_pos,   MAX_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_home_pos,  HOME_POS);
XYZ_CONSTS_FROM_CONFIG(float, max_length,     MAX_LENGTH);
XYZ_CONSTS_FROM_CONFIG(float, home_bump_mm,   HOME_BUMP_MM);
XYZ_CONSTS_FROM_CONFIG(signed char, home_dir, HOME_DIR);


/**
 * Software endstops can be used to monitor the open end of
 * an axis that has a hardware endstop on the other end. Or
 * they can prevent axes from moving past endstops and grinding.
 *
 * To keep doing their job as the coordinate system changes,
 * the software endstop positions must be refreshed to remain
 * at the same positions relative to the machine.
 */
static void update_software_endstops(AxisEnum axis) {
  float offs = LOGICAL_POSITION(0, axis);
  sw_endstop_min[axis] = base_min_pos(axis) + offs;
  sw_endstop_max[axis] = base_max_pos(axis) + offs;
}

static void set_axis_is_at_home(AxisEnum axis) {
  position_shift[axis] = 0;
  current_position[axis] = LOGICAL_POSITION(base_home_pos(axis), axis);
  update_software_endstops(axis);
}

/**
 * Some planner shorthand inline functions
 */
inline float get_homing_bump_feedrate(AxisEnum axis) {
  const int homing_bump_divisor[] = HOMING_BUMP_DIVISOR;
  int hbd = homing_bump_divisor[axis];
  if (hbd < 1) {
    hbd = 10;
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Warning: Homing Bump Divisor < 1");
  }
  return homing_feedrate_mm_m[axis] / hbd;
}
//
// line_to_current_position
// Move the planner to the current position from wherever it last moved
// (or from wherever it has been told it is located).
//
inline void line_to_current_position() {
  planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], MMM_TO_MMS(feedrate_mm_m));
}

inline void line_to_z(float zPosition) {
  planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, MMM_TO_MMS(feedrate_mm_m));
}

inline void line_to_axis_pos(AxisEnum axis, float where, float fr_mm_m = 0.0) {
  float old_feedrate_mm_m = feedrate_mm_m;
  current_position[axis] = where;
  feedrate_mm_m = (fr_mm_m != 0.0) ? fr_mm_m : homing_feedrate_mm_m[axis];
  planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], MMM_TO_MMS(feedrate_mm_m));
  stepper.synchronize();
  feedrate_mm_m = old_feedrate_mm_m;
}

//
// line_to_destination
// Move the planner, not necessarily synced with current_position
//
inline void line_to_destination(float fr_mm_m) {
  planner.buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], MMM_TO_MMS(fr_mm_m));
}
inline void line_to_destination() { line_to_destination(feedrate_mm_m); }
inline void set_current_to_destination() { memcpy(current_position, destination, sizeof(current_position)); }
inline void set_destination_to_current() { memcpy(destination, current_position, sizeof(destination)); }

/**
 *  Plan a move to (X, Y, Z) and set the current_position
 *  The final current_position may not be the one that was requested
 */
void do_blocking_move_to(float x, float y, float z, float fr_mm_m /*=0.0*/) {
  float old_feedrate_mm_m = feedrate_mm_m;
  // If Z needs to raise, do it before moving XY
  if (current_position[Z_AXIS] < z) {
    feedrate_mm_m = (fr_mm_m != 0.0) ? fr_mm_m : homing_feedrate_mm_m[Z_AXIS];
    current_position[Z_AXIS] = z;
    line_to_current_position();
  }

  feedrate_mm_m = (fr_mm_m != 0.0) ? fr_mm_m : XY_PROBE_FEEDRATE_MM_M;
  current_position[X_AXIS] = x;
  current_position[Y_AXIS] = y;
  line_to_current_position();

  // If Z needs to lower, do it after moving XY
  if (current_position[Z_AXIS] > z) {
    feedrate_mm_m = (fr_mm_m != 0.0) ? fr_mm_m : homing_feedrate_mm_m[Z_AXIS];
    current_position[Z_AXIS] = z;
    line_to_current_position();
  }

  stepper.synchronize();
  feedrate_mm_m = old_feedrate_mm_m;
}
void do_blocking_move_to_x(float x, float fr_mm_m/*=0.0*/) {
  do_blocking_move_to(x, current_position[Y_AXIS], current_position[Z_AXIS], fr_mm_m);
}
void do_blocking_move_to_z(float z, float fr_mm_m/*=0.0*/) {
  do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], z, fr_mm_m);
}
void do_blocking_move_to_xy(float x, float y, float fr_mm_m/*=0.0*/) {
  do_blocking_move_to(x, y, current_position[Z_AXIS], fr_mm_m);
}

//
// Prepare to do endstop or probe moves
// with custom feedrates.
//
//  - Save current feedrates
//  - Reset the rate multiplier
//  - Reset the command timeout
//  - Enable the endstops (for endstop moves)
//
static void setup_for_endstop_or_probe_move() {
  saved_feedrate_mm_m = feedrate_mm_m;
  saved_feedrate_percentage = feedrate_percentage;
  feedrate_percentage = 100;
  refresh_cmd_timeout();
}

static void clean_up_after_endstop_or_probe_move() {
  feedrate_mm_m = saved_feedrate_mm_m;
  feedrate_percentage = saved_feedrate_percentage;
  refresh_cmd_timeout();
}

#if 0 || 0 || 0 || HAS_PROBING_PROCEDURE || HOTENDS > 1 || 0 || 0
  static bool axis_unhomed_error(const bool x, const bool y, const bool z) {
    const bool xx = x && !axis_homed[X_AXIS],
               yy = y && !axis_homed[Y_AXIS],
               zz = z && !axis_homed[Z_AXIS];
    if (xx || yy || zz) {
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM(MSG_HOME " ");
      if (xx) SERIAL_ECHOPGM(MSG_X);
      if (yy) SERIAL_ECHOPGM(MSG_Y);
      if (zz) SERIAL_ECHOPGM(MSG_Z);
      SERIAL_ECHOLNPGM(" " MSG_FIRST);

      #if ENABLED(ULTRA_LCD)
        char message[3 * (LCD_WIDTH) + 1] = ""; // worst case is kana.utf with up to 3*LCD_WIDTH+1
        strcat_P(message, PSTR(MSG_HOME " "));
        if (xx) strcat_P(message, PSTR(MSG_X));
        if (yy) strcat_P(message, PSTR(MSG_Y));
        if (zz) strcat_P(message, PSTR(MSG_Z));
        strcat_P(message, PSTR(" " MSG_FIRST));
        lcd_setstatus(message);
      #endif
      return true;
    }
    return false;
  }
#endif

#define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)

static void homeaxis(AxisEnum axis) {
  #define HOMEAXIS_DO(LETTER) \
    ((LETTER##_MIN_PIN > -1 && LETTER##_HOME_DIR==-1) || (LETTER##_MAX_PIN > -1 && LETTER##_HOME_DIR==1))

  if (!(axis == X_AXIS ? HOMEAXIS_DO(X) : axis == Y_AXIS ? HOMEAXIS_DO(Y) : axis == Z_AXIS ? HOMEAXIS_DO(Z) : 0))
    return;

  int axis_home_dir = home_dir(axis);

  // Set the axis position as setup for the move
  current_position[axis] = 0;
  sync_plan_position();

  // Set a flag for Z motor locking
  #if ENABLED(Z_DUAL_ENDSTOPS)
    if (axis == Z_AXIS) stepper.set_homing_flag(true);
  #endif

  // Move towards the endstop until an endstop is triggered
  line_to_axis_pos(axis, 1.5 * max_length(axis) * axis_home_dir);

  // Set the axis position as setup for the move
  current_position[axis] = 0;
  sync_plan_position();

  // Move away from the endstop by the axis HOME_BUMP_MM
  line_to_axis_pos(axis, -home_bump_mm(axis) * axis_home_dir);

  // Move slowly towards the endstop until triggered
  line_to_axis_pos(axis, 2 * home_bump_mm(axis) * axis_home_dir, get_homing_bump_feedrate(axis));

  // reset current_position to 0 to reflect hitting endpoint
  current_position[axis] = 0;
  sync_plan_position();

  #if ENABLED(Z_DUAL_ENDSTOPS)
    if (axis == Z_AXIS) {
      float adj = fabs(z_endstop_adj);
      bool lockZ1;
      if (axis_home_dir > 0) {
        adj = -adj;
        lockZ1 = (z_endstop_adj > 0);
      }
      else
        lockZ1 = (z_endstop_adj < 0);

      if (lockZ1)
        stepper.set_z_lock(true);
      else
        stepper.set_z2_lock(true);

      // Move to the adjusted endstop height
      line_to_axis_pos(axis, adj);

      if (lockZ1)
        stepper.set_z_lock(false);
      else
        stepper.set_z2_lock(false);

      stepper.set_homing_flag(false);
    } // Z_AXIS
  #endif

  // Set the axis position to its home position (plus home offsets)
  set_axis_is_at_home(axis);

  SYNC_PLAN_POSITION_KINEMATIC();

  destination[axis] = current_position[axis];
  endstops.hit_on_purpose(); // clear endstop hit flags
  axis_known_position[axis] = true;
  axis_homed[axis] = true;
}

/**
 * ***************************************************************************
 * ***************************** G-CODE HANDLING *****************************
 * ***************************************************************************
 */

/**
 * Set XYZE destination and feedrate from the current GCode command
 *
 *  - Set destination from included axis codes
 *  - Set to current for missing axis codes
 *  - Set the feedrate, if included
 */
void gcode_get_destination() {
  LOOP_XYZ(i) {
    if (code_seen(axis_codes[i]))
      destination[i] = code_value_axis_units(i) + (axis_relative_modes[i] || relative_mode ? current_position[i] : 0);
    else
      destination[i] = current_position[i];
  }

  if (code_seen('F') && code_value_linear_units() > 0.0 && plasmaManager.get_state() == Off)
    feedrate_mm_m = code_value_linear_units();
}

void unknown_command_error() {
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
  SERIAL_ECHO(cmd);
  SERIAL_ECHOLNPGM("\"");
}

#if ENABLED(HOST_KEEPALIVE_FEATURE)
  /**
   * Output a "busy" message at regular intervals
   * while the machine is not accepting commands.
   */
  void host_keepalive() {
    millis_t ms = millis();
    if (host_keepalive_interval && busy_state != NOT_BUSY) {
      if (PENDING(ms, next_busy_signal_ms)) return;
      switch (busy_state) {
        case IN_HANDLER:
        case IN_PROCESS:
          SERIAL_ECHO_START;
          SERIAL_ECHOLNPGM(MSG_BUSY_PROCESSING);
          break;
        case PAUSED_FOR_USER:
          SERIAL_ECHO_START;
          SERIAL_ECHOLNPGM(MSG_BUSY_PAUSED_FOR_USER);
          break;
        case PAUSED_FOR_INPUT:
          SERIAL_ECHO_START;
          SERIAL_ECHOLNPGM(MSG_BUSY_PAUSED_FOR_INPUT);
          break;
        default:
          break;
      }
    }
    next_busy_signal_ms = ms + host_keepalive_interval * 1000UL;
  }
#endif //HOST_KEEPALIVE_FEATURE

/**
 * G0, G1: Coordinated movement of X Y Z E axes
 */
inline void gcode_G0_G1() {
  if (IsRunning()) {
    while(plasmaManager.get_state() == Slowdown_THC) { idle(); }
    ABORT_IF_UNHOMED;

    gcode_get_destination(); // For X Y Z E F

    prepare_move_to_destination();
  }
}

/**
 * G2: Clockwise Arc
 * G3: Counterclockwise Arc
 */
#if ENABLED(ARC_SUPPORT)
  inline void gcode_G2_G3(bool clockwise) {
    if (IsRunning()) {

      #if ENABLED(SF_ARC_FIX)
        bool relative_mode_backup = relative_mode;
        relative_mode = true;
      #endif

      gcode_get_destination();

      #if ENABLED(SF_ARC_FIX)
        relative_mode = relative_mode_backup;
      #endif

      // Center of arc as offset from current_position
      float arc_offset[2] = {
        code_seen('I') ? code_value_axis_units(X_AXIS) : 0,
        code_seen('J') ? code_value_axis_units(Y_AXIS) : 0
      };

      // Send an arc to the planner
      plan_arc(destination, arc_offset, clockwise);

      refresh_cmd_timeout();
    }
  }
#endif


inline void dwell(millis_t dwell_ms) {
  refresh_cmd_timeout();
  dwell_ms += previous_cmd_ms;  // keep track of when we started waiting

  while (PENDING(millis(), dwell_ms))
  {
    // idle fast (no screen refresh) when pause approaches the end.
    idle(!PENDING(millis(), dwell_ms - 80));
  }
}

/**
 * G4: Dwell S<seconds> or P<milliseconds>
 */
inline void gcode_G4() {
  millis_t dwell_ms = 0;

  if (code_seen('P'))
    dwell_ms = code_value_millis(); // milliseconds to wait
  if (code_seen('S'))
    dwell_ms = code_value_millis_from_seconds(); // seconds to wait

  stepper.synchronize();

  if (!lcd_hasstatus())
    LCD_MESSAGEPGM(MSG_DWELL);

  dwell(dwell_ms);
}

#if ENABLED(BEZIER_CURVE_SUPPORT)
  /**
   * Parameters interpreted according to:
   * http://linuxcnc.org/docs/2.6/html/gcode/gcode.html#sec:G5-Cubic-Spline
   * However I, J omission is not supported at this point; all
   * parameters can be omitted and default to zero.
   */

  /**
   * G5: Cubic B-spline
   */
  inline void gcode_G5() {
    if (IsRunning()) {

      gcode_get_destination();

      float offset[] = {
        code_seen('I') ? code_value_axis_units(X_AXIS) : 0.0,
        code_seen('J') ? code_value_axis_units(Y_AXIS) : 0.0,
        code_seen('P') ? code_value_axis_units(X_AXIS) : 0.0,
        code_seen('Q') ? code_value_axis_units(Y_AXIS) : 0.0
      };

      plan_cubic_move(offset);
    }
  }
#endif // BEZIER_CURVE_SUPPORT

#if ENABLED(INCH_MODE_SUPPORT)
  /**
   * G20: Set input mode to inches
   */
  inline void gcode_G20() {
    set_input_linear_units(LINEARUNIT_INCH);
  }

  /**
   * G21: Set input mode to millimeters
   */
  inline void gcode_G21() {
    set_input_linear_units(LINEARUNIT_MM);
  }
#endif // INCH_MODE_SUPPORT

#if ENABLED(QUICK_HOME)
  static void quick_home_xy() {
    // Pretend the current position is 0,0
    current_position[X_AXIS] = current_position[Y_AXIS] = 0.0;
    sync_plan_position();

    int x_axis_home_dir = home_dir(X_AXIS);

    float mlx = max_length(X_AXIS);
    float mly = max_length(Y_AXIS);
    float mlratio = mlx > mly ? mly / mlx : mlx / mly;
    float fr_mm_m = min(homing_feedrate_mm_m[X_AXIS], homing_feedrate_mm_m[Y_AXIS]) * sqrt(sq(mlratio) + 1.0);

    do_blocking_move_to_xy(1.5 * mlx * x_axis_home_dir, 1.5 * mly * home_dir(Y_AXIS), fr_mm_m);
    endstops.hit_on_purpose(); // clear endstop hit flags
    current_position[X_AXIS] = current_position[Y_AXIS] = 0.0;
  }
#endif // QUICK_HOME

/**
 * G28: Home all axes according to settings
 *
 * Parameters
 *
 *  None  Home to all axes with no parameters.
 *        With QUICK_HOME enabled XY will home together, then Z.
 *
 * Cartesian parameters
 *
 *  X   Home to the X endstop
 *  Y   Home to the Y endstop
 *  Z   Home to the Z endstop
 *
 */
inline void gcode_G28() {
  while(plasmaManager.get_state() == Slowdown_THC) { idle(); }

  if(plasmaManager.get_state() != Off)
  {
    lcd_setstatus("Stop: G28 while plasma activated.");
    stateManager.stop();
  }
  else
  {
    autohome(code_seen('X'), code_seen('Y'), code_seen('Z'));
  }
}

void autohome(bool homeX, bool homeY, bool homeZ)
{
  // Wait for planner moves to finish!
  stepper.synchronize();

  setup_for_endstop_or_probe_move();
  endstops.enable(true); // Enable endstops for next homing move

  home_all_axis = (!homeX && !homeY && !homeZ) || (homeX && homeY && homeZ);

  set_destination_to_current();

  #if Z_HOME_DIR > 0  // If homing away from BED do Z first
    if (home_all_axis || homeZ) { HOMEAXIS(Z); }
  #else
    if (home_all_axis || homeX || homeY) {
      // Raise Z before homing any other axes and z is not already high enough (never lower z)
      destination[Z_AXIS] = LOGICAL_Z_POSITION(Z_HOMING_HEIGHT);
      if (destination[Z_AXIS] > current_position[Z_AXIS]) {
        do_blocking_move_to_z(destination[Z_AXIS]);
      }
    }
  #endif

  #if ENABLED(QUICK_HOME)
    if (home_all_axis || (homeX && homeY)) { quick_home_xy(); }
  #endif

  #if ENABLED(HOME_Y_BEFORE_X)
    // Home Y
    if (home_all_axis || homeY) { HOMEAXIS(Y); }
  #endif

  // Home X
  if (home_all_axis || homeX) { HOMEAXIS(X); }

  #if DISABLED(HOME_Y_BEFORE_X)
    // Home Y
    if (home_all_axis || homeY) { HOMEAXIS(Y); }
  #endif

  // Home Z last if homing towards the bed
  #if Z_HOME_DIR < 0
    // Home Z
    if (home_all_axis || homeZ) { HOMEAXIS(Z); }
  #endif // Z_HOME_DIR < 0

  SYNC_PLAN_POSITION_KINEMATIC();

  endstops.not_homing();
  endstops.hit_on_purpose(); // clear endstop hit flags

  clean_up_after_endstop_or_probe_move();
  report_current_position();
}

#if HAS_PROBING_PROCEDURE
  void out_of_range_error(const char* p_edge) {
    SERIAL_PROTOCOLPGM("?Probe ");
    serialprintPGM(p_edge);
    SERIAL_PROTOCOLLNPGM(" position out of range.");
  }
#endif

/**
 * G92: Set current position to given X Y Z
 */
inline void gcode_G92() {
  stepper.synchronize();

  bool didXYZ = false;
  LOOP_XYZ(i) {
    if (code_seen(axis_codes[i])) {
      float p = current_position[i],
            v = code_value_axis_units(i);

      current_position[i] = v;

      position_shift[i] += v - p; // Offset the coordinate space
      update_software_endstops((AxisEnum)i);
      didXYZ = true;
    }
  }
  if (didXYZ)
    SYNC_PLAN_POSITION_KINEMATIC();
}

#if ENABLED(ULTIPANEL)
  /**
   * M0: Unconditional stop - Wait for user button press on LCD
   */
  inline void gcode_M0_M1() {
    char* args = cmd_args;

    millis_t codenum = 0;
    bool hasP = false, hasS = false;
    if (code_seen('P')) {
      codenum = code_value_millis(); // milliseconds to wait
      hasP = codenum > 0;
    }
    if (code_seen('S')) {
      codenum = code_value_millis_from_seconds(); // seconds to wait
      hasS = codenum > 0;
    }

    if (!hasP && !hasS && *args != '\0')
      lcd_setstatus(args, true);
    else {
      LCD_MESSAGEPGM(MSG_USERWAIT);
      #if ENABLED(LCD_PROGRESS_BAR) && PROGRESS_MSG_EXPIRE > 0
        dontExpireStatus();
      #endif
    }

    lcd_ignore_click();
    stepper.synchronize();
    refresh_cmd_timeout();
    if (codenum > 0) {
      codenum += previous_cmd_ms;  // wait until this time for a click
      KEEPALIVE_STATE(PAUSED_FOR_USER);
      while (PENDING(millis(), codenum) && !lcd_clicked()) idle();
      KEEPALIVE_STATE(IN_HANDLER);
      lcd_ignore_click(false);
    }
    else {
      if (!lcd_detected()) return;
      KEEPALIVE_STATE(PAUSED_FOR_USER);
      while (!lcd_clicked()) idle();
      KEEPALIVE_STATE(IN_HANDLER);
    }
    if (IS_RUNNING)
      LCD_MESSAGEPGM(MSG_RESUMING);
    else
      LCD_MESSAGEPGM(WELCOME_MSG);
  }
#endif // ULTIPANEL

void ohmic_probing() {
  endstops.enable(true);
  line_to_axis_pos(Z_AXIS, 0, HOMING_FEEDRATE_Z);

  stepper.synchronize();
  if(TEST(endstops.endstop_hit_bits, Z_MIN))
    current_position[Z_AXIS] = stepper.triggered_position_mm(Z_AXIS);
  else
    current_position[Z_AXIS] = 0;
  sync_plan_position();

  endstops.not_homing();
  endstops.hit_on_purpose();

  // retract
  destination[Z_AXIS] = current_position[Z_AXIS] + 3.8;
  prepare_move_to_destination();
}

/**
 * M3: Start plasma torch and wait for arc transfer
 */
inline void gcode_M3() {
  stepper.synchronize();
  ABORT_IF_UNHOMED;

  while(plasmaManager.get_state() == Slowdown_THC) { idle(); }
  if(plasmaManager.get_state() != Off)
    return;

  ohmic_probing();
  stepper.synchronize();

  while(true) {
      refresh_cmd_timeout();
      millis_t transfer_timeout = PLASMA_TRANSFER_TIMEOUT_MS;
      transfer_timeout += previous_cmd_ms;

      if(!plasmaManager.ignite())
        return;

      KEEPALIVE_STATE(PAUSED_FOR_INPUT);
      while (PENDING(millis(), transfer_timeout)) {
        PlasmaState plasma_state = plasmaManager.get_state();
        if(IS_WAITING_FILE)
        {
          return;
        }
        if(plasma_state == Established)
        {
          dwell((millis_t)plasmaManager._pierce_time_ms);
          saved_feedrate_mm_m = feedrate_mm_m;
          feedrate_mm_m = plasmaManager._cutting_feedrate_mm_m;
          plasmaManager.activate_thc();
          lcd_setstatus("Running...");
          return;
        }
        if(IS_SUSPENDED)
          break;
        idle(true);
      }
      plasmaManager.stop();

      if(stateManager.suspend())
        lcd_setstatus("Paused, plasma error.");

      while (IS_SUSPENDED) idle();
      if(IS_WAITING_FILE)
        return;

      lcd_setstatus("Retry ignition...");
  }
}

/**
 * M5: Stop plasma torch
 */
inline void gcode_M5() {
  plasmaManager.stop_after_move();
  stepper.synchronize();

  if(plasmaManager.is_lost())
  {
    lcd_setstatus("Paused, plasma stopped.");
    stateManager.suspend();
  }

  if(stateManager.suspend_if_pending())
  {
    lcd_setstatus("Paused.");
  }

  while (IS_SUSPENDED) idle();
  plasmaManager.stop();
  lcd_setstatus("Running...");

  feedrate_mm_m = saved_feedrate_mm_m;
}

/**
 * M75: Start print timer
 */
inline void gcode_M75() { print_job_timer.start(); }

/**
 * M76: Pause print timer
 */
inline void gcode_M76() { print_job_timer.pause(); }

/**
 * M77: Stop print timer
 */
inline void gcode_M77() { print_job_timer.stop(); }

#if ENABLED(PRINTCOUNTER)
  /**
   * M78: Show print statistics
   */
  inline void gcode_M78() {
    // "M78 S78" will reset the statistics
    if (code_seen('S') && code_value_int() == 78)
      print_job_timer.initStats();
    else print_job_timer.showStats();
  }
#endif

/**
 * M110: Set Current Line Number
 */
inline void gcode_M110() {
  if (code_seen('N')) gcode_N = code_value_long();
}

/**
 * Output the current position to serial
 */
static void report_current_position() {
  SERIAL_PROTOCOLPGM("X:");
  SERIAL_PROTOCOL(current_position[X_AXIS]);
  SERIAL_PROTOCOLPGM(" Y:");
  SERIAL_PROTOCOL(current_position[Y_AXIS]);
  SERIAL_PROTOCOLPGM(" Z:");
  SERIAL_PROTOCOL(current_position[Z_AXIS]);
  stepper.report_positions();
}

/**
 * M117: Set LCD Status Message
 */
inline void gcode_M117() {
  lcd_setstatus(cmd_args);
}

#if HAS_BUZZER
  /**
   * M300: Play beep sound S<frequency Hz> P<duration ms>
   */
  inline void gcode_M300() {
    uint16_t const frequency = code_seen('S') ? code_value_ushort() : 260;
    uint16_t duration = code_seen('P') ? code_value_ushort() : 1000;

    // Limits the tone duration to 0-5 seconds.
    NOMORE(duration, 5000);

    BUZZ(duration, frequency);
  }
#endif // HAS_BUZZER

/**
 * M400: Finish all moves
 */
inline void gcode_M400() {
  stepper.synchronize();
}

void quickstop_stepper() {
  stepper.quick_stop();
    stepper.synchronize();
    LOOP_XYZ(i) set_current_from_steppers_for_axis((AxisEnum)i);
    SYNC_PLAN_POSITION_KINEMATIC();
}

/**
 * Process a single command and dispatch it to its handler
 * This is called from the main loop()
 */
void process_command(char* cmd) {
  if (DEBUGGING(ECHO)) {
    SERIAL_ECHO_START;
    SERIAL_ECHOLN(cmd);
  }

  // Sanitize the command:
  //  - Skip leading spaces
  //  - Bypass N[-0-9][0-9]*[ ]*
  //  - Overwrite * with nul to mark the end
  while (*cmd == ' ') ++cmd;
  if (*cmd == 'N' && NUMERIC_SIGNED(cmd[1])) {
    cmd += 2; // skip N[-0-9]
    while (NUMERIC(*cmd)) ++cmd; // skip [0-9]*
    while (*cmd == ' ') ++cmd; // skip [ ]*
  }
  char* starpos = strchr(cmd, '*');  // * should always be the last parameter
  if (starpos) while (*starpos == ' ' || *starpos == '*') *starpos-- = '\0'; // nullify '*' and ' '

  char *cmd_ptr = cmd;

  // Get the command code, which must be G, M, or T
  char command_code = *cmd_ptr++;

  // Skip spaces to get the numeric part
  while (*cmd_ptr == ' ') cmd_ptr++;

  uint16_t codenum = 0; // define ahead of goto

  // Bail early if there's no code
  bool code_is_good = NUMERIC(*cmd_ptr);
  if (!code_is_good) goto ExitUnknownCommand;

  // Get and skip the code number
  do {
    codenum = (codenum * 10) + (*cmd_ptr - '0');
    cmd_ptr++;
  } while (NUMERIC(*cmd_ptr));

  // Skip all spaces to get to the first argument, or nul
  while (*cmd_ptr == ' ') cmd_ptr++;

  // The command's arguments (if any) start here, for sure!
  cmd_args = cmd_ptr;

  KEEPALIVE_STATE(IN_HANDLER);

  // Handle a known G, M, or T
  switch (command_code) {
    case 'G': switch (codenum) {

      // G0, G1
      case 0:
      case 1:
        gcode_G0_G1();
        break;

      // G4 Dwell
      case 4:
        gcode_G4();
        break;

      case 28: // G28: Home all axes, one at a time
        gcode_G28();
        break;

      case 90: // G90
        relative_mode = false;
        break;

      case 91: // G91
        if(code_seen('Z'))
          axis_relative_modes[Z_AXIS] = true;
        else
          relative_mode = true;
        break;

      case 92: // G92
        gcode_G92();
        break;
    }
    break;

    case 'M': switch (codenum) {

      case 3: //Start plasma torch and wait for arc transfer
        gcode_M3();
        break;

      case 5: //Stop plasma torch
        gcode_M5();
        break;

      case 75: // Start print timer
        gcode_M75();
        break;

      case 76: // Pause print timer
        gcode_M76();
        break;

      case 77: // Stop print timer
        gcode_M77();
        break;

      #if ENABLED(PRINTCOUNTER)
        case 78: // Show print statistics
          gcode_M78();
          break;
      #endif

      case 110: // M110: Set Current Line Number
        gcode_M110();
        break;

      case 117: // M117: Set LCD message text, if possible
        gcode_M117();
        break;

      #if HAS_BUZZER
        case 300: // M300 - Play beep tone
          gcode_M300();
          break;
      #endif // HAS_BUZZER

      case 400: // M400 finish all moves
        gcode_M400();
        break;
    }
    break;

    default: code_is_good = false;
  }

  KEEPALIVE_STATE(NOT_BUSY);

ExitUnknownCommand:

  // Still unknown command? Throw an error
  if (!code_is_good) unknown_command_error();

  ok_to_send();
}

void FlushSerialRequestResend() {
  //char command_queue[cmd_queue_index_r][100]="Resend:";
  MYSERIAL.flush();
  SERIAL_PROTOCOLPGM(MSG_RESEND);
  SERIAL_PROTOCOLLN(gcode_LastN + 1);
  ok_to_send();
}

void ok_to_send() {
  refresh_cmd_timeout();
  if (!send_ok[cmd_queue_index_r]) return;
  SERIAL_PROTOCOLPGM(MSG_OK);
  #if ENABLED(ADVANCED_OK)
    char* p = command_queue[cmd_queue_index_r];
    if (*p == 'N') {
      SERIAL_PROTOCOL(' ');
      SERIAL_ECHO(*p++);
      while (NUMERIC_SIGNED(*p))
        SERIAL_ECHO(*p++);
    }
    SERIAL_PROTOCOLPGM(" P"); SERIAL_PROTOCOL(int(BLOCK_BUFFER_SIZE - planner.movesplanned() - 1));
    SERIAL_PROTOCOLPGM(" B"); SERIAL_PROTOCOL(BUFSIZE - commands_in_queue);
  #endif
  SERIAL_EOL;
}

void clamp_to_software_endstops(float target[3]) {
  if (min_software_endstops) {
    NOLESS(target[X_AXIS], sw_endstop_min[X_AXIS]);
    NOLESS(target[Y_AXIS], sw_endstop_min[Y_AXIS]);
    NOLESS(target[Z_AXIS], sw_endstop_min[Z_AXIS]);
  }
  if (max_software_endstops) {
    NOMORE(target[X_AXIS], sw_endstop_max[X_AXIS]);
    NOMORE(target[Y_AXIS], sw_endstop_max[Y_AXIS]);
    NOMORE(target[Z_AXIS], sw_endstop_max[Z_AXIS]);
  }
}

#define IS_BELOW_SOFT_END(v,a) (v[a] < sw_endstop_min[a])
#define IS_ABOVE_SOFT_END(v,a) (v[a] > sw_endstop_max[a])

bool breach_software_endstops(float target[3])
{
  bool breach = false;

  LOOP_XYZ(axis) breach |= IS_BELOW_SOFT_END(target, axis);
  LOOP_XYZ(axis) breach |= IS_ABOVE_SOFT_END(target, axis);

  if(breach)
  {
    lcd_setstatus("Stop: prevent overrun.");
    stateManager.stop();
  }

  return breach;
}

void set_current_from_steppers_for_axis(AxisEnum axis) {
    current_position[axis] = stepper.get_axis_position_mm(axis); // CORE handled transparently
}

inline bool prepare_move_to_destination_cartesian() {
  // Do not use feedrate_percentage for Z only moves
  if (current_position[X_AXIS] == destination[X_AXIS] && current_position[Y_AXIS] == destination[Y_AXIS]) {
    line_to_destination();
  }
  else {
    line_to_destination(MMM_SCALED(feedrate_mm_m));
  }
  return true;
}

/**
 * Prepare a single move and get ready for the next one
 *
 * (This may call planner.buffer_line several times to put
 *  smaller moves into the planner for DELTA or SCARA.)
 */
void prepare_move_to_destination() {
  if(breach_software_endstops(destination))
    return;

  refresh_cmd_timeout();

  if (!prepare_move_to_destination_cartesian())
    return;

  set_current_to_destination();
}

#if ENABLED(ARC_SUPPORT)
  /**
   * Plan an arc in 2 dimensions
   *
   * The arc is approximated by generating many small linear segments.
   * The length of each segment is configured in MM_PER_ARC_SEGMENT (Default 1mm)
   * Arcs should only be made relatively large (over 5mm), as larger arcs with
   * larger segments will tend to be more efficient. Your slicer should have
   * options for G2/G3 arc generation. In future these options may be GCode tunable.
   */
  void plan_arc(
    float target[NUM_AXIS], // Destination position
    float* offset,          // Center of rotation relative to current_position
    uint8_t clockwise       // Clockwise?
  ) {

    float radius = HYPOT(offset[X_AXIS], offset[Y_AXIS]),
          center_X = current_position[X_AXIS] + offset[X_AXIS],
          center_Y = current_position[Y_AXIS] + offset[Y_AXIS],
          linear_travel = target[Z_AXIS] - current_position[Z_AXIS],
          r_X = -offset[X_AXIS],  // Radius vector from center to current location
          r_Y = -offset[Y_AXIS],
          rt_X = target[X_AXIS] - center_X,
          rt_Y = target[Y_AXIS] - center_Y;

    // CCW angle of rotation between position and target from the circle center. Only one atan2() trig computation required.
    float angular_travel = atan2(r_X * rt_Y - r_Y * rt_X, r_X * rt_X + r_Y * rt_Y);
    if (angular_travel < 0) angular_travel += RADIANS(360);
    if (clockwise) angular_travel -= RADIANS(360);

    // Make a circle if the angular rotation is 0
    if (angular_travel == 0 && current_position[X_AXIS] == target[X_AXIS] && current_position[Y_AXIS] == target[Y_AXIS])
      angular_travel += RADIANS(360);

    float mm_of_travel = HYPOT(angular_travel * radius, fabs(linear_travel));
    if (mm_of_travel < 0.001) return;
    uint16_t segments = floor(mm_of_travel / (MM_PER_ARC_SEGMENT));
    if (segments == 0) segments = 1;

    float theta_per_segment = angular_travel / segments;
    float linear_per_segment = linear_travel / segments;

    /**
     * Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
     * and phi is the angle of rotation. Based on the solution approach by Jens Geisler.
     *     r_T = [cos(phi) -sin(phi);
     *            sin(phi)  cos(phi] * r ;
     *
     * For arc generation, the center of the circle is the axis of rotation and the radius vector is
     * defined from the circle center to the initial position. Each line segment is formed by successive
     * vector rotations. This requires only two cos() and sin() computations to form the rotation
     * matrix for the duration of the entire arc. Error may accumulate from numerical round-off, since
     * all double numbers are single precision on the Arduino. (True double precision will not have
     * round off issues for CNC applications.) Single precision error can accumulate to be greater than
     * tool precision in some cases. Therefore, arc path correction is implemented.
     *
     * Small angle approximation may be used to reduce computation overhead further. This approximation
     * holds for everything, but very small circles and large MM_PER_ARC_SEGMENT values. In other words,
     * theta_per_segment would need to be greater than 0.1 rad and N_ARC_CORRECTION would need to be large
     * to cause an appreciable drift error. N_ARC_CORRECTION~=25 is more than small enough to correct for
     * numerical drift error. N_ARC_CORRECTION may be on the order a hundred(s) before error becomes an
     * issue for CNC machines with the single precision Arduino calculations.
     *
     * This approximation also allows plan_arc to immediately insert a line segment into the planner
     * without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
     * a correction, the planner should have caught up to the lag caused by the initial plan_arc overhead.
     * This is important when there are successive arc motions.
     */
    // Vector rotation matrix values
    float cos_T = 1 - 0.5 * sq(theta_per_segment); // Small angle approximation
    float sin_T = theta_per_segment;

    float arc_target[NUM_AXIS];
    float sin_Ti, cos_Ti, r_new_Y;
    uint16_t i;
    int8_t count = 0;

    // Initialize the linear axis
    arc_target[Z_AXIS] = current_position[Z_AXIS];

    float fr_mm_s = MMM_TO_MMS_SCALED(feedrate_mm_m);

    millis_t next_idle_ms = millis() + 200UL;

    for (i = 1; i < segments; i++) { // Iterate (segments-1) times

      thermalManager.manage_heater();
      millis_t now = millis();
      if (ELAPSED(now, next_idle_ms)) {
        next_idle_ms = now + 200UL;
        idle();
      }

      if (++count < N_ARC_CORRECTION) {
        // Apply vector rotation matrix to previous r_X / 1
        r_new_Y = r_X * sin_T + r_Y * cos_T;
        r_X = r_X * cos_T - r_Y * sin_T;
        r_Y = r_new_Y;
      }
      else {
        // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
        // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
        // To reduce stuttering, the sin and cos could be computed at different times.
        // For now, compute both at the same time.
        cos_Ti = cos(i * theta_per_segment);
        sin_Ti = sin(i * theta_per_segment);
        r_X = -offset[X_AXIS] * cos_Ti + offset[Y_AXIS] * sin_Ti;
        r_Y = -offset[X_AXIS] * sin_Ti - offset[Y_AXIS] * cos_Ti;
        count = 0;
      }

      // Update arc_target location
      arc_target[X_AXIS] = center_X + r_X;
      arc_target[Y_AXIS] = center_Y + r_Y;
      arc_target[Z_AXIS] += linear_per_segment;

      clamp_to_software_endstops(arc_target);

        planner.buffer_line(arc_target[X_AXIS], arc_target[Y_AXIS], arc_target[Z_AXIS], fr_mm_s);
    }

    // Ensure last segment arrives at target location.
      planner.buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], fr_mm_s);

    // As far as the parser is concerned, the position is now == target. In reality the
    // motion control system might still be processing the action and the real tool position
    // in any intermediate location.
    set_current_to_destination();
  }
#endif

#if ENABLED(BEZIER_CURVE_SUPPORT)

  void plan_cubic_move(const float offset[4]) {
    cubic_b_spline(current_position, destination, offset, MMM_TO_MMS_SCALED(feedrate_mm_m));

    // As far as the parser is concerned, the position is now == target. In reality the
    // motion control system might still be processing the action and the real tool position
    // in any intermediate location.
    set_current_to_destination();
  }

#endif // BEZIER_CURVE_SUPPORT

void enable_all_steppers() {
  enable_x();
  enable_y();
  enable_z();
}

void disable_all_steppers() {
  disable_x();
  disable_y();
  disable_z();
}

/**
 * Standard idle routine keeps the machine alive
 */
void idle(bool fast
) {
  if(stateManager.get_state() != waiting_file && !card.is_inserted())
  {
      if(stateManager.stop())
        lcd_setstatus("Stop: card removed.");
  }

  lcd_update(fast);
  host_keepalive();
  manage_inactivity(
  );

  thermalManager.manage_heater();

  #if ENABLED(PRINTCOUNTER)
    print_job_timer.tick();
  #endif

  #if HAS_BUZZER && PIN_EXISTS(BEEPER)
    buzzer.tick();
  #endif
}

/**
 * Manage several activities:
 *  - Keep the command buffer full
 *  - Check for maximum inactive time between commands
 *  - Check for maximum inactive time between stepper commands
 *  - Check if pin CHDK needs to go LOW
 *  - Check for KILL button held down
 *  - Check for HOME button held down
 */
void manage_inactivity(bool ignore_stepper_queue/*=false*/) {


  millis_t ms = millis();

  if (max_inactive_time && ELAPSED(ms, previous_cmd_ms + max_inactive_time)) kill(PSTR(MSG_KILLED));

  if (stepper_inactive_time && ELAPSED(ms, previous_cmd_ms + stepper_inactive_time)
      && !ignore_stepper_queue && !planner.blocks_queued()) {
    #if ENABLED(DISABLE_INACTIVE_X)
      disable_x();
    #endif
    #if ENABLED(DISABLE_INACTIVE_Y)
      disable_y();
    #endif
    #if ENABLED(DISABLE_INACTIVE_Z)
      disable_z();
    #endif
  }

  #ifdef CHDK // Check if pin should be set to LOW after M240 set it to HIGH
    if (chdkActive && PENDING(ms, chdkHigh + CHDK_DELAY)) {
      chdkActive = false;
      WRITE(CHDK, LOW);
    }
  #endif

  #if HAS_HOME
    // Check to see if we have to home, use poor man's debouncer
    // ---------------------------------------------------------
    static int homeDebounceCount = 0;   // poor man's debouncing count
    const int HOME_DEBOUNCE_DELAY = 2500;
    if (!READ(HOME_PIN)) {
      if (!homeDebounceCount) {
        enqueue_and_echo_commands_P(PSTR("G28"));
        LCD_MESSAGEPGM(MSG_AUTO_HOME);
      }
      if (homeDebounceCount < HOME_DEBOUNCE_DELAY)
        homeDebounceCount++;
      else
        homeDebounceCount = 0;
    }
  #endif

  planner.check_axes_activity();
}

void kill_pin_handler() {
  kill("Kill switch pressed.");
}

void kill(const char* lcd_msg) {
  stateManager.stop();

  SERIAL_ERROR_START;
  SERIAL_ERRORLNPGM(MSG_ERR_KILLED);

  #if ENABLED(ULTRA_LCD)
    kill_screen(lcd_msg);
  #else
    UNUSED(lcd_msg);
  #endif

  for (int i = 5; i--;)
    delay(100); // Wait a short time

  cli(); // Stop interrupts
  disable_all_steppers();

  #if HAS_POWER_SWITCH
    pinMode(PS_ON_PIN, INPUT);
  #endif

  suicide();

  uint8_t rearm = 0;
  while(rearm != 2)
  {
    if(KILL_PRESSED)
      rearm = 1;
    if(rearm == 1 && !KILL_PRESSED)
      rearm = 2;

    #if ENABLED(USE_WATCHDOG)
      watchdog_reset();
    #endif

    delay(50);
  }

  software_reset();
}

void stop() {
  plasmaManager.lock();
  print_job_timer.stop();
  if (IsRunning()) {
    Running = false;
    Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
    LCD_MESSAGEPGM(MSG_STOPPED);
  }
}
