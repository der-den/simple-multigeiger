// Simple Geiger Counter
// Extracted from MultiGeiger project (https://github.com/ecocurious2/MultiGeiger)
// Only essential parts for reading the Geiger tube and displaying via Serial
// Includes speaker functionality for audible feedback

#include <Arduino.h>
#include <driver/mcpwm.h>
#include <U8x8lib.h>

// Pin definitions
#define PIN_HV_FET_OUTPUT 23
#define PIN_HV_CAP_FULL_INPUT 22  // !! has to be capable of "interrupt on change"
#define PIN_GMC_COUNT_INPUT 2     // !! has to be capable of "interrupt on change"
#define PIN_SPEAKER_OUTPUT_P 12
#define PIN_SPEAKER_OUTPUT_N 0

// OLED Display pin definitions
#define PIN_OLED_RST 16
#define PIN_OLED_SCL 15
#define PIN_OLED_SDA 4

// Dead Time of the Geiger Counter. [usec]
// Has to be longer than the complete pulse generated on the Pin PIN_GMC_COUNT_INPUT.
#define GMC_DEAD_TIME 190

// hw timer period and microseconds -> periods conversion
#define PERIOD_DURATION_US 100
#define PERIODS(us) ((us) / PERIOD_DURATION_US)

// Maximum amount of HV capacitor charge pulses to generate in one charge cycle.
#define MAX_CHARGE_PULSES 3333

// Target loop duration [ms]
#define LOOP_DURATION 1000

// Tube type conversion factor (SBM-20 as default)
#define TUBE_FACTOR (1 / 2.47)

// Volatile variables for ISR communication
volatile bool isr_GMC_cap_full;
volatile unsigned long isr_hv_pulses;
volatile bool isr_hv_charge_error;
volatile unsigned int isr_GMC_counts;
volatile unsigned long isr_count_timestamp;
volatile unsigned long isr_count_time_between;

// MUX (mutexes used for mutual exclusive access to isr variables)
portMUX_TYPE mux_cap_full = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mux_GMC_count = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mux_hv = portMUX_INITIALIZER_UNLOCKED;

// Timer handles
hw_timer_t *timer = NULL;
hw_timer_t *audio_timer = NULL;

// Speaker settings
bool speaker_enabled = true;  // Set to false to disable speaker
bool led_enabled = true;     // Set to false to disable LED

// Display settings
bool display_enabled = true;  // Set to false to disable display

// MUX for audio
portMUX_TYPE mux_audio = portMUX_INITIALIZER_UNLOCKED;

// OLED Display
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(PIN_OLED_RST, PIN_OLED_SCL, PIN_OLED_SDA);

// ISR for high voltage generation
void IRAM_ATTR isr_recharge() {
  // this code is periodically called by a timer hw interrupt, always same period.
  // we need to decide internally whether we actually want to do something.
  static unsigned int current = 0;  // current period counter
  static unsigned int next_state = 0;  // periods to next state machine execution
  static unsigned int next_charge = PERIODS(1000000);  // periods between recharges, initially 1s
  if (++current < next_state)
    return;  // nothing to do yet

  // we reached "next_state", so we execute the state machine:
  current = 0;

  enum State {init, pulse_h, pulse_l, check_full, is_full, charge_fail};
  static State state = init;
  static int charge_pulses;
  if (state == init) {
    charge_pulses = 0;
    portENTER_CRITICAL_ISR(&mux_cap_full);
    isr_GMC_cap_full = 0;
    portEXIT_CRITICAL_ISR(&mux_cap_full);
    state = pulse_h;
    // fall through
  }
  while (state < is_full) {
    if (state == pulse_h) {
      digitalWrite(PIN_HV_FET_OUTPUT, HIGH);  // turn the HV FET on
      state = pulse_l;
      next_state = PERIODS(1500);  // 1500us
      return;
    }
    if (state == pulse_l) {
      digitalWrite(PIN_HV_FET_OUTPUT, LOW);   // turn the HV FET off
      state = check_full;
      next_state = PERIODS(1000);  // 1000us
      return;
    }
    if (state == check_full) {
      charge_pulses++;
      if (isr_GMC_cap_full) {
        state = is_full;
        break;  // capacitor is full, break out of while loop
      }
      if (charge_pulses >= MAX_CHARGE_PULSES) {
        state = charge_fail;
        next_state = 0;  // execute immediately
        return;
      }
      state = pulse_h;
      next_state = 0;  // execute immediately
      return;
    }
  }
  if (state == is_full) {
    // capacitor is full
    portENTER_CRITICAL_ISR(&mux_hv);
    isr_hv_charge_error = false;
    isr_hv_pulses += charge_pulses;
    portEXIT_CRITICAL_ISR(&mux_hv);
    state = init;
    // depending on how many charge pulses we needed (which depends on
    // leak currents (diode leak current depends on temperature), tube type, ...),
    // we might need to charge the HV capacitor more or less often.
    // we target charging with 2 pulses here because if we only needed 1 charge
    // pulse, this does not imply the HV capacitor needed charging. if
    // we needed 2 pulses, we are sure the HV capacitor needed a little charge.
    if (charge_pulses <= 1) {
      // one charge pulse was enough, so maybe we charge too often
      next_charge = next_charge * 5 / 4;
    } else {
      // 2 charge pulses: no change
      // > 2: the more charge pulses we needed, the more frequently we want to recharge
      next_charge = next_charge * 2 / charge_pulses;
    }
    // never go below 1ms or above 10s
    if (next_charge < PERIODS(1000))
      next_charge = PERIODS(1000);
    else if (next_charge > PERIODS(10000000))
      next_charge = PERIODS(10000000);
    next_state = next_charge;
    return;
  }
  if (state == charge_fail) {
    // capacitor does not charge!
    portENTER_CRITICAL_ISR(&mux_hv);
    isr_hv_charge_error = true;
    isr_hv_pulses += charge_pulses;
    portEXIT_CRITICAL_ISR(&mux_hv);
    // let's retry charging later
    state = init;
    next_charge = PERIODS(1000000);  // reset to default 1s charge interval
    next_state = PERIODS(10 * 60 * 1000000);  // wait for 10 minutes before retrying
    return;
  }
}

// ISR for capacitor full detection
void IRAM_ATTR isr_GMC_capacitor_full() {
  portENTER_CRITICAL_ISR(&mux_cap_full);
  isr_GMC_cap_full = 1;
  portEXIT_CRITICAL_ISR(&mux_cap_full);
}

// Flag to indicate a tick is needed
volatile bool tick_requested = false;
volatile bool tick_high = false;

// Speaker tick function - just sets a flag, actual work done in loop()
void IRAM_ATTR tick(bool high) {
  if (speaker_enabled || led_enabled) {
    // Set flag for main loop to handle
    tick_requested = true;
    tick_high = high;
  }
}

// ISR for Geiger-Mueller tube pulse detection
void IRAM_ATTR isr_GMC_count() {
  unsigned long now;
  static unsigned long last;
  portENTER_CRITICAL_ISR(&mux_GMC_count);
  now = micros();  // unsigned long == uint32_t == 32bit -> overflows after ~72 minutes.
  // safely compute dt, even if <now> has overflowed and <last> not [yet]:
  unsigned long dt = (now >= last) ? (now - last) : ((now + (1 << 30)) - (last + (1 << 30)));
  if (dt > GMC_DEAD_TIME) {
    // We only consider a pulse valid if it happens more than GMC_DEAD_TIME after the last valid pulse.
    // Reason: Pulses occurring short after a valid pulse are false pulses generated by the rising edge on the PIN_GMC_COUNT_INPUT.
    //         This happens because we don't have a Schmitt trigger on this controller pin.
    isr_count_timestamp = millis();        // remember (system) time of the pulse
    isr_count_time_between = dt;           // save for statistics debuging
    isr_GMC_counts++;                      // count the pulse
    last = now;                            // remember timestamp of last **valid** pulse
    
    // Generate a tick sound for each valid pulse
    tick(true);
  }
  portEXIT_CRITICAL_ISR(&mux_GMC_count);
}

// Read high voltage status
void read_hv(bool *hv_error, unsigned long *pulses) {
  portENTER_CRITICAL(&mux_hv);
  *pulses = isr_hv_pulses;
  *hv_error = isr_hv_charge_error;
  portEXIT_CRITICAL(&mux_hv);
}

// Read Geiger-Mueller counter values
void read_GMC(unsigned long *counts, unsigned long *timestamp, unsigned int *between) {
  portENTER_CRITICAL(&mux_GMC_count);
  *counts += isr_GMC_counts;
  isr_GMC_counts = 0;
  *timestamp = isr_count_timestamp;
  *between = isr_count_time_between;
  portEXIT_CRITICAL(&mux_GMC_count);
}

// Setup timer for recharge
void setup_recharge_timer(void (*fn)(void), uint64_t us) {
  // Create timer with 1MHz frequency (1us resolution)
  timer = timerBegin(1000000);
  
  // Attach the interrupt handler
  timerAttachInterrupt(timer, fn);
  
  // Set alarm with auto-reload
  timerAlarm(timer, us, true, 0);
  
  // Start the timer
  timerStart(timer);
}

// Setup speaker
void setup_speaker() {
  // Configure LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  // Configure speaker pins
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PIN_SPEAKER_OUTPUT_P);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, PIN_SPEAKER_OUTPUT_N);
  
  // Configure MCPWM
  mcpwm_config_t pwm_config;
  pwm_config.frequency = 1000;
  pwm_config.cmpr_a = 50.0;  // 50% duty cycle
  pwm_config.cmpr_b = 50.0;  // 50% duty cycle
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;  // active high duty
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
  
  // Initial state: speaker off
  mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
  mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
  
  // Play startup sound if speaker is enabled
  if (speaker_enabled) {
    // Simple startup beep
    mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, 2000);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_1);
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
    delay(100);
    mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
    delay(50);
    
    mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, 3000);
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
    delay(100);
    mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
  }
}

// Setup display
void setup_display() {
  if (!display_enabled) {
    return;
  }
  
  // Initialize display
  u8x8.begin();
  u8x8.clear();
  
  // Show startup screen
  u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
  u8x8.drawString(0, 0, "Simple Geiger");
  u8x8.drawString(0, 1, "Counter");
  
  // Wait a moment to show the startup screen
  delay(1000);
}

void setup() {
  // Initialize Serial
  Serial.begin(115200);
  Serial.println("Simple Geiger Counter Starting...");
  
  // Setup pins
  pinMode(PIN_HV_FET_OUTPUT, OUTPUT);
  pinMode(PIN_HV_CAP_FULL_INPUT, INPUT);
  pinMode(PIN_GMC_COUNT_INPUT, INPUT);

  digitalWrite(PIN_HV_FET_OUTPUT, LOW);

  // Initialize variables
  isr_count_timestamp = 0;
  isr_count_time_between = 0;
  isr_GMC_cap_full = 0;
  isr_GMC_counts = 0;
  isr_hv_pulses = 0;
  isr_hv_charge_error = false;

  // Setup speaker
  setup_speaker();
  
  // Setup display
  setup_display();
  
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(PIN_HV_CAP_FULL_INPUT), isr_GMC_capacitor_full, RISING);  // capacitor full
  attachInterrupt(digitalPinToInterrupt(PIN_GMC_COUNT_INPUT), isr_GMC_count, FALLING);            // GMC pulse detected

  // Setup timer for high voltage generation
  setup_recharge_timer(isr_recharge, PERIOD_DURATION_US);
  
  Serial.println("Setup complete - Speaker " + String(speaker_enabled ? "enabled" : "disabled") + 
                 ", Display " + String(display_enabled ? "enabled" : "disabled"));
}

void loop() {
  static bool hv_error = false;  // true means a HV capacitor charging issue
  static unsigned long hv_pulses = 0;
  static unsigned long gm_counts = 0;
  static unsigned long gm_count_timestamp = 0;
  static unsigned int gm_count_time_between = 0;
  static unsigned long last_counts = 0;
  static unsigned long last_timestamp = 0;
  
  unsigned long current_ms = millis();  // to save multiple calls to millis()

  // Read Geiger-Mueller counter values
  read_GMC(&gm_counts, &gm_count_timestamp, &gm_count_time_between);

  // Read high voltage status
  read_hv(&hv_error, &hv_pulses);
  
  // Handle tick request from ISR (safe to do this in the main loop)
  if (tick_requested) {
    // Generate a tick sound and/or LED flash
    if (speaker_enabled) {
      // Set frequency based on high/low tick
      int freq = tick_high ? 5000 : 1000;
      
      // Generate a short beep
      mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, freq);
      mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
      mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_1);
      mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
      
      // Stop after a very short time
      delay(4);
      mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
      mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
      mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
    }
    
    if (led_enabled && tick_high) {
      // Flash the LED
      digitalWrite(LED_BUILTIN, HIGH);
      delay(4);
      digitalWrite(LED_BUILTIN, LOW);
    }
    
    // Reset flag
    tick_requested = false;
  }

  // Print data every second
  if ((current_ms - last_timestamp) >= LOOP_DURATION) {
    last_timestamp = current_ms;
    
    // Calculate counts in the last period
    unsigned long counts = gm_counts - last_counts;
    last_counts = gm_counts;
    
    // Calculate CPM (counts per minute)
    unsigned int cpm = counts * 60;
    
    // Calculate µSv/h using the tube factor
    float usvh = cpm * TUBE_FACTOR;
    int usvh_nanos = (int)(usvh * 1000); // Convert to nanosieverts for display
    
    // Print the data
    Serial.print("Counts: ");
    Serial.print(counts);
    Serial.print(" | CPM: ");
    Serial.print(cpm);
    Serial.print(" | µSv/h: ");
    Serial.print(usvh, 4);
    Serial.print(" | HV Status: ");
    Serial.print(hv_error ? "ERROR" : "OK");
    Serial.print(" | HV Pulses: ");
    Serial.println(hv_pulses);
    
    // Update display if enabled
    if (display_enabled) {
      // Clear display
      u8x8.clear();
      
      // Show radiation value in large font
      u8x8.setFont(u8x8_font_inb33_3x6_n);
      char output[20];
      sprintf(output, "%5d", usvh_nanos);
      u8x8.drawString(0, 0, output);
      
      // Show unit in smaller font
      u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
      u8x8.drawString(0, 6, "nSv/h");
    }
  }
  
  // Delay to achieve target loop duration
  long loop_duration = millis() - current_ms;
  if (loop_duration < LOOP_DURATION) {
    delay(LOOP_DURATION - loop_duration);
  }
}
