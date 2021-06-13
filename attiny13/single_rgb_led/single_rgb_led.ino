
// CPU-Speed 4.8 MHz to work without problems at 3.0V
#define F_CPU 4800000
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define DELAY_US 30
#define RED_BRIGHTNESS rgbValues[0]
#define GREEN_BRIGHTNESS rgbValues[1]
#define BLUE_BRIGHTNESS rgbValues[2]
#define MODE_COUNT 6
#define MODE_STATIC 0
#define MODE_FADING 1
#define MODE_RAINBOW 2
#define MODE_FIRE 3
#define MODE_HEARTBEAT 4
#define MODE_SLEEP 5
#define RANDOM_SEED (161)
#define DEBOUNCE_CYCLES 1000

#define RED PB1
#define GREEN PB0
#define BLUE PB2
#define SWITCH_BUTTON PB4
#define DELAY_US 30

/**
 * Generate a pseudo random 16-bit integer
 */
static uint16_t random_uint16(void) {
    static uint16_t currentRandomSeedValue = RANDOM_SEED;
    return (currentRandomSeedValue = (currentRandomSeedValue >> 1) ^ (-(currentRandomSeedValue & 1) & 0xB400));
}

int main(void) {
  uint8_t mode = MODE_SLEEP;

  uint8_t buttonIsPressedOldState = 1;

  uint16_t debounceCycles = 0;

  // extraData contains extra information in every mode, which can be used to calculate the effect
  uint16_t extraData = 0;

  // Led pins as output pins
  DDRB |= (1 << GREEN) | (1 << RED) | (1 << BLUE);

  // Switch button pin as input pin
  DDRB &= ~(1 << SWITCH_BUTTON);
  // Enable switch button pullup
  PORTB |= (1 << SWITCH_BUTTON);

  // Brightness values of red green and blue (accessed from the macros RED_BRIGHTNESS, BLUE_BRIGHTNESS and GREEN_BRIGHTNESS)
  uint8_t rgbValues[] = {0, 0, 0};

  /** 
   *  pwm_state is the iterator variable for the software pwm signal. It will 
   *  be increased by 1 every iteration and the output registers will
   *  be activated depending on the brightness of the color and the
   *  current state of s
   */
  uint8_t pwm_state = 0;

  while (1) {
    // Only update the mode data on pwm_state == 0, between 1 and 255 every calculation is only for the software pwm
    if (pwm_state == 0) {
      switch (mode) {
        case MODE_FADING:
          BLUE_BRIGHTNESS += extraData;
          RED_BRIGHTNESS += extraData;
          GREEN_BRIGHTNESS += extraData;
  
          if (BLUE_BRIGHTNESS == 255) {
            extraData = -1;
          } else if (BLUE_BRIGHTNESS == 0) {
            extraData = 1;  
          }
          break;
  
  
        case MODE_STATIC:
          BLUE_BRIGHTNESS = 255;
          RED_BRIGHTNESS = 255;
          GREEN_BRIGHTNESS = 255;
          break;
  
  
        case MODE_RAINBOW:
          switch (extraData) {
            case 0:
              BLUE_BRIGHTNESS = 255;
              RED_BRIGHTNESS = 0;
              GREEN_BRIGHTNESS = 0;
              extraData = 1;
              break;
            case 1:
              BLUE_BRIGHTNESS -= 1;
              RED_BRIGHTNESS += 1;
              if (BLUE_BRIGHTNESS == 0) {
                extraData = 2;
              }
              break;
            case 2:
              RED_BRIGHTNESS -= 1;
              GREEN_BRIGHTNESS += 1;
              if (RED_BRIGHTNESS == 0) {
                extraData = 3;
              }
              break;
            case 3:
              GREEN_BRIGHTNESS -= 1;
              BLUE_BRIGHTNESS += 1;
              if (GREEN_BRIGHTNESS == 0) {
                extraData = 1;
              }
              break;
              
          }
          break;
  
  
        case MODE_FIRE:
          if (extraData == 0) {
            uint16_t randVal = random_uint16();
            RED_BRIGHTNESS = randVal & 0xFF;
            GREEN_BRIGHTNESS = (randVal & 0xFF) / 8;
            BLUE_BRIGHTNESS = 0;
            extraData = ((uint8_t)((randVal > 8) & 0xFF)) * 8;
          } else {
            extraData--;
          }
          break;
  
  
        case MODE_HEARTBEAT:
          GREEN_BRIGHTNESS = 0;
          BLUE_BRIGHTNESS = 0;
          if (extraData < 8) {
            RED_BRIGHTNESS = ((uint8_t)extraData) * 16;
          } else if (extraData < 16) {
            RED_BRIGHTNESS = (16 - ((uint8_t)extraData)) * 16;
          } else if (extraData < 26) {
            RED_BRIGHTNESS = 0;
          } else if (extraData < 34) {
            RED_BRIGHTNESS = (((uint8_t)extraData) - 26) * 16;
          } else if (extraData < 42) {
            RED_BRIGHTNESS = (34 - ((uint8_t)extraData)) * 16;
          } else {
            RED_BRIGHTNESS = 0;
          }
          extraData = (extraData + 1) % 120;
          break;
  
        case MODE_SLEEP:
          // Turn all lights off
          PORTB &= ~((1 << RED) | (1 << GREEN) | (1 << BLUE));

          // Only switch to sleepMode if the button is released, to prevent the AtTiny to wake up immediately
          if (!buttonIsPressedOldState) {
            // Start deep sleep mode
            sleepMode();
            mode = 0;

            // Since we come from an interrupt, we have to save, that the button is down
            buttonIsPressedOldState = 1;
            // Start debouncing
            debounceCycles = 1;
          }
          break;
      }
    }
    
    /**
     * The folowing part is for the calculation of the software pwm
     * Every color can have 256 brightness states and is active for
     * the amount of cycles
     */
    if (RED_BRIGHTNESS > pwm_state) {
      PORTB |= (1 << RED);
    } else {
      PORTB &= ~(1 << RED);
    }
    if (GREEN_BRIGHTNESS > pwm_state) {
      PORTB |= (1 << GREEN);
    } else {
      PORTB &= ~(1 << GREEN);
    }
    if (BLUE_BRIGHTNESS > pwm_state) {
      PORTB |= (1 << BLUE);
    } else {
      PORTB &= ~(1 << BLUE);
    }
    pwm_state++;

    // Since the input pullup is activated a 0 means the button is pressed
    bool buttonIsCurrentlyPressed = (PINB & (1 << SWITCH_BUTTON)) == 0;
 
    bool isDebouncing = debounceCycles > 0;

    if (isDebouncing) {
      debounceCycles++;
      if (debounceCycles > DEBOUNCE_CYCLES) {
        debounceCycles = 0;
        isDebouncing = false;
      }
    }

    // Button release, we also debounce the button release
    if (!buttonIsCurrentlyPressed && buttonIsPressedOldState && !isDebouncing) {
      buttonIsPressedOldState = 0;
      debounceCycles = 1;
    }

    // Make sure that only when the button is pressed down the mode is changed
    if (!buttonIsPressedOldState && buttonIsCurrentlyPressed && !isDebouncing) {
      mode = (mode + 1) % MODE_COUNT;
      buttonIsPressedOldState = 1;
      extraData = 0;
      debounceCycles = 1;
    }
    _delay_us(DELAY_US);
  }
  return 0;
}

static inline void sleepMode(void) {
  GIMSK |= (1 << PCIE);   // pin change interrupt enable
  PCMSK |= (1 << PCINT4); // pin change interrupt enabled for PCINT4

  // debounce 
  delay(100);
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sei();
  sleep_mode();
  cli();
}

// The ISR is only needed for waking the AtTiny up
ISR(PCINT0_vect) {}
