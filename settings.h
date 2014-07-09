

#define BASE_RATE 16000 // The base sample rate of the audio

// Pins
#define POT_PIN A1     // Pot wiper when using pot mode
#define POT_CS  4      // MCP4131 CS // If using a digi pot to control volume these are the pins
#define POT_SCK 5      // MCP4131 Clock 
#define POT_SDO 6      // MCP4131 Data


#define DEFAULT_VOLUME 127      // Volume when in non managed mode
#define VOL_MIN 20              // Min volume in managed mode 0 - 127
#define VOL_MAX 127             // Max volume in managed mode 0 - 127
#define TOP_SPEED_MULTIPLIER 15 // RPM multiplier in managed mode, bigger the number the larger the rev range, 10 - 15 is a good place to start

