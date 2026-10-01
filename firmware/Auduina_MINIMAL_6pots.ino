// Auduino, the Lo-Fi granular synthesiser
//
// by Peter Knight, Tinker.it http://tinker.it
//
// key select functionality by Patrick Enright
// Modified: Inverted potentiometer direction for all analog inputs
// Fixed: Boundary protection in scale mapping to prevent overflow
//
// VERSION MINIMA (tutorial): circuito de 6 potenciometros + salida de
// audio con filtro pasabajos RC. Sin jack de entrada CV, sin deteccion
// de jack, sin boton "arm" (ninguno de los dos se usa en este circuito).
//
// Analog in 0: Grain 1 pitch (inverted)
// Analog in 1: Grain 2 decay (inverted)
// Analog in 2: Grain 1 decay (inverted)
// Analog in 3: Grain 2 pitch (inverted)
// Analog in 4: Grain repetition frequency (filtered, inverted)
// Analog in 5: Scale selection
// Digital 3: Audio out (a traves de filtro RC pasabajos)

#include <avr/io.h>
#include <avr/interrupt.h>

// ============================================================
// Pin definitions
// ============================================================
const int scalePin = 5;

// ============================================================
// Note frequency definitions (pentatonic scales)
// ============================================================
int c = 17, cs = 18, d = 19, ds = 20, e = 22, f = 23, fs = 24, g = 26, gs = 27;
int a = 29, as = 31, b = 32, c0 = 34, cs0 = 36, d0 = 38, ds0 = 41, e0 = 43;
int f0 = 46, fs0 = 48, g0 = 51, gs0 = 54, a0 = 58, as0 = 61, b0 = 65;
int c1 = 69, cs1 = 73, d1 = 77, ds1 = 82, e1 = 86, f1 = 92, fs1 = 97, g1 = 103;
int gs1 = 109, a1 = 115, as1 = 122, b1 = 129, c2 = 137, cs2 = 145, d2 = 154;
int ds2 = 163, e2 = 173, f2 = 183, fs2 = 194, g2 = 206, gs2 = 218, a2 = 231;
int as2 = 244, b2 = 259, c3 = 274, cs3 = 291, d3 = 308, ds3 = 326, e3 = 346;
int f3 = 366, fs3 = 388, g3 = 411, gs3 = 435, a3 = 461, as3 = 489, b3 = 518;
int c4 = 549, cs4 = 581, d4 = 616, ds4 = 652, e4 = 691, f4 = 732, fs4 = 776;
int g4 = 822, gs4 = 871, a4 = 923, as4 = 978, b4 = 1036, c5 = 1097, cs5 = 1163;
int d5 = 1232, ds5 = 1305, e5 = 1383, f5 = 1465, fs5 = 1552, g5 = 1644;
int gs5 = 1742, a5 = 1845, as5 = 1955, b5 = 2071, c6 = 2195, cs6 = 2325;
int d6 = 2463, ds6 = 2610, e6 = 2765, f6 = 2930, fs6 = 3104, g6 = 3288;
int gs6 = 3484, a6 = 3691, as6 = 3910, b6 = 4143, c7 = 4389, cs7 = 4650;
int d7 = 4927, ds7 = 5220, e7 = 5530, f7 = 5859, fs7 = 6207, g7 = 6577;
int gs7 = 6968, a7 = 7382, as7 = 7821, b7 = 8286, c8 = 8779, cs8 = 9301;
int d8 = 9854, ds8 = 10440, e8 = 11060, f8 = 11718, fs8 = 12415, g8 = 13153;
int gs8 = 13935, a8 = 14764, as8 = 15642, b8 = 16572, c9 = 17557, cs9 = 18601;
int d9 = 19708, ds9 = 20879, e9 = 22121, f9 = 23436, fs9 = 24830, g9 = 26306;

// ============================================================
// Audio variables
// ============================================================
uint16_t syncPhaseAcc;
uint16_t syncPhaseInc;
uint16_t grainPhaseAcc;
uint16_t grainPhaseInc;
uint16_t grainAmp;
uint8_t grainDecay;
uint16_t grain2PhaseAcc;
uint16_t grain2PhaseInc;
uint16_t grain2Amp;
uint8_t grain2Decay;

// ============================================================
// Analog channel mapping
// ============================================================
#define SYNC_CONTROL         (4)
#define GRAIN_FREQ_CONTROL   (0)
#define GRAIN_DECAY_CONTROL  (2)
#define GRAIN2_FREQ_CONTROL  (3)
#define GRAIN2_DECAY_CONTROL (1)

// ============================================================
// Hardware configuration
// ============================================================
#if defined(__AVR_ATmega8__)
  #define LED_PIN       13
  #define LED_PORT      PORTB
  #define LED_BIT       5
  #define PWM_PIN       11
  #define PWM_VALUE     OCR2
  #define PWM_INTERRUPT TIMER2_OVF_vect
#elif defined(__AVR_ATmega1280__)
  #define LED_PIN       13
  #define LED_PORT      PORTB
  #define LED_BIT       7
  #define PWM_PIN       3
  #define PWM_VALUE     OCR3C
  #define PWM_INTERRUPT TIMER3_OVF_vect
#else
  #define PWM_PIN       3
  #define PWM_VALUE     OCR2B
  #define LED_PIN       13
  #define LED_PORT      PORTB
  #define LED_BIT       5
  #define PWM_INTERRUPT TIMER2_OVF_vect
#endif

// ============================================================
// Antilog table for exponential mapping
// ============================================================
uint16_t antilogTable[] = {
  64830, 64132, 63441, 62757, 62081, 61413, 60751, 60097, 59449, 58809,
  58176, 57549, 56929, 56316, 55709, 55109, 54515, 53928, 53347, 52773,
  52204, 51642, 51085, 50535, 49991, 49452, 48920, 48393, 47871, 47356,
  46846, 46341, 45842, 45348, 44859, 44376, 43898, 43425, 42958, 42495,
  42037, 41584, 41136, 40693, 40255, 39821, 39392, 38968, 38548, 38133,
  37722, 37316, 36914, 36516, 36123, 35734, 35349, 34968, 34591, 34219,
  33850, 33486, 33125, 32768
};

uint16_t mapPhaseInc(uint16_t input) {
  return (antilogTable[input & 0x3f]) >> (input >> 6);
}

// ============================================================
// Pentatonic scales mapping (con inversion interna)
// ============================================================

// Pentatonica de C
uint16_t majorC[46] = {
  c, d, e, g, a, c0, d0, e0, g0, a0, c1, d1, e1, g1, a1,
  c2, d2, e2, g2, a2, c3, d3, e3, g3, a3, c4, d4, e4, g4, a4,
  c5, d5, e5, g5, a5, c6, d6, e6, g6, a6, c7, d7, e7, g7, a7, c8
};

uint16_t mapMajorC(uint16_t input) {
  uint8_t value = (1023 - input) / (1024 / 45);
  if (value >= 46) value = 45;
  return majorC[value];
}

// Pentatonica de G
uint16_t majorG[46] = {
  g, a, b, d0, e0, g0, a0, b0, d1, e1, g1, a1, b1, d2, e2,
  g2, a2, b2, d3, e3, g3, a3, b3, d4, e4, g4, a4, b4, d5, e5,
  g5, a5, b5, d6, e6, g6, a6, b6, d7, e7, g7, a7, b7, d8, e8, g8
};

uint16_t mapMajorG(uint16_t input) {
  uint8_t value = (1023 - input) / (1024 / 45);
  if (value >= 46) value = 45;
  return majorG[value];
}

// Pentatonica de D
uint16_t majorD[46] = {
  d, e, fs, a, b, d0, e0, fs0, a0, b0, d1, e1, fs1, a1, b1,
  d2, e2, fs2, a2, b2, d3, e3, fs3, a3, b3, d4, e4, fs4, a4, b4,
  d5, e5, fs5, a5, b5, d6, e6, fs6, a6, b6, d7, e7, fs7, a7, b7, d8
};

uint16_t mapMajorD(uint16_t input) {
  uint8_t value = (1023 - input) / (1024 / 45);
  if (value >= 46) value = 45;
  return majorD[value];
}

// Pentatonica de A
uint16_t majorA[46] = {
  a, b, cs0, e0, fs0, a0, b0, cs1, e1, fs1, a1, b1, cs2, e2, fs2,
  a2, b2, cs3, e3, fs3, a3, b3, cs4, e4, fs4, a4, b4, cs5, e5, fs5,
  a5, b5, cs6, e6, fs6, a6, b6, cs7, e7, fs7, a7, b7, cs8, e8, fs8, a8
};

uint16_t mapMajorA(uint16_t input) {
  uint8_t value = (1023 - input) / (1024 / 45);
  if (value >= 46) value = 45;
  return majorA[value];
}

// Pentatonica de E
uint16_t majorE[46] = {
  e, fs, gs, b, cs0, e0, fs0, gs0, b0, cs1, e1, fs1, gs1, b1, cs2,
  e2, fs2, gs2, b2, cs3, e3, fs3, gs3, b3, cs4, e4, fs4, gs4, b4, cs5,
  e5, fs5, gs5, b5, cs6, e6, fs6, gs6, b6, cs7, e7, fs7, gs7, b7, cs8, e8
};

uint16_t mapMajorE(uint16_t input) {
  uint8_t value = (1023 - input) / (1024 / 45);
  if (value >= 46) value = 45;
  return majorE[value];
}

// Pentatonica de B
uint16_t majorB[46] = {
  b, cs1, ds1, fs1, gs1, b1, cs2, ds2, fs2, gs2, b2, cs3, ds3, fs3, gs3,
  b3, cs4, ds4, fs4, gs4, b4, cs5, ds5, fs5, gs5, b5, cs6, ds6, fs6, gs6,
  b6, cs7, ds7, fs7, gs7, b7, cs8, ds8, fs8, gs8, b8
};

uint16_t mapMajorB(uint16_t input) {
  uint8_t value = (1023 - input) / (1024 / 45);
  if (value >= 46) value = 45;
  return majorB[value];
}

// Pentatonica de F#
uint16_t majorFs[46] = {
  fs, gs, as, cs0, ds0, fs0, gs0, as0, cs1, ds1, fs1, gs1, as1, cs2, ds2,
  fs2, gs2, as2, cs3, ds3, fs3, gs3, as3, cs4, ds4, fs4, gs4, as4, cs5, ds5,
  fs5, gs5, as5, cs6, ds6, fs6, gs6, as6, cs7, ds7, fs7, gs7, as7, cs8, ds8, fs8
};

uint16_t mapMajorFs(uint16_t input) {
  uint8_t value = (1023 - input) / (1024 / 45);
  if (value >= 46) value = 45;
  return majorFs[value];
}

// Pentatonica de Db
uint16_t majorDb[46] = {
  cs, ds0, f0, gs0, as0, cs1, ds1, f1, gs1, as1, cs2, ds2, f2, gs2, as2,
  cs3, ds3, f3, gs3, as3, cs4, ds4, f4, gs4, as4, cs5, ds5, f5, gs5, as5,
  cs6, ds6, f6, gs6, as6, cs7, ds7, f7, gs7, as7, cs8
};

uint16_t mapMajorDb(uint16_t input) {
  uint8_t value = (1023 - input) / (1024 / 45);
  if (value >= 46) value = 45;
  return majorDb[value];
}

// Pentatonica de Ab
uint16_t majorAb[46] = {
  gs, as, c0, ds0, f0, gs0, as0, c1, ds1, f1, gs1, as1, c2, ds2, f2,
  gs2, as2, c3, ds3, f3, gs3, as3, c4, ds4, f4, gs4, as4, c5, ds5, f5,
  gs5, as5, c6, ds6, f6, gs6, as6, c7, ds7, f7, gs7, as7, c8, ds8, f8, gs8
};

uint16_t mapMajorAb(uint16_t input) {
  uint8_t value = (1023 - input) / (1024 / 45);
  if (value >= 46) value = 45;
  return majorAb[value];
}

// Pentatonica de Eb
uint16_t majorEb[46] = {
  ds, f, g, as, c0, ds0, f0, g0, as0, c1, ds1, f1, g1, as1, c2,
  ds2, f2, g2, as2, c3, ds3, f3, g3, as3, c4, ds4, f4, g4, as4, c5,
  ds5, f5, g5, as5, c6, ds6, f6, g6, as6, c7, ds7, f7, g7, as7, c8, ds8
};

uint16_t mapMajorEb(uint16_t input) {
  uint8_t value = (1023 - input) / (1024 / 45);
  if (value >= 46) value = 45;
  return majorEb[value];
}

// Pentatonica de Bb
uint16_t majorBb[46] = {
  as, c0, d0, f0, g0, as0, c1, d1, f1, g1, as1, c2, d2, f2, g2,
  as2, c3, d3, f3, g3, as3, c4, d4, f4, g4, as4, c5, d5, f5, g5,
  as5, c6, d6, f6, g6, as6, c7, d7, f7, g7, as7, c8, d8, f8, g8, as8
};

uint16_t mapMajorBb(uint16_t input) {
  uint8_t value = (1023 - input) / (1024 / 45);
  if (value >= 46) value = 45;
  return majorBb[value];
}

// Pentatonica de F
uint16_t majorF[46] = {
  f, g, a, c0, d0, f0, g0, a0, c1, d1, f1, g1, a1, c2, d2,
  f2, g2, a2, c3, d3, f3, g3, a3, c4, d4, f4, g4, a4, c5, d5,
  f5, g5, a5, c6, d6, f6, g6, a6, c7, d7, f7, g7, a7, c8, d8, f8
};

uint16_t mapMajorF(uint16_t input) {
  uint8_t value = (1023 - input) / (1024 / 45);
  if (value >= 46) value = 45;
  return majorF[value];
}

// ============================================================
// Filtered read for A4 (evita ruido / saltos, no requiere jack)
// ============================================================
int filteredAnalogRead(uint8_t pin, uint8_t samples = 8) {
  long sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(100);
  }
  return sum / samples;
}

inline int invertedFilteredAnalogRead(uint8_t pin, uint8_t samples = 8) {
  return 1023 - filteredAnalogRead(pin, samples);
}

// ============================================================
// Helper function to invert analog reading (0-1023 -> 1023-0)
// ============================================================
inline int invertedAnalogRead(uint8_t pin) {
  return 1023 - analogRead(pin);
}

// ============================================================
// Setup
// ============================================================
void setup() {
  pinMode(PWM_PIN, OUTPUT);
  audioOn();
  pinMode(LED_PIN, OUTPUT);
  pinMode(scalePin, INPUT);
}

// ============================================================
// Main loop
// ============================================================
void loop() {
  // A4: lectura filtrada e invertida, sin logica de jack (no aplica
  // en este circuito minimo)
  int syncValue = invertedFilteredAnalogRead(SYNC_CONTROL, 8);

  // Las demas entradas usan inversion normal (sin filtro)
  int grainFreqValue = invertedAnalogRead(GRAIN_FREQ_CONTROL);
  int grainDecayValue = invertedAnalogRead(GRAIN_DECAY_CONTROL);
  int grain2FreqValue = invertedAnalogRead(GRAIN2_FREQ_CONTROL);
  int grain2DecayValue = invertedAnalogRead(GRAIN2_DECAY_CONTROL);

  // Scale selector
  int scale = analogRead(scalePin);
  scale = constrain(scale, 0, 1023);
  int scaleSelect = map(scale, 0, 1024, 0, 12);

  // Select scale
  switch (scaleSelect) {
    case 0:  syncPhaseInc = mapMajorC(syncValue);    break;
    case 1:  syncPhaseInc = mapMajorG(syncValue);    break;
    case 2:  syncPhaseInc = mapMajorD(syncValue);    break;
    case 3:  syncPhaseInc = mapMajorA(syncValue);    break;
    case 4:  syncPhaseInc = mapMajorE(syncValue);    break;
    case 5:  syncPhaseInc = mapMajorB(syncValue);    break;
    case 6:  syncPhaseInc = mapMajorFs(syncValue);   break;
    case 7:  syncPhaseInc = mapMajorDb(syncValue);   break;
    case 8:  syncPhaseInc = mapMajorAb(syncValue);   break;
    case 9:  syncPhaseInc = mapMajorEb(syncValue);   break;
    case 10: syncPhaseInc = mapMajorBb(syncValue);   break;
    case 11: syncPhaseInc = mapMajorF(syncValue);    break;
  }

  // Update grain parameters
  grainPhaseInc  = mapPhaseInc(grainFreqValue) / 2;
  grainDecay     = grainDecayValue / 8;
  grain2PhaseInc = mapPhaseInc(grain2FreqValue) / 2;
  grain2Decay    = grain2DecayValue / 4;
}

// ============================================================
// Audio initialization
// ============================================================
void audioOn() {
#if defined(__AVR_ATmega8__)
  TCCR2 = _BV(WGM20) | _BV(COM21) | _BV(CS20);
  TIMSK = _BV(TOIE2);
#elif defined(__AVR_ATmega1280__)
  TCCR3A = _BV(COM3C1) | _BV(WGM30);
  TCCR3B = _BV(CS30);
  TIMSK3 = _BV(TOIE3);
#else
  TCCR2A = _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  TIMSK2 = _BV(TOIE2);
#endif
}

// ============================================================
// PWM Interrupt Service Routine
// ============================================================
SIGNAL(PWM_INTERRUPT) {
  uint8_t value;
  uint16_t output;

  syncPhaseAcc += syncPhaseInc;
  if (syncPhaseAcc < syncPhaseInc) {
    grainPhaseAcc = 0;
    grainAmp = 0x7fff;
    grain2PhaseAcc = 0;
    grain2Amp = 0x7fff;
    LED_PORT ^= 1 << LED_BIT;
  }

  grainPhaseAcc += grainPhaseInc;
  grain2PhaseAcc += grain2PhaseInc;

  value = (grainPhaseAcc >> 7) & 0xff;
  if (grainPhaseAcc & 0x8000) value = ~value;
  output = value * (grainAmp >> 8);

  value = (grain2PhaseAcc >> 7) & 0xff;
  if (grain2PhaseAcc & 0x8000) value = ~value;
  output += value * (grain2Amp >> 8);

  grainAmp -= (grainAmp >> 8) * grainDecay;
  grain2Amp -= (grain2Amp >> 8) * grain2Decay;

  uint8_t scaleShift = 9 - (syncPhaseInc >> 13);
  if (scaleShift < 8) scaleShift = 8;
  output >>= scaleShift;

  if (output > 255) output = 255;
  PWM_VALUE = output;
}
