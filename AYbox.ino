// AYbox.
// Copyright (C) Aleh Dzenisiuk. All rights reserved.
//
// This is to turn an old music chip (YM2149, pin compatible with AY-8910) used by ZX Spectrum 
// and many other home computers into a simple chiptune MIDI synth.

#include <a21.hpp>

using namespace a21;

#define _1(x) (1 << (x))

#define _0(x) 0

/**
 * Wraps access to Yamaha YM2149 sound chip which is pin-compatible with AY-8910.
 * Uses Timer 2 to provide the chip with 1MHz clock. pinCLOCK should correspond to OC2A pin. 
 */
template<
  typename dataIO,
  typename pinBDIR, typename pinBC1, typename pinBC2,
  typename pinRESET, typename pinCLOCK
>
class YM2149 {

protected:

  static void setupClock() {

    // The output driver on OC2A has to be enabled.
    pinCLOCK::setOutput();
    
    // Setting up Timer 2 to generate 1MHz clock on OC2A pin.
    TCCR2A = _0(COM2A1) | _1(COM2A0) // OC2A: toggle on Compare Match.
      | _0(COM2B1) | _0(COM2B0) // OC2B: normal port operation.
      | _1(WGM21) | _0(WGM20);  // CTC mode.
    OCR2A = 0; // Will toggle OC2A on every CLK, so when the CLK is 2MHz, then it will give 1MHz on OC2A pin.
    TIMSK2 = 0;
    TCCR2B = _0(WGM22) // MSB of the CTC mode.
      | _0(CS22) | _1(CS21) | _0(CS20); // Prescaler: CLK/8, assuming 16 MHz clock.
  }
  
  /** Resets the chip. Use begin() for full initialization. */
  static void reset() {

    pinRESET::setOutput();
    pinRESET::setLow();
    
    // Reset Pulse Width, Trw = 500ns.
    _delay_us(2 * 0.500);

    // Reset should have a built-in pull-up.
    pinRESET::setInput(false); 
    
    // Reset Bus Control Delay Time, Trb = 100ns.
    _delay_us(2 * 0.100);
  }

public:

  /** The clock frequency of the chip, Hz. */
  static const uint32_t F = 1000000;

  /** Prepares all the pins, sets up the clock and resets the chip. */
  static void begin() {
    
    dataIO::setOutput();
    
    pinBDIR::setOutput();
    pinBC1::setOutput();
    pinBC2::setOutput();

    // Bus: inactive state.
    pinBDIR::setLow();
    pinBC1::setLow();
    pinBC2::setHigh();
    
    setupClock();

    reset();
  }

  /** Writes directly to the given register. */
  static void write(uint8_t reg, uint8_t data) {

    // Bus: assuming BC2 always stays HIGH.

    // Bus: latch the address.
    pinBDIR::setHigh();
    pinBC1::setHigh();

    // Set up the register address.
    dataIO::write(reg);

    // Address Setup Time, Tas = 300ns.
    _delay_us(2 * 0.300);

    // Bus: inactive.
    pinBDIR::setLow();
    pinBC1::setLow();

    // Address Hold Time, Tah = 50ns.
    _delay_us(2 * 0.050);

    // Set up the data.
    dataIO::write(data);

    // Write Data Setup Time, Tds = 0ns.
    _delay_us(0);
    
    // Bus: write to PSG.
    pinBDIR::setHigh();
    pinBC1::setLow();

    // Write Signal Time, Tdw > 300ns.
    _delay_us(2 * 0.300);

    // Bus: back to inactive.
    pinBDIR::setLow();
    pinBC1::setLow();

    // Write Data Hold Time, Tdh = 80ns.
    _delay_us(2 * 0.080);
  }

  enum ChannelIndex : uint8_t {
    A = 0,
    B = 1,
    C = 2
  };

  /** @{ */
  /** Immediate access routines. */

  /** Sets period for the given tone channel. 
   * The master clock is divided by 16 by the chip before it is used with the tone generator. */
  static void setTonePeriod(ChannelIndex channel, uint16_t period) {
    write(channel * 2, (uint8_t)period);
    write(channel * 2 + 1, (uint8_t)(period >> 8));
  }

  /** Sets the period of the noise generator, which is shared among all the channels. 
   * The master clock is divided by 16 by the chip before it is used with the noise generator. */
  static void setNoisePeriod(uint8_t period) {
    write(6, period);
  }

  enum MixerSetup {
    
    ToneA = 1 << 0, 
    ToneB = 1 << 1,
    ToneC = 1 << 2,
    
    NoiseA = 1 << 3,
    NoiseB = 1 << 4,
    NoiseC = 1 << 5,
    
    PortA = 1 << 6,
    PortB = 1 << 7
  };

  /** Writes the mixer setup register directly with the provided value. Note that port A and B bits are always set to disable the IO ports. */
  static void setupMixer(MixerSetup setup) {
    write(7, ~setup); 
  }

  /** Sets amplitude (0-0xF) and envelope mode (i.e. if the envelope should be used) for the given channel. */
  static void setAmplitude(ChannelIndex channel, bool useEnvelope, uint8_t amplitude) {
    write(8 + channel, useEnvelope ? (0x10 | amplitude) : amplitude);
  }

  /** The master clock is divided by 256 by the chip before it is used by the envelope generator. */
  static void setEnvelopePeriod(uint16_t period) {
    write(11, (uint8_t)(period));
    write(12, (uint8_t)(period >> 8));
  }

  enum EnvelopeShape : uint8_t {
    
    Hold = 1 << 0,
    Alternate = 1 << 1,
    Attack = 1 << 2,
    Continue = 1 << 3,

    EnvelopeDecay = 0,
    EnvelopeAttack = Continue | Attack | Hold,
    EnvelopeTriangle = Continue | Attack | Alternate, 
  };
  
  static void setEnvelopeShape(EnvelopeShape shape) {
    write(13, shape);
  }
  
  /** @} */
};

template<typename AY>
class AYRender {

public:

  /** @{ */
  /** The whole state of the chip. You change anything directly, then call render() to write everything to the chip. */  

  static const uint8_t ChannelCount = 3;

  /** To group the state of a single channel. */
  struct Channel {

    /** The current period of the tone generator of this channel (in clock periods multiplied by 16). */
    uint16_t tonePeriod16;

    // True, if the output of the tone generator of this channel should be mixed in.
    bool toneEnabled;    

    // True, if the output of the shared noise generator should be mixed in.
    bool noiseEnabled;

    // True, if the shared enveope generator should be used to control the volume in this channel.
    bool envelopeEnabled;

    // The current volume of the channel, 0-0xF.
    uint8_t volume;
  };

  Channel channels[ChannelCount];

  /** The shape of the envelope of the shared envelope generator. */
  typename AY::EnvelopeShape envelopeShape;

  /** The period of the shared envelope generator (in clock periods multiplied by 256). */
  uint16_t envelopePeriod256;

  /** The period of the shared noise generator (in clock periods multiplied by 16). */
  uint16_t noisePeriod16;

  /** Writes the shadow state into the chip. */
  void render() {
    
    typename AY::MixerSetup mixerSetup = 0;

    if (channels[0].toneEnabled)
      mixerSetup |= AY::ToneA;    
    if (channels[0].noiseEnabled)
      mixerSetup |= AY::NoiseA;
      
    if (channels[1].toneEnabled)
      mixerSetup |= AY::ToneB;    
    if (channels[1].noiseEnabled)
      mixerSetup |= AY::NoiseB;
      
    if (channels[2].toneEnabled)
      mixerSetup |= AY::ToneC;
    if (channels[2].noiseEnabled)
      mixerSetup |= AY::NoiseC;

    AY::setEnvelopePeriod(envelopePeriod256);
    AY::setEnvelopeShape(envelopeShape);

    AY::setTonePeriod(AY::A, channels[0].tonePeriod16);
    AY::setAmplitude(AY::A, channels[0].envelopeEnabled, channels[0].volume);
    
    AY::setTonePeriod(AY::B, channels[1].tonePeriod16);
    AY::setAmplitude(AY::B, channels[1].envelopeEnabled, channels[1].volume);
    
    AY::setTonePeriod(AY::C, channels[2].tonePeriod16);
    AY::setAmplitude(AY::C, channels[2].envelopeEnabled, channels[2].volume);

    AY::setNoisePeriod(noisePeriod16);

    AY::setupMixer(mixerSetup);
  }

  AYRender() {
    for (uint8_t i = 0; i < ChannelCount; i++) {
      channels[i].toneEnabled = false;
      channels[i].noiseEnabled = false;
      channels[i].envelopeEnabled = false;
    }
    noisePeriod16 = 0;
    envelopePeriod256 = 0;
  }

  /** @} */  
};

typedef YM2149<
  PinBus< FastPin<2>, FastPin<3>, FastPin<4>, FastPin<5>, FastPin<6>, FastPin<7>, FastPin<8>, FastPin<9> >,
  FastPin<12>,  // BDIR
  FastPin<10>,  // BC1.
  UnusedPin<>,  // BC2, always tied to VCC.
  UnusedPin<>,  // RESET, can be wired to stay low for some time at startup.
  FastPin<11>   // CLOCK. This must be the pin corresponding to OC2A, which is 11 on Arduino Uno/Nano.
> AY;

typedef FastPin<13> DebugLED;

typedef SoftwareI2C< FastPin<A4>, FastPin<A5>, true, 100000L > I2C;

// 128x32 (i.e. 4 "pages" high).
typedef SSD1306<I2C, 4> LCD;

typedef FastPin<A0> encoder1PinA;
typedef FastPin<A1> encoder1PinB;
EC11 encoder1;

typedef FastPin<A2> encoderButton;

typedef FastPin<A3> button1Pin;
typedef FastPin<A6> button2Pin;

typedef FastPin<A7> mixPin;

class AYbox : protected a21::MIDIParser<AYbox> {

protected:

  friend a21::MIDIParser<AYbox>;

  typedef AYbox Self;

  static Self& getSelf() {
    static Self self = Self();
    return self;
  }

  AYRender<AY> ayRender;

  /** @{ */

  struct SampleEntry {
    
    uint8_t volume : 4;
    bool envelopeEnabled : 1;
    bool toneEnabled : 1;
    bool noiseEnabled : 1;

    SampleEntry(uint8_t volume = 0, bool toneEnabled = true, bool noiseEnabled = false, bool envelopeEnabled = true) 
      : volume(volume), toneEnabled(toneEnabled), noiseEnabled(noiseEnabled), envelopeEnabled(envelopeEnabled)
    {}
  };

  static int const EntriesPerSample = 16;
  
  struct Sample {
    
    SampleEntry entries[EntriesPerSample];
    uint8_t loopStart;
    uint8_t loopLength;
    
    AY::EnvelopeShape envelopeShape;
    uint16_t envelopePeriod;
  };

  enum ChannelState {
    
    ChannelStateOff = 0,
    
    /** Attack state actually includes the "sustained" (looped) part as well. */
    ChannelStateAttack,

    /** The key has been release, playing the "release" part of the sample, the one after the looped part. */
    ChannelStateRelease
  };

  struct Channel {

    /** The index of the channel, for convenience. */
    uint8_t index;

    ChannelState state;
    
    /** The MIDI note that triggered this channel; 0 when the state is 'off'. */
    uint8_t note;

    uint8_t velocity;

    /** The current "sample" in this channel. */
    Sample sample;

    /** The current position within the channel's sample. */
    uint8_t samplePosition;
  };
  
  static int const ChannelCount = 3;
    
  Channel channels[ChannelCount];

  /** A channel where the given MIDI note is being played now. */
  Channel* channelForNote(uint8_t note) {
    
    for (uint8_t i = 0; i < ChannelCount; i++) {
      Channel *ch = &channels[i];
      if (ch->note == note)
        return ch;
    }
    
    // Not found.
    return NULL;
  }  

  /** A channel where we can begin playing the given MIDI note. */
  Channel* allocateChannel(uint8_t note) {

    // First check if playing the same note already, then can just choke it.
    Channel *ch = channelForNote(note);
    if (ch)
      return ch;

    // OK, not playing this note yet. Let's try to grab the next silent.
    for (uint8_t i = 0; i < ChannelCount; i++) {
      Channel* ch = &channels[i];
      if (ch->state == ChannelStateOff)
        return ch;
    }

    // No luck, let's see if we have a channel where a release part of the sample is played.
    for (uint8_t i = 0; i < ChannelCount; i++) {
      Channel* ch = &channels[i];
      if (ch->state == ChannelStateRelease)
        return ch;
    }

    // OK, all are busy, no luck for now.
    // TODO: we could try to choke the oldest playing for example.
    return NULL;
  }  
  
  /** @} */

  /** @{ */
  /** MIDI */

  void handleNoteOn(uint8_t channel, uint8_t note, uint8_t velocity) {

    Channel *ch = allocateChannel(note);
    if (!ch)
      return;

    ch->state = ChannelStateAttack;
    ch->note = note;  
    ch->velocity = velocity;
    ch->samplePosition = 0;

    Sample& sample = ch->sample;
    sample.envelopeShape = AY::EnvelopeAttack;
    sample.envelopePeriod = 1000;
    
    sample.loopStart = 2;
    sample.loopLength = 1;
    SampleEntry *entries = sample.entries;
    entries[0] = SampleEntry(5, true, false, false);
    entries[1] = SampleEntry(10, true, false, false);
    entries[2] = SampleEntry(15, true, false, false);
    entries[3] = SampleEntry(15, true, false, false);
    entries[4] = SampleEntry(15, true, false, false);
    entries[5] = SampleEntry(14, true, false, false);
    entries[6] = SampleEntry(14, true, false, false);
    entries[7] = SampleEntry(13, true, false, false);
    entries[8] = SampleEntry(13, true, false, false);
    entries[9] = SampleEntry(12, true, false, false);
    entries[10] = SampleEntry(12, true, false, false);
    entries[11] = SampleEntry(11, true, false, false);
    entries[12] = SampleEntry(10, true, false, false);
    entries[13] = SampleEntry(9, true, false, false);
    entries[14] = SampleEntry(8, true, false, false);
    entries[15] = SampleEntry(5, true, false, false);
  }
  
  void handleNoteOff(uint8_t channel, uint8_t note, uint8_t velocity) {
    
    Channel *ch = channelForNote(note);
    if (!ch)
      return;

    ch->state = ChannelStateRelease;
  }
  
  void handlePolyAftertouch(uint8_t channel, uint8_t note, uint8_t velocity) {
  }
  
  void handleControlChange(uint8_t channel, uint8_t control, uint8_t value) {
  }
  
  void handleProgramChange(uint8_t channel, uint8_t program) {
  }
  
  void handleAftertouch(uint8_t channel, uint8_t value) {
  }
  
  void handlePitchBend(uint8_t channel, uint16_t value) {
  }
  
  /** @} */

  uint16_t prevTickMillis;

  // TODO: use a table
  static uint16_t frequencyForNote(uint8_t note) {
    return (220.0 / 8) * pow(pow(2, 1.0 / 12), (note - 21));
  }

  static uint16_t period16FromFrequency(uint16_t frequency) {
    return AY::F / ((uint32_t)frequency << 4);
  }  

  uint16_t envelopePeriodTest;
  AY::EnvelopeShape envelopeShape;

  void tick() {

    ayRender.noisePeriod16 = 0;
    //! ayRender.envelopePeriod256 = 0;
    //! ayRender.envelopeShape = 0;
    
    for (uint8_t i = 0; i < ChannelCount; i++) {
      
      Channel& ch = channels[i];

      AYRender<AY>::Channel& ayChannel = ayRender.channels[i];
      
      if (ch.state != ChannelStateOff) {
        
        ayChannel.tonePeriod16 = period16FromFrequency(frequencyForNote(ch.note));

        const Sample& sample = ch.sample;
        const SampleEntry& sampleEntry = ch.sample.entries[ch.samplePosition];

        // How much we should take velocity into account, 0-16.
        const uint8_t velocitySense = 4;
        uint16_t v = sampleEntry.volume;
        ayChannel.volume = (v * (16 - velocitySense) + v * (ch.velocity + 1) * velocitySense / 128) / 16;
        ayChannel.noiseEnabled = sampleEntry.noiseEnabled;
        ayChannel.toneEnabled = sampleEntry.toneEnabled;
        ayChannel.envelopeEnabled = sampleEntry.envelopeEnabled;

        //
        // Envelope and noise are set from the first non-silent channel.
        //
        if (ayRender.noisePeriod16 == 0) {
          // For noise let's always use the pitch of the note.
          ayRender.noisePeriod16 = ayChannel.tonePeriod16;
        }
        /*!
        if (ayRender.envelopePeriod256 == 0) {
          ayRender.envelopePeriod256 = sample.envelopePeriod;
          ayRender.envelopeShape = sample.envelopeShape;
        }  
        */      

        //
        // Loop.
        //
        ch.samplePosition++;

        if ((ch.state == ChannelStateAttack)
          && (ch.samplePosition >= sample.loopStart + sample.loopLength)
        ) {
          ch.samplePosition = ch.sample.loopStart;
        }
          
        if (ch.samplePosition >= EntriesPerSample) {
          ch.state = ChannelStateOff;
          ch.note = 0;
        }
        
      } else {

        ayChannel.toneEnabled = false;
        ayChannel.noiseEnabled = false;
        ayChannel.envelopeEnabled = false;
        ayChannel.volume = 0;
        ayChannel.tonePeriod16 = 0;
      }
    }

    //~ ayRender.envelopePeriod256 = envelopePeriodTest;
    //~ ayRender.envelopeShape = envelopeShape;

    ayRender.render();    
  }
  
public:

  static void begin() {

    Self& self = getSelf();

    AY::begin();
    
    Serial.begin(31250);

    static_cast< MIDIParser<AYbox>& >(self).begin();

    for (uint8_t i = 0; i < ChannelCount; i++) {
      Channel& ch = self.channels[i];
      ch.index = i;
      ch.state = ChannelStateOff;
    }    

    I2C::begin();
    LCD::begin();
    LCD::setFlippedVertically(false);
    LCD::setContrast(10);
    
    LCD::clear();
    LCD::drawTextCentered(Font8Console::data(), 0, 1, LCD::Cols, "AY BOX", Font8::DrawingScale2);
    
    LCD::turnOn();
    
    DebugLED::setOutput();
    DebugLED::setHigh();
    delay(1000);
    DebugLED::setLow();

    self.prevTickMillis = millis();
    
    self.tick();

    self.envelopePeriodTest = 1000;
  }

  void draw() {
    LCD::clear();
    char buf[20];
    sprintf(buf, "Period: %u", (unsigned int)envelopePeriodTest);
    LCD::drawTextCentered(Font8Console::data(), 0, 1, LCD::Cols, buf, Font8::DrawingScale1);
  }
  
  static void check() {

    Self& self = getSelf();

    // Calling the tick handler without a dedicated timer for now.
    uint16_t now = millis();
    if ((uint16_t)(now - getSelf().prevTickMillis) >= 20) {
      self.prevTickMillis = now;
      self.tick();
    }
    
    if (Serial.available()) {
      self.handleByte(Serial.read());
    }

    bool needsRedraw = false;

    if (!button1Pin::read()) {
      self.envelopeShape = (uint8_t)self.envelopeShape + 1;
      //needsRedraw = true;
    }

    if (!button2Pin::read()) {
      //self.envelopePeriodTest = 1000;
      //needsRedraw = true;
    }
    
    EC11Event e;
    if (encoder1.read(&e)) {
      if (e.type == EC11Event::StepCW) {
        self.envelopePeriodTest += 8 * e.count;
      } else {
        self.envelopePeriodTest -= 8 * e.count;
      }
      needsRedraw = true;
    }

    if (needsRedraw) {
      self.draw();
    }
  }
};

void setup() {
  
  encoder1PinA::setInput(true);
  encoder1PinB::setInput(true);

  encoderButton::setInput(true);
  button1Pin::setInput(true);
  button2Pin::setInput(true);
  
  AYbox::begin();
}

void loop() {
  encoder1.checkPins(!encoder1PinA::read(), !encoder1PinB::read());
  AYbox::check();
}

