#include <SPI.h>
#include <EEPROM.h>

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

#define recordMemStart 8*18*4
#define modOffset controlValues[26]
#define modScale controlValues[27]
#define modFrequency controlValues[1]

__attribute__((optimize("O0")))

const PROGMEM float sine_table [101] = {  // values of sine from 0 - pi such that sine_table[0] = 0, sine_table[50] = 1 and sine_table[100] = 0
       0.0       , 0.03141076, 0.06279052, 0.09410831, 0.12533323,
       0.15643447, 0.18738131, 0.21814324, 0.24868989, 0.27899111,
       0.30901699, 0.33873792, 0.36812455, 0.39714789, 0.42577929,
       0.4539905 , 0.48175367, 0.50904142, 0.53582679, 0.56208338,
       0.58778525, 0.61290705, 0.63742399, 0.66131187, 0.68454711,
       0.70710678, 0.72896863, 0.75011107, 0.77051324, 0.79015501,
       0.80901699, 0.82708057, 0.84432793, 0.86074203, 0.87630668,
       0.89100652, 0.90482705, 0.91775463, 0.92977649, 0.94088077,
       0.95105652, 0.96029369, 0.96858316, 0.97591676, 0.98228725,
       0.98768834, 0.9921147 , 0.99556196, 0.99802673, 0.99950656,
       1.0       , 0.99950656, 0.99802673, 0.99556196, 0.9921147 ,
       0.98768834, 0.98228725, 0.97591676, 0.96858316, 0.96029369,
       0.95105652, 0.94088077, 0.92977649, 0.91775463, 0.90482705,
       0.89100652, 0.87630668, 0.86074203, 0.84432793, 0.82708057,
       0.80901699, 0.79015501, 0.77051324, 0.75011107, 0.72896863,
       0.70710678, 0.68454711, 0.66131187, 0.63742399, 0.61290705,
       0.58778525, 0.56208338, 0.53582679, 0.50904142, 0.48175367,
       0.4539905 , 0.42577929, 0.39714789, 0.36812455, 0.33873792,
       0.30901699, 0.27899111, 0.24868989, 0.21814324, 0.18738131,
       0.15643447, 0.12533323, 0.09410831, 0.06279052, 0.03141076, 0.0};

const int DACoutValues [18] = {20, 40, 80, 160, 320, 640, 750, 890, 950, 1056, 1140, 1280, 2000, 2560, 3000, 3500, 3800, 4095};
//float freqTuning [4][18];
float maxFreqs[8];

// pins used for the connection with the sensor
// the other you need are controlled by the SPI library):
const int freqChipPin0 = 6;
const int freqChipPin1 = 5;
const int freqChipPin2 = 4;
const int freqChipPin3 = 3;
const int ampChipPin = 2;
const int LED = 13;

//bool interruptTest;
bool playing = false;
bool recording = false;
int n = 0;
int maxN = 0;
long loopStartTime;
int loopLength;
byte lastbpm;

int nextCommand;
bool newData = false;
int newCommand = -1;
int newData1 = -1;
int newData2 = -1;

int activeNotes [8];
byte activeVelocities [8];
byte notePressed; // this is actually 8 boolean values stored as a single byte (MSb is for osc 7, LSb for osc 0)
byte decayVelocities [8];
long lastUpdated [8];
long releaseTimes [8];
byte lastVelocities [8];
float freqs [8];

byte controlValues [128];
byte aftertouch = 0;
byte pitchbend = 64;

float modPhase = 0;  // the phase of the mod wheel oscillation. Doesn't need to stay within 2*pi, and is updated by the interrupt
float modAmp = 0;  // amplitude of the mod wheel oscillation, updated by the interrupt but the oscillator amplitudes are controlled in the main loop

void setup() {
  modOffset = 127;
  modScale = 0;
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  //cli();
  //set timer2 interrupt at 2kHz
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = 249;// = (16*10^6) / (2000*32) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 and CS20 bits for 32 prescaler
  TCCR2B |= (1 << CS21);
  TCCR2B |= (1 << CS20);
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
  


  Serial.begin(31250);
  SPI.begin();

  //pinMode(12, OUTPUT); //test
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(ampChipPin, OUTPUT);
  digitalWrite(ampChipPin, HIGH);
  pinMode(freqChipPin0, OUTPUT);
  digitalWrite(freqChipPin0, HIGH);
  pinMode(freqChipPin1, OUTPUT);
  digitalWrite(freqChipPin1, HIGH);
  pinMode(freqChipPin2, OUTPUT);
  digitalWrite(freqChipPin2, HIGH);
  pinMode(freqChipPin3, OUTPUT);
  digitalWrite(freqChipPin3, HIGH);
  findMaxFreqs();
  delay(100);
  for (int i = 0; i < 8; i++) { //Set all outputs high which lowers noise
    writeOscillatorAmp(i, 255);
    writeOscillatorFreq(i, 0);
    lastUpdated[i] = micros();
    activeNotes[i] = -1;
  }
  //autotune();
  //Serial.println("Finished autotuning");
  sei();//allow interrupts
}

void loop() {
  if (recording) {
    byte bpm = controlValues[16]+50; // controlled by E1 on patch 2
    float beats = (micros()*(bpm/float(60e6)));
    if ((beats - floor(beats)) > 0.5) {
      digitalWrite(LED_BUILTIN, HIGH);
    } else {
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
  if (!newData && playing) { // can insert recorded data
    byte bpm = controlValues[16]+50; // controlled by E1 on patch 2
    if (bpm != lastbpm) {
      // adjust loopStartTime so playback doesn't skip
      lastbpm = bpm;
    }
    int currentTime = (micros() - loopStartTime)*bpm/100000.0; // a time step in milliseconds at 100 bpm
    if (currentTime >= (loopLength*600)) {
      loopStartTime = micros();
      currentTime = 0;
      n = 0;
    }

    if (n < maxN) {
      int address = recordMemStart + n*4;
      int pressTime;
      EEPROM.get(address, pressTime);
      if (pressTime <= currentTime) {
        newCommand = 144;
        newData = true;
        newData1 = EEPROM.read(address + 2);
        newData2 = EEPROM.read(address + 3);
        n++;
      }
    }
  }
  checkMIDI();
  if (newData) {
    if (newCommand == 144) { // note on/off
      changeNote();
    } else if (newCommand == 192) {// change patch, not really useful, seems to not have any data either when I spotted it
      newData = false;
    } else if (newCommand == 153) {// special notes (the big pads)
      changeSpecialNote();
    } else if (newCommand == 191) {// playback buttons
      playbackControl();
    } else if (newCommand == 176) {// control values
      if (newData2 != -1) {
        newData = false;
        controlValues[newData1] = newData2;
        Serial.print("Control value:\t");
        Serial.print(newData1);
        Serial.print("\t");
        Serial.println(newData2);
      }
    } else if (newCommand == 208) {// aftertouch (channel wide, not note specific)
      if (newData1 != -1) {
        newData = false;
        aftertouch = newData1;
        Serial.print("Aftertouch:\t");
        Serial.println(aftertouch);
      }
    } else if (newCommand == 224) {// pitchbend
      if (newData2 != -1) {
        newData = false;
        pitchbend = newData2; //newData1 is pretty much unused here, just displays 127 when pitch wheel is at max?
        Serial.print("Pitchbend:\t");
        Serial.println(pitchbend);
      }
    } else {
      Serial.print("Unknown command:\t");
      Serial.println(newCommand);
      newData = false;
    }
    //    printCommand();

  }
  //          Serial.print(activeNotes[0]);
  //          Serial.print("\t");
  //          Serial.print(activeVelocities[0]);
  //          Serial.print("\t");
  //          Serial.print(decayVelocities[0]);
  //          Serial.print("\t");
  //          Serial.println(releaseTimes[0]);



  long currentMicros = micros();
  long attack1 = controlValues[74]; // patch one knobs
  long decay1 = controlValues[71];
  byte sustain1 = controlValues[91];
  long release1 = controlValues[93];
  //   long attack2 = controlValues[73];
  //   long decay2 = controlValues[72];
  //   byte sustain2 = controlValues[5];
  //   long release2 = controlValues[84];

  // times in us
  long attackTime1 = 310 * attack1 * attack1;
  long decayTime1 = 310 * decay1 * decay1;
  long releaseTime1 = 310 * release1 * release1;
  //   long attackTime2 = 310*attack2*attack2;
  //   long decayTime2 = 310*decay2*decay2;
  //   long releaseTime2 = 310*release2*release2;

  for (int i = 0; i < 8; i++) {
    if (activeNotes[i] != -1) {
      // Frequency correcting to pitch wheel (maybe fm synthesis):
      float freq = 440 * (pow(2, (float(activeNotes[i]) + ((pitchbend - 64.0) / 32) - 69.0) / 12.0));
      if (abs((freq - freqs[i]) / freq) > 0.004) {  // if significant change has occurred
        //          Serial.print("Updating freq of note ");
        //          Serial.print(i);
        //          Serial.print(" to ");
        //          Serial.println(freq);
        //          delayMicroseconds(100); // test
        int DACOUT = tuningMap(i, freq);
        writeOscillatorFreq(i, DACOUT);
        freqs[i] = freq;
      }

      long delayedTime = currentMicros - lastUpdated[i];

      byte velocity = 0;

      if (delayedTime < attackTime1) { // in attack period (want this to execute even if the note has been released)
        velocity = activeVelocities[i] * float(delayedTime) / attackTime1;
      }
      else if ((notePressed & (1 << i)) != 0) { //note is pressed
        if (delayedTime < (attackTime1 + decayTime1)) {// in decay period
          float tempSustain = sustain1;
          if (activeVelocities[i] < sustain1) {
            tempSustain = activeVelocities[i];
          }
          velocity = activeVelocities[i] - (activeVelocities[i] - tempSustain) * (float(delayedTime - attackTime1) / decayTime1);
        } else {// in sustain period
          velocity = sustain1;
          if (activeVelocities[i] < sustain1) {
            velocity = activeVelocities[i];
          }
        }
      } else  { //not pressing the note anymore
        if (decayVelocities[i] == 0) { // note has just been let go
          decayVelocities[i] = lastVelocities[i];
          releaseTimes[i] = currentMicros; // without this line the decay is calculated based on the release time of the note even if the note was released long before attack time was finished
        }
        long releasedTimeDelay = currentMicros - releaseTimes[i];
        if (releasedTimeDelay < releaseTime1) { // still releasing
          velocity = decayVelocities[i] * float(releaseTime1 - releasedTimeDelay) / releaseTime1;
        }
        else { // note should be recycled
          decayVelocities[i] = 0;
          velocity = 127;
          activeNotes[i] = -1;
          writeOscillatorAmp(i, 255); // consider removing this tbh
          writeOscillatorFreq(i, 0);
          freqs[i] = 0;
        }
      }
      velocity *= 2;
      float tmp = float(velocity) * ((float(modOffset) + modAmp*float(modScale))/127.0);
      velocity = byte(max(min(tmp, 255), 0));
      if (abs(velocity - lastVelocities[i]) > 2) { // if significant change:
        writeOscillatorAmp(i, amplitudeMapping(i, velocity));
        lastVelocities[i] = velocity;
      }
    }
  }

}

void playbackControl() {
  if (newData2 != -1) {
    newData = false;
    Serial.print("Playback button\t");
    Serial.print(newData1);
    Serial.print("\t");
    if (newData2 == 127) {
      Serial.println("on");
      if (newData1 == 117) { // play
        playing = !recording;
        if (playing) {
          loopStartTime = micros();
          n = 0;
        }
      } else if (newData1 == 116) { // stop
        playing = false;
      } else if (newData1 == 118) { // record
        if (!recording) {
          playing = false;
          n = 0;
          recording = true;
          Serial.println("Started recording");
        }
        else {
          if (n != 0) {
            byte bpm = controlValues[16]+50; // controlled by E1 on patch 2
            loopLength = ceil((micros() - loopStartTime)*bpm/float(60e6));
            maxN = n;
            Serial.print("Finished recording. Used ");
            Serial.print(maxN);
            Serial.print(" notes, over ");
            Serial.print(loopLength);
            Serial.print(" beats at a BPM of ");
            Serial.println(bpm);
          }
          recording = false;
        }
      } else if (newData1 == 113) { // loop button
          autotune();
      }
    }
    else {
      Serial.println("off");
    }
  }
}

void checkMIDI() {
  if (Serial.available()) { // if new byte(s) received
    if (newData == true) { // last recieved message hasn't been fully processed. Probably because of missing byte or two
      byte data = Serial.read();
      if (newData1 == -1 && data < 128) {
        newData1 = data;
        if (Serial.available()) { //still more bytes, check if this is for the second byte or a new command:
          data = Serial.read();
          if (newData2 == -1 && data < 128) {
            newData2 = data;
            return;
          }
          else if (data >= 128) {
            nextCommand = data;
            return;
          }
        }
      }
      else if (newData2 == -1 && data < 128) { // for the second byte. This command is now definitely finished.
        newData2 = data;
        return;
      }
    } else { //old command processed, can delete and refill with new command
      newCommand = -1;
      newData1 = -1;
      newData2 = -1;
      newData = true;
      if (nextCommand != -1) { // a previous command had been read but left alone
        newCommand = nextCommand;
        nextCommand = -1;
        newData1 = -1;
        newData2 = -1;
      }
      byte data = Serial.read();
      if (newData1 == -1 && data < 128) {
        newData1 = data;
        if (Serial.available()) { //still more bytes, check if this is for the second byte or a new command:
          data = Serial.read();
          if (newData2 == -1 && data < 128) {
            newData2 = data;
            return;
          }
          else if (data >= 128) {
            nextCommand = data;
            return;
          }
        }
      }
      else if (newData2 == -1 && data < 128) { // for the second byte. This command is now definitely finished.
        newData2 = data;
        return;
      }
      else if (data >= 128) {
        newCommand = data;
        if (Serial.available()) {
          data = Serial.read();
          if (newData1 == -1 && data < 128) {
            newData1 = data;
            if (Serial.available()) { //still more bytes, check if this is for the second byte or a new command:
              data = Serial.read();
              if (newData2 == -1 && data < 128) {
                newData2 = data;
                return;
              }
              else if (data >= 128) {
                nextCommand = data;
                return;
              }
            }
          }
          else if (newData2 == -1 && data < 128) { // for the second byte. This command is now definitely finished.
            newData2 = data;
            return;
          }
        }
      }

      //    Serial.print(activeNote, BIN);
      //    Serial.print(" ");
      //    Serial.println(activeVelocity, BIN);
    }
  }
}

void changeSpecialNote() {
  newData = false;
  if (newData2 > 0) { // New note has been pressed
    Serial.print("Special note:\t");
    // Serial.print("\t");
    Serial.print(newData1);
    Serial.print("\t");
    Serial.println(newData2);
  }
  else if (newData2 == 0) { // Note has been released
    Serial.print("Special note:\t");
    Serial.print(newData1);
    Serial.println("\treleased");
  }
  else { // MIDI command not yet fully received
    newData = true; // This will keep running this function in every loop until the third byte has been processed by checkMIDI();
  }
}

void changeNote() {
  newData = false;
  if (newData2 > 0) { // New note has been pressed
    Serial.print("Note:\t");
    // Serial.print("\t");
    Serial.print(newData1);
    Serial.print("\t");
    Serial.println(newData2);
    float freq = 440 * (pow(2, (float(newData1) - 69.0) / 12.0));
    int oscillatorNum = -1;
    bool replacedOsc = false;
    for (int i = 0; i < 8; i++) { //Check if oscillator with no use exists or one already in release of the same note
      if ((activeNotes[i] == -1 && maxFreqs[i] >= freq) || activeNotes[i] == newData1) {
        oscillatorNum = i;
        break;
      }
    }
    if (oscillatorNum == -1) { // haven't found an oscillator yet
      long oldestUpdate = 1e10;
      for (int i = 0; i < 8; i++) { //Check if oscillator which is in release time exists, and pick the oldest if so
        if (activeVelocities[i] == 0  && maxFreqs[i] >= freq) {
          if (releaseTimes[i] < oldestUpdate) {
            replacedOsc = true;
            oldestUpdate = releaseTimes[i];
            oscillatorNum = i;
          }
        }
      }
    }
    if (oscillatorNum == -1) { // still haven't found an oscillator
      long oldestUpdate = 1e10;
      for (int i = 0; i < 8; i++) { //Choose oldest oscillator
        if (lastUpdated[i] < oldestUpdate  && maxFreqs[i] >= freq) {
          replacedOsc = true;
          oldestUpdate = lastUpdated[i];
          oscillatorNum = i;
        }
      }
    }
    if (replacedOsc) {// oscillator in use has been replaced, if possible copy all data onto an unused oscillator (which couldn't initially have been used as it was outside the frequency range of needed note)
      int replacedNote = -1;
      freq = 440 * (pow(2, (float(activeNotes[oscillatorNum]) - 69.0) / 12.0));
      for (int i = 0; i < 8; i++) { //Check if oscillator with no use exists or one already in release of the same note
        if ((activeNotes[i] == -1 && maxFreqs[i] >= freq)) {
          replacedNote = i;
          break;
        }
      }
      if (replacedNote != -1) { // found suitable replacement which wasn't being used
        activeVelocities[replacedNote] = activeVelocities[oscillatorNum];
        lastUpdated[replacedNote] = lastUpdated[oscillatorNum];
        activeNotes[replacedNote] = activeNotes[oscillatorNum];
        if ((notePressed & 1 << oscillatorNum) > 0) {
          notePressed = notePressed | 1 << replacedNote;
        }
      }
    }
    if (oscillatorNum != -1) {
      lastUpdated[oscillatorNum] = micros();
      activeVelocities[oscillatorNum] = controlValues[7] + float(1.0 - controlValues[7] / 127.0) * newData2;
      activeNotes[oscillatorNum] = newData1;
      notePressed = notePressed | 1 << oscillatorNum;
    }
  }
  else if (newData2 == 0) { // Note has been released
    Serial.print("Note:\t");
    Serial.print(newData1);
    Serial.println("\treleased");
    int oscillatorNum = -1;
    for (int i = 0; i < 8; i++) { //Find oscillator which was being used (but it may have been overwritten)
      if (activeNotes[i] == newData1) {
        oscillatorNum = i;
        break;
      }
    }
    if (oscillatorNum != -1) {
      //activeVelocities[oscillatorNum] *= -1;
      notePressed = notePressed - (1 << oscillatorNum);
      //writeOscillatorFreq(oscillatorNum,0);
      //writeOscillatorAmp(oscillatorNum,255);
      releaseTimes[oscillatorNum] = micros();
    }
  }
  else { // MIDI command not yet fully received
    newData = true; // This will keep running this function in every loop until the third byte has been processed by checkMIDI();
  }

  if (recording && newData2 >= 0) {
    if (n == 0) {
      loopStartTime = micros();
    }
    float bpm = controlValues[16]+50; // controlled by E1 on patch 2
    int currentTime = (micros() - loopStartTime)*bpm/100000.0; // a time step in milliseconds at 100 bpm
    int address = recordMemStart + n*4;
    if (address < 1024) {  // otherwise address will overflow and corrupt the start of the memory
      EEPROM.put(address, currentTime);
      EEPROM.write(address + 2, newData1);
      EEPROM.write(address + 3, newData2);
      n++;
    }
  }
}

void autotune() {
  Serial.println("Autotuning:");
  for (int i = 0; i < 8; i++) { //Set all outputs high which lowers noise
    writeOscillatorAmp(i, 255);
    writeOscillatorFreq(i, 0);
    lastUpdated[i] = micros();
    activeNotes[i] = -1;
  }
  for (int i = 0; i < 8; i++) {
    Serial.print("Oscillator ");
    Serial.println(i);
    for (int j = 0; j < 18; j++) {
      writeOscillatorFreq(i, DACoutValues[j]);
      delay(50);
      int periods = 0;
      long lastMicros = micros();
      bool isHigh = true;
      bool isSound = true;
      while (true) {
        int analogValue = analogRead(0);
        if (isHigh && analogValue < 400) { // Falling edge
          isHigh = false;
        }
        if (!isHigh && analogValue > 600) { // First rising edge
          isHigh = true;
          break;
        }
        if (micros() - lastMicros > 2000000) { //two seconds have passed, give up and assume no waveform
          isSound = false;
          break;
        }
      }
      float freq = 0;
      if (isSound) {
        bool shouldTerminate = false;
        lastMicros = micros();
        long timeOut = 0;
        int maxCycles = 500;
        int cycles = 0;
        while (cycles < maxCycles) {
          int analogValue = analogRead(0);
          if (isHigh && analogValue < 400) { //Falling edge
            isHigh = false;
          }
          if (!isHigh && analogValue > 600) { //Rising edge
            isHigh = true;
            cycles++;
            if (shouldTerminate) {
              break;
            }
          }
          if (!shouldTerminate && (micros() - lastMicros) > 3000000) { //two seconds have passed, for low frequencies this may not have reached 100 cycles, but accuracy is still acceptable
            shouldTerminate = true;
            timeOut = micros();
          }
          if (shouldTerminate && ((micros() - timeOut) > 500000)) { //if it somehow found a peak the first time it might get stuck here so make sure that it always exits after 1.5s max
            cycles = 0;
            break;
          }
        }
        freq = float(cycles * 1000000) / float(micros() - lastMicros);
      }
      Serial.println(freq);
      EEPROM.put((i * 18 + j) * 4, freq);
    }
    writeOscillatorFreq(i, 0);
    delay(50);
  }
  findMaxFreqs();
}



void writeOscillatorAmp(byte DACnum, byte value) {

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  //digitalWrite(ampChipPin, HIGH);

  byte dataToSend = value;
  // LSb denotes the gain of amplifier, 0 for 1x and 1 for 2x (full range)
  byte DACnumMapped = ampDacMapping(DACnum);
  byte address = (DACnumMapped << 1) + 1;

  SPI.transfer(address);
  delayMicroseconds(5);
  SPI.transfer(dataToSend);


  //This isn't how typical SPI CS pins operate, but this is how the datasheet does it, and calls it a "LOAD" pin instead
  digitalWrite(ampChipPin, LOW);
  delayMicroseconds(5);
  digitalWrite(ampChipPin, HIGH);
  SPI.endTransaction();
}

void writeOscillatorFreq(byte DACnum, unsigned int value) {

  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
  int chipSelectPin = freqDACMapping(DACnum);
  DACnum = 1 - (DACnum % 2);
  digitalWrite(chipSelectPin, LOW);
  // MSb is the DAC select between A and B, then an unused bit followed by
  // a gain bit (which should be 0 for the full rail to rail range),  then
  // a shutdown bit, which should be kept high, and finally
  // the 12 bit output value:
  unsigned int dataToSend = value + DACnum * 32768 + 4096;

  // take the chip select low to select the device:


  SPI.transfer16(dataToSend); //Send register location
  //SPI.transfer(thisValue);  //Send value to record into register

  // take the chip select high to de-select:
  digitalWrite(chipSelectPin, HIGH);
  SPI.endTransaction();
}

int tuningMap(int oscillator, float freq) {
  // interpolate between values:
  int index;
  for (int i = 0; i < 13; i++) {
    if (getFreq(oscillator, i + 1) > freq) {
      index = i;
      break;
    }
  }
  int i = index;
  float dacOUT = float(DACoutValues[i]) + float(DACoutValues[i + 1] - DACoutValues[i]) * (freq - getFreq(oscillator, i)) / (getFreq(oscillator, i + 1) -  getFreq(oscillator, i));
  return round(dacOUT);
}

int freqDACMapping(byte num) {
  num = num / 2;
  if (num == 0) {
    return freqChipPin0;
  } else if (num == 1) {
    return freqChipPin1;
  } else if (num == 2) {
    return freqChipPin2;
  } else {
    return freqChipPin3;
  }
}


byte ampDacMapping(byte num) {
  if (num == 0) {
    return 6;
  } else if (num == 1) {
    return 7;
  } else if (num == 2) {
    return 5;
  } else if (num == 3) {
    return 4;
  } else if (num == 4) {
    return 0;
  } else if (num == 5) {
    return 1;
  } else if (num == 6) {
    return 3;
  } else {
    return 2;
  }
}

byte amplitudeMapping(byte num, byte amplitude) { // takes oscialltor number and amplitude between 0 and 127 and returns value which can be applied to DACs
  if (num == 0) {
    return 80 + (110.0 / 255.0) * amplitude;
  } else if (num == 1) {
    return 100 + (120.0 / 255.0) * amplitude;
  } else if (num == 2) {
    return 20 + (164.0 / 255.0) * amplitude;
  } else if (num == 3) {
    return 82 + (132.0 / 255.0) * amplitude;
  } else if (num == 4) {
    return 80 + (100.0 / 255.0) * amplitude;
  } else if (num == 5) {
    return 180 + (75.0 / 255.0) * amplitude;
  } else if (num == 6) {
    return 110 + (130.0 / 255.0) * amplitude;
  } else if (num == 7) {
    return 130 + (116.0 / 255.0) * amplitude;
  }
}

float getFreq(int index1, int index2) {
  int address = (index1 * 18 + index2) * 4;
  float f;
  EEPROM.get(address, f);
  return f;
}

void findMaxFreqs() {
  for (int osc = 0; osc < 8; osc++) {
    float maximum = 0;
    for (int i = 0; i < 18; i++) {
      float value = getFreq(osc, i);
      if (value > maximum) {
        maxFreqs[osc] = value;
        maximum = value;
      }
    }
//    Serial.print("Max frequency of oscillator :\t");
//    Serial.print(osc);
//    Serial.print(" is ");
//    Serial.println(maxFreqs[osc]); 
  }
}

float fast_sine(float theta) {
  float scaled_theta = theta - floor(theta/(2*PI))*2*PI;
  // could do interpolation here but nearest value is just easier:
  if (scaled_theta > PI) {
    scaled_theta = scaled_theta - PI;
    int index = round(scaled_theta*31.831);
    return - pgm_read_float_near(sine_table + index);
  }
  else {
    int index = round(scaled_theta*31.831);
    return pgm_read_float_near(sine_table + index);
  }
}

ISR(TIMER2_COMPA_vect) { // interrupt on timer 2 (currently approximately 2 kHz I believe)
  //   //test for interrupt speed:
  //   interruptTest = !interruptTest;
  //   if (interruptTest) {
  //     digitalWrite(12,HIGH);
  //   } else {
  //     digitalWrite(12,LOW);
  //   }
    float modFreq = float(modFrequency) + 10;
    float frequency = (modFreq*modFreq)/800;
    modPhase = modPhase + frequency*0.00314159; // assuming 2 kHz update frequency
    modAmp = fast_sine(modPhase);
}
