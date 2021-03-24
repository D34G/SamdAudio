/*
 * Copyright D34G (c) 2021
 * based on AudioZero library from Arduino Team
 *
 * Audio library for Arduino Nano 33 IoT.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */


 //note: so far, only single channel unsigned 8-bit .WAV files have been tested
 //      44100 and 22050 have been successfully tested as sample rates
 //      88200 has NOT been successfully tested

 //To Do: add digital potentiometer code for control of amplifier gain

#include "arduino.h"
#include "SamdAudioSD.h"


//-----------------------------------Global Variables-----------------------------------
#define MAX_N_CHANNELS         4 // maximum allowable channels to play at once
#define RAMPIN                 255 // sets size of buffer for ramp divisor
#define MAX_AUDIO_BUFFER_SIZE  512 // can set to 1024 if there is enough space in dynamic memory but really not recommended because of no successful tests using 1024

File __audioFile[MAX_N_CHANNELS]; // .wav file to be playedindex from 0 to 3
volatile bool __audioPlaying[MAX_N_CHANNELS]={false};
volatile bool __audioLooping[MAX_N_CHANNELS][2]={false}; // 0 index is looping (true) or not looping (false), 1 index is ???playing or not playing??? (never see it set to true so idk it's purpose)
volatile bool __audioFileReady[MAX_N_CHANNELS] = {false}; // file is good to go for playback
volatile uint32_t __SampleIndex[MAX_N_CHANNELS]; // audio processing sample array
uint8_t __WavSamples[MAX_N_CHANNELS][2][MAX_AUDIO_BUFFER_SIZE]; // audio processing wav sample array
uint16_t rampDivisor[MAX_N_CHANNELS]={1}; // audio processing ramp array
uint8_t whichBuffer[MAX_N_CHANNELS]={0}; // audio processing buffer array
volatile bool fillNextBuffer[MAX_N_CHANNELS]={true}; // audio processing buffer array
volatile uint16_t __audioData; // audio processing data

int __audioBufferSize = 512; // can set to 1024 if there is enough space in dynamic memory
int __numOfChannelsUsed = 4; // could be 1,2 or 4, number of channels as defined by caller
uint8_t __shutdownPin = 7;  // can be any pin, used to control a Class D amplifier like the PAM8302
uint8_t __audioOutputPin = A0; // set to DAC output pin, used to send audio data to an amplifier circuit
uint8_t __audioPlayerVolume = 4; // 0 to 3, software volume control (shouldn't need if using hardware gain control)
bool __criticalSection = false; // used in audio read handler
bool __enableBlocking = false; // prevents code execution at the end of play() until track finishes 
bool __playingAudio = false; // indicates if audio is playing when blocking is enabled
bool __shutdownPinState = LOW; // current state of shutdown pin
bool __enableShutdown = false;  // indicates that an amplifier shutdown pin is being used
bool __muteAudio = false; // override to prevent changes to SD pin during playback

void TC5_Handler (void) __attribute__ ((weak, alias("AudioPlay_Handler")));
void TC3_Handler (void) __attribute__ ((weak, alias("AudioRead_Handler")));


//-----------------------------------Functions-----------------------------------
void SamdAudioSD::begin(uint32_t sampleRate, uint8_t numOfChannels, uint16_t audio_buffer_size) // initates audio player. variables required are sample rate, number of channels to use, and audio buffer size
{
  __audioBufferSize = audio_buffer_size; // should not be set greater than 1024

  if(numOfChannels == 1 || numOfChannels == 2 || numOfChannels == 4)  // set the global variable with the channels to use
  {
    __numOfChannelsUsed = numOfChannels;
  }
  else
  {
    __numOfChannelsUsed = 4; // bad input passed, assume all 4 channels to be safe
  }

  for(uint8_t index=0; index<MAX_N_CHANNELS; index++) // set up arrays
  {
    __audioFileReady[index]=false;
    __SampleIndex[index]=0;
  }

  dacConfigure(); // modules configuration
  configurePlayerTimer(sampleRate);
  configureReaderTimer();
}

void SamdAudioSD::end() // called to manually stop the audio player
{
  disablePlayerTimer();
  resetPlayerTimer();
  disableReaderTimer();
  analogWrite(__audioOutputPin, 0);
}

void SamdAudioSD::stopChannel(uint8_t c) // called to stop a specific audio channel
{
  if(c < __numOfChannelsUsed)
  {
    __audioFileReady[c]=false;
    __audioFile[c].close();
    __SampleIndex[c]=0;
    __audioPlaying[c]=false;
  }
}

void SamdAudioSD::loopChannel(uint8_t c, bool loopEnable) // sets a specific channel to be looped until stopped
{
  __audioLooping[c][0]=loopEnable;
}

void SamdAudioSD::setGain(uint8_t v) // set initial volume, default set to 4 (shouldn't need if using hardware gain control)
{
 __audioPlayerVolume=constrain(v,0,4);
}

void SamdAudioSD::gainUp() // increase volume (shouldn't need if using hardware gain control)
{
  if(__audioPlayerVolume < 4)
  {
    __audioPlayerVolume++;
  }
}

void SamdAudioSD::gainDown() // increase volume (shouldn't need if using hardware gain control)
{
  if(__audioPlayerVolume > 0)
  {
    __audioPlayerVolume--;
  }
}

void SamdAudioSD::play(const char *fname, uint8_t channel) // to play a single track, used when multiple channels are necessary
{

  if(channel >= __numOfChannelsUsed)
  {
    return;
  }
  disableReaderTimer();
  if(__audioFileReady[channel])
  {
    __audioFile[channel].close();
  }

  __audioFile[channel] = SD.open(fname);
  if(!__audioFile[channel])
  {
    return;
  }

  whichBuffer[channel]=0;
  fillNextBuffer[channel]=1;

  __audioFile[channel].read(__WavSamples[channel][whichBuffer[channel]], __audioBufferSize);

  __SampleIndex[channel]=0;
  __audioFileReady[channel] = true;

  rampDivisor[channel]=RAMPIN;

  if (__enableBlocking) // if blocking is enabled set flag for playing audio
  {
    __playingAudio = true;
  }
  else
  {
    __playingAudio = false;
  }

  if(alonePlaying(channel)) // once the buffer is filled for the first time the counter can be started
  {
    enablePlayerTimer(); // here's where we airbend the magic smoke inside all components
  }
  __audioPlaying[channel]=true;

  enableReaderTimer(); // here's where we also airbend the magic smoke inside all components

  if (__enableBlocking) // if blocking is enabled, wait here until audio completes
  {
    while(__playingAudio)
    {
      // wait
    }
  }
}

void SamdAudioSD::play(const char *fname) // to play a single track, used when multiple channels are not necessary
{
  disableReaderTimer();
  if(__audioFileReady[0])
  {
    __audioFile[0].close();
  }

  __audioFile[0] = SD.open(fname);
  if(!__audioFile[0])
  {
    return;
  }

  whichBuffer[0]=0;
  fillNextBuffer[0]=1;

  __audioFile[0].read(__WavSamples[0][whichBuffer[0]], __audioBufferSize);

  __SampleIndex[0]=0;
  __audioFileReady[0] = true;

  rampDivisor[0]=RAMPIN;

  if (__enableBlocking) // if blocking is enabled set flag for playing audio
  {
    __playingAudio = true;
  }
  else
  {
    __playingAudio = false;
  }

  if(alonePlaying(0)) // once the buffer is filled for the first time the counter can be started
  {
    enablePlayerTimer(); // here's where we airbend the magic smoke inside all components
  }
  __audioPlaying[0]=true;

  enableReaderTimer(); // here's where we also airbend the magic smoke inside all components

  if (__enableBlocking) // if blocking is enabled, wait here until audio completes
  {
    while(__playingAudio)
    {
      // wait
    }
  }
}

bool SamdAudioSD::alonePlaying(uint8_t channel) // check to see if selected channel is the only one playing
{
  for(uint8_t index=0; index < __numOfChannelsUsed; index++)
  {
      if(index!=channel && __audioPlaying[index])
      {
        return false;
      }
  }
  return true;
}

bool SamdAudioSD::isPlaying(uint8_t c) // check to see if selected channel is playing
{
  return __audioPlaying[c%__numOfChannelsUsed];
}

bool __channelsPlaying()
{
  for(uint8_t index=0; index < __numOfChannelsUsed; index++)
  {
    if(__audioPlaying[index])
    {
      return true;
    }
  }
  return false;
}

void SamdAudioSD::dacConfigure(void) // configures the DAC to use default configuration, with output channel mode configured for event triggered conversions.
{
  analogWriteResolution(10);

  DAC->CTRLA.bit.ENABLE = 0x01;
  DAC->DATA.reg = 0;
  while (DAC->STATUS.bit.SYNCBUSY == 1);
}

void SamdAudioSD::configurePlayerTimer(uint32_t sampleRate) // configures the TC in Frequency Generation mode, with an event output once each time the audio sample frequency period expires.
{

  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)); // enable GCLK for TCC2 and TC5 (timer counter input clock)
  while (GCLK->STATUS.bit.SYNCBUSY);

  resetPlayerTimer();


  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16; // set timer counter mode to 16 bits

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ; // set TC5 mode as match frequency

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;

  TC5->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / sampleRate - 1);
  while (syncPlayerTimer());

  NVIC_DisableIRQ(TC5_IRQn); // configure interrupt request
  NVIC_ClearPendingIRQ(TC5_IRQn);
  NVIC_SetPriority(TC5_IRQn, 0);
  NVIC_EnableIRQ(TC5_IRQn);

  TC5->COUNT16.INTENSET.bit.MC0 = 1; // enable the TC5 interrupt request
  while (syncPlayerTimer());
}

bool SamdAudioSD::syncPlayerTimer()
{
  return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

void SamdAudioSD::enablePlayerTimer() // do the thing Zhu Li
{
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; // enable TC
  while (syncPlayerTimer());
}

void SamdAudioSD::resetPlayerTimer() // do the other thing Zhu Li
{
  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST; // reset TCx
  while (syncPlayerTimer());
  while (TC5->COUNT16.CTRLA.bit.SWRST);
}

void SamdAudioSD::disablePlayerTimer() // Zhu Li stop doing the thing
{
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE; // disable TC5
  while (syncPlayerTimer());
}


void SamdAudioSD::criticalON() // purpose unknown
{
  __criticalSection = true;
}

void SamdAudioSD::criticalOFF() // purpose unknown
{
  __criticalSection = false;
}


//-----------------------------------SamdAudioSD AudioPlayer-----------------------------------
void SamdAudioSD::configureReaderTimer() // we configure the magic smoke here to manipulate the .wav files
{
  // The GCLK clock provider to use
  // GCLK0, GCLK1 & GCLK3 are used already, see startup.c
  const uint8_t GCLK_SRC = 3; // NOTICE: 3 works for arduino nano 33 iot, try 0, 1, or 3 to see what works for you

  // Configure the XOSC32K to run in standby
  //SYSCTRL->XOSC32K.bit.RUNSTDBY = 1;

  // Setup clock provider GCLK_SRC with a /2 source divider
  // GCLK_GENDIV_ID(X) specifies which GCLK we are configuring
  // GCLK_GENDIV_DIV(Y) specifies the clock prescalar / divider
  // If GENCTRL.DIVSEL is set (see further below) the divider
  // is 2^(Y+1). If GENCTRL.DIVSEL is 0, the divider is simply Y
  // This register has to be written in a single operation
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(GCLK_SRC) | GCLK_GENDIV_DIV(2);
  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
  {
    /* Wait for synchronization */
  }

  // Configure the GCLK module
  // GCLK_GENCTRL_GENEN, enable the specific GCLK module
  // GCLK_GENCTRL_SRC_XOSC32K, set the source to the XOSC32K
  // GCLK_GENCTRL_ID(X), specifies which GCLK we are configuring
  // GCLK_GENCTRL_DIVSEL, specify which prescalar mode we are using
  // GCLK_RUNSTDBY, keep the GCLK running when in standby mode
  // Output from this module is 16khz (32khz / 2)
  // This register has to be written in a single operation.
  GCLK->GENCTRL.reg = GCLK_GENCTRL_GENEN |
  GCLK_GENCTRL_SRC_XOSC32K |
  GCLK_GENCTRL_ID(GCLK_SRC);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
  {
    /* Wait for synchronization */
  }

  // Turn the power to the TC3 module on
  PM->APBCMASK.reg |= PM_APBCMASK_TC3;

  // Set TC3 (shared with TCC2) GCLK source to GCLK_SRC
  // GCLK_CLKCTRL_CLKEN, enable the generic clock
  // GCLK_CLKCTRL_GEN(X), specifies the GCLK generator source
  // GCLK_CLKCTRL_ID(X), specifies which generic clock we are configuring
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |
  GCLK_CLKCTRL_GEN(GCLK_SRC) |
  GCLK_CLKCTRL_ID(GCM_TCC2_TC3);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
  {
    /* Wait for synchronization */
  }

  // Disable TC3. This is required (if enabled already)
  // before setting certain registers
  TC3->COUNT8.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC3->COUNT8.STATUS.reg & TC_STATUS_SYNCBUSY)
  {
    /* Wait for synchronization */
  }

  // Set the mode to 8 bit and set it to run in standby
  // TC_CTRLA_MODE_COUNT8, specify 8bit mode
  // TC_CTRLA_RUNSTDBY, keep the module running when in standby
  // TC_CTRLA_PRESCALER_DIVxx, set the prescalar to 64
  // Prescalar options include: DIV1, DIV2, DIV4, DIV8,
  // DIV16, DIV64, DIV256, DIV1024
  TC3->COUNT8.CTRLA.reg = TC_CTRLA_MODE_COUNT8 |
  TC_CTRLA_RUNSTDBY |
  TC_CTRLA_PRESCALER_DIV1;
  while (TC3->COUNT8.STATUS.reg & TC_STATUS_SYNCBUSY)
  {
    /* Wait for synchronization */
  }

  NVIC_DisableIRQ(TC3_IRQn); // Enable the TC3 interrupt vector
  NVIC_ClearPendingIRQ(TC3_IRQn);
  NVIC_SetPriority(TC3_IRQn, 0xFFFF); // set the priority to second (less important than feeding the DAC)
  NVIC_EnableIRQ(TC3_IRQn);

  TC3->COUNT8.INTENSET.reg = TC_INTENSET_OVF; // TC_INTENSET_OVF, enable an interrupt on overflow
  while (TC3->COUNT8.STATUS.reg & TC_STATUS_SYNCBUSY)
  {
    /* Wait for synchronization */
  }

  TC3->COUNT8.CTRLA.reg |= TC_CTRLA_ENABLE; // enable TC3
  while (TC3->COUNT8.STATUS.reg & TC_STATUS_SYNCBUSY)
  {
    /* Wait for synchronization */
  }
}

void SamdAudioSD::enableReaderTimer() // this gets the audio out the door
{
  NVIC_EnableIRQ(TC3_IRQn);
  TC3->COUNT8.CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC3->COUNT8.STATUS.reg & TC_STATUS_SYNCBUSY)
  {
    /* Wait for synchronization */
  }
}

void SamdAudioSD::disableReaderTimer() // this stops the audio
{
  TC3->COUNT8.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC3->COUNT8.STATUS.reg & TC_STATUS_SYNCBUSY)
  {
    /* Wait for synchronization */
  }
  NVIC_DisableIRQ(TC3_IRQn);
}

void SamdAudioSD::selectShutdownPin(uint8_t shutdownPin) // select the pin to control amplifiers that contain shutdown pins (SD)
{
  __shutdownPin = shutdownPin; // defaults to pin D7 if not set here
  pinMode(__shutdownPin, OUTPUT);    // sets the shutdown pin as output
  setShutdownPinState(LOW); // start with shutdown pin off
  __enableShutdown = true;
}

void SamdAudioSD::setShutdownPinState(bool pinState) // change the state of the shutdown pin of an amplifier
{
  if(pinState == HIGH)
  {
    digitalWrite(__shutdownPin, HIGH); // sets the shutdown pin on
  __shutdownPinState = HIGH;
  }
  if(pinState == LOW)
  {
    digitalWrite(__shutdownPin, LOW);  // sets the shutdown pin off
  __shutdownPinState = LOW;
  }
}

void SamdAudioSD::setShutdownPinState(bool pinState, bool muteAudio) // change the state of the shutdown pin of an amplifier
{
  if(pinState == HIGH)
  {
    digitalWrite(__shutdownPin, HIGH); // sets the shutdown pin on
  __shutdownPinState = HIGH;
  }
  if(pinState == LOW)
  {
    digitalWrite(__shutdownPin, LOW);  // sets the shutdown pin off
  __shutdownPinState = LOW;
  }
  if(muteAudio)
  {
    __muteAudio = true;
  }
  else
  {
    __muteAudio = false;
  }
}

void SamdAudioSD::selectDACPin(uint8_t audioOutputPin) // select the audio output pin on the microcontroller
{
  __audioOutputPin = audioOutputPin; // defaults to DAC pin A0 if not set here
}

void SamdAudioSD::setBlocking(bool blockFlag) // enable or disable blocking mode to prevent code execution after play() is called until audio track finishes, disabled by default
{
  __enableBlocking = blockFlag; // sets the state of the global blocking flag
}


#ifdef __cplusplus
extern "C" {
#endif

  void AudioPlay_Handler (void) // audio processing
  {
    __audioData=0;

    for(uint8_t index=0; index < __numOfChannelsUsed; index++)
    {
      if (__audioPlaying[index])
      {
        if(__audioFile[index].available())
        {
          if (__SampleIndex[index] < __audioBufferSize - 1)
          {
            __audioData+=__WavSamples[index][whichBuffer[index]][__SampleIndex[index]++]/rampDivisor[index];
          }
          else // last sample from buffer
          {
            __audioData+=__WavSamples[index][whichBuffer[index]][__SampleIndex[index]++]/rampDivisor[index];

            __SampleIndex[index] = 0;
            if(!fillNextBuffer[index]) // we have been able to load next buffer : continue; else, loop the buffer...
            {
              whichBuffer[index]=1-whichBuffer[index];
            }
            fillNextBuffer[index]=1;
          }

          if(rampDivisor[index]>1)
          {
            rampDivisor[index]--;
          }

          if (__enableShutdown && !__shutdownPinState && !__muteAudio)
          {
            digitalWrite(__shutdownPin, HIGH);  // sets the shutdown pin on
            __shutdownPinState = HIGH;
          }
        }
        else if (__audioFileReady[index]) // end of file, now play ramp out
        {
          if(__audioLooping[index][0])
          {
            __audioFile[index].seek(44); // if this block is hit, there is a 'snap' at the end of the wav file playback
            __audioFile[index].read(__WavSamples[index][1-whichBuffer[index]], __audioBufferSize);
            whichBuffer[index]=1-whichBuffer[index];
            fillNextBuffer[index]=1;
            __SampleIndex[index]=0;
          }
          else
          {
            __audioFile[index].close();
            __audioFileReady[index] = false;
            rampDivisor[index]=__WavSamples[index][whichBuffer[index]][__SampleIndex[index]]; // start ramp out from last audio sample
            __audioData+=rampDivisor[index];

            bool isLooping = false; 
            for(uint8_t channel=0; channel<__numOfChannelsUsed; channel++) // check if any channel is looping to prevent setting SD pin low
            {
              if(__audioLooping[channel][0])
              {
                isLooping = true;
                break;
              }
            }
            if(__enableShutdown && __shutdownPinState && !isLooping) // set the SD pin low if all conditions are met. this is to help prevent the "pop" from the wav playback
            {
              digitalWrite(__shutdownPin, LOW);  // sets the shutdown pin off
              __shutdownPinState = LOW;
            }
          }
        }
        else if(rampDivisor[index]>0) // ramp out finish, end of activity on the channel
        {
          __audioData+=rampDivisor[index]--;
        }
        else
        {
          __audioPlaying[index]=false;

          if(!__channelsPlaying()) // tc disable
          {
            TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
            while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);

            TC3->COUNT8.CTRLA.reg &= ~TC_CTRLA_ENABLE;
            while (TC3->COUNT8.STATUS.reg & TC_STATUS_SYNCBUSY)
            {
              /* Wait for synchronization */
            }
            if (__enableBlocking)
            {
              __playingAudio = false;
            }
          }
        }
      }
    }

    if(__channelsPlaying())
    {
      #if __numOfChannelsUsed > 4
        __audioData>>=1;
      #elif __numOfChannelsUsed <= 2
      __audioData<<=1;
      #endif

      switch (__audioPlayerVolume) {
        case 0:
          __audioData=0;
          break;
        case 1:
          __audioData>>=3;
          break;
        case 2:
          __audioData>>=2;
          break;
        case 3:
          __audioData>>=1;
          break;
        default:
          break;
      }
      DAC->DATA.reg = __audioData & 0x3FF; // DAC on 10 bits.
      while (DAC->STATUS.bit.SYNCBUSY == 1);
    }
    TC5->COUNT16.INTFLAG.bit.MC0 = 1; // clear the interrupt

  }

void AudioRead_Handler() // TC3 ISR
{
  if (TC3->COUNT8.INTFLAG.bit.OVF)
  {
    for(uint8_t index=0; index < __numOfChannelsUsed; index++)
    {
      if(__audioPlaying[index])
      {
        if(__audioLooping[index][1])
        {
          __audioFile[index].seek(44);
          __audioLooping[index][1]=false;
        }
        if(__audioFile[index].available() && !__criticalSection && fillNextBuffer[index])
        {
          __audioFile[index].read(__WavSamples[index][1-whichBuffer[index]], __audioBufferSize);
          fillNextBuffer[index]=0;
        }
      }
    }
    TC3->COUNT8.INTFLAG.bit.OVF = 1; // reset interrupt flag
  }
}

#ifdef __cplusplus
}
#endif
