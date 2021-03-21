/*
 * Copyright D34G (c) 2021
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

#ifndef SAMDAUDIOSD_H
#define SAMDAUDIOSD_H

#include "Arduino.h"
#include "Print.h"
#include <SD.h>

class SamdAudioSD
{
public:
    SamdAudioSD(){};
    void begin(uint32_t sampleRate, uint8_t numOfChannels, uint16_t audio_buffer_size);
    void play(const char *fname, uint8_t channel);
    void play(const char *fname);
    void stopChannel(uint8_t c);
    void loopChannel(uint8_t c, bool loopEnable);

    bool isPlaying(uint8_t c);
    void setGain(uint8_t v);
    void gainUp();
    void gainDown();

    void end();

    void criticalON();
    void criticalOFF();

    void selectShutdownPin(uint8_t shutdownPin);
    void setShutdownPinState(bool pinState);
    void selectDACPin(uint8_t audioOutputPin);

private:
  void dacConfigure(void);

  //The first timer is used to feed the DAC with data every 1/sampleRate sec
  void configurePlayerTimer(uint32_t sampleRate);
  bool syncPlayerTimer(void);
  void resetPlayerTimer(void);
  void enablePlayerTimer(void);
  void disablePlayerTimer(void);

  //the second timer is used to read audio data from the SD card every 15.6ms
  void configureReaderTimer();
  void enableReaderTimer();
  void disableReaderTimer();

  bool alonePlaying(uint8_t channel);
};
#endif //SAMDAUDIOSD_H