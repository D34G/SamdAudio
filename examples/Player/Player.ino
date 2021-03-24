/*
  ###########################
        WORK IN PROGRESS
  ###########################
  Audio player, non blocking.
  read 8bit mono .wav file, up to 4 channels
  use Audacity to convert your audio file

  Author : D34G
*/

//To Do: add digital potentiometer example

#include <SD.h>
#include <SPI.h>
#include <SamdAudioSD.h>
SamdAudioSD AudioPlayer;


#define NUM_AUDIO_CHANNELS 4 // Set to either 1, 2, or 4 for simultaneous playback
#define AUDIO_BUFFER_SIZE 512 // 512 works fine for 22.05kh, 32khz, and 44.1khz. not recommeneded to use any other size
#define YOUR_SD_CS 8 // SD chip select pin (with ethernet shield : 4)
#define SHUTDOWN_PIN 7 // shutdown pin for Class D amplifiers such as the PAM8302 (defaults to pin 7 in library if no explicitly set in setup)
#define DAC_PIN A0 // audio output pin on Arduino board (defaults to pin A0 in library if no explicitly set in setup)

const char *filename = "test.wav"; // your wav file, be cautious of the 8.3 filename format and place in root of your sd card
uint32_t sampleRate = 44100; //indicate sample rate here (use audacity to convert your wav)

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    delay(500);
  }

  Serial.print("Initializing SD card...");
  if (!SD.begin(YOUR_SD_CS)) // required to start communication with SD card
  {
    Serial.println(" failed!");
    return;
  }
  Serial.println(" done.");

  //SPI.setClockDivider(4); // hi-speed SPI transfers, needed if playing 88200khz sample rate (88200 currently not working, don't know why)
  AudioPlayer.selectShutdownPin(SHUTDOWN_PIN); // used to set shutdown pin (SD) on amplifier if desired and set pinmode to output, if set .play() will 
  AudioPlayer.selectDACPin(DAC_PIN); // used to set audio output pin on Arduino (defaults to pin A0)
  //AudioPlayer.setBlocking(true); // code execution waits for audio to finish (default is set to false)
  AudioPlayer.begin(sampleRate, NUM_AUDIO_CHANNELS, AUDIO_BUFFER_SIZE); // required inputs: sample rate, number of audio channels possible, size of audio buffer for processing

  Serial.print("Playing file: ");
  Serial.println(filename);
  AudioPlayer.play(filename); // plays audio on channel one only
  
  Serial.println("Send any command to continue...");
}

void loop()
{
  if (Serial.available()) {
    char c = Serial.read();

    if ( c == 'o') 
    {
      AudioPlayer.play("FX/FX6.wav", 0);
      Serial.println("pouwww! ch1!");
    }
    else if ( c == 'O') 
    {
      AudioPlayer.play("FX/FX6.wav", 1);
      Serial.println("pouwww! ch2!");
    }
    else if ( c == 's') 
    {
      AudioPlayer.play("TRACK/HAWKEYE.wav", 2);
      Serial.println("hawkeye ch3!");
    }
    else if ( c == 'S') 
    {
      AudioPlayer.play("TRACK/HAWKEYE.wav", 3);
      Serial.println("hawkeye ch4!");
    }
    else if ( c == 'd') 
    {
      AudioPlayer.play("TRACK/COMMANDO.wav", 0);
      Serial.println("commando ch1!");
    }
    else if ( c == 'D') 
    {
      AudioPlayer.play("TRACK/COMMANDO.wav", 1);
      Serial.println("commando ch2!");
    }
    else if ( c == 'm') 
    {
      AudioPlayer.play("TRACK/MARIACHI.wav", 2);
      Serial.println("mariachi ch3!");
    }
    else if ( c == 'M') 
    {
      AudioPlayer.play("TRACK/MARIACHI.wav", 3);
      Serial.println("mariachi ch4!");
    }
    else if ( c == 'l') 
    {
      AudioPlayer.loopChannel(0, true);
      Serial.println("start looping ch1!");
    }
    else if ( c == 'L') 
    {
      AudioPlayer.loopChannel(0, false);
      Serial.println("stop looping ch1!");
    }
    else if ( c == 'v') 
    {
      AudioPlayer.gainUp();
      Serial.println("increase gain!");
    }
    else if ( c == 'V') 
    {
      AudioPlayer.gainDown();
      Serial.println("decrease gain!");
    }
    else if ( c == '0') 
    {
      AudioPlayer.stopChannel(0);
      //AudioPlayer.setShutdownPinState(LOW);
      Serial.println("ch1 off!");
    }
    else if ( c == '1') 
    {
      AudioPlayer.stopChannel(1);
      Serial.println("ch2 off!");
    }
    else if ( c == '2') 
    {
      AudioPlayer.stopChannel(2);
      Serial.println("ch3 off!");
    }
    else if ( c == '3') 
    {
      AudioPlayer.stopChannel(3);
      Serial.println("ch4 off!");
    }
    else if ( c == 'X') 
    {
      AudioPlayer.end();
      Serial.println("stop audio player!");
    }
    else if ( c == 'Y') 
    {
      AudioPlayer.setShutdownPinState(LOW, true); // sets SD output pin LOW, to deactivate amplifier and set mute flag to prevent the SD pin going high during playback
      Serial.println("mute!");
    }
    else if ( c == 'y') 
    {
      AudioPlayer.setShutdownPinState(HIGH, false); // sets SD output pin HIGH, to activate amplifier and remove mute flag for the SD pin
      Serial.println("unmute!");
    }
  }
}
