/*
  ###########################
        WORK IN PROGRESS
  ###########################
  Audio player, non blocking.
  read 8bit mono .wav file, up to 4 channels
  use Audacity to convert your audio file

  Author : D34G
*/

#include <SD.h>
#include <SPI.h>
#include <SamdAudioSD.h>


//-----------------------------------Global Variables-----------------------------------
#define NUM_AUDIO_CHANNELS 4 // Set to either 1, 2, or 4 for simultaneous playback
#define AUDIO_BUFFER_SIZE 512 // 512 works fine for 22.05kh, 32khz, and 44.1khz (not recommeneded to use any other size)
#define YOUR_SD_CS 8 // SD chip select pin 
#define SHUTDOWN_PIN 7 // shutdown pin for Class D amplifiers such as the PAM8302 (defaults to pin 7 in library if no explicitly set in setup)
#define DAC_PIN A0 // audio output pin on Arduino board (defaults to pin A0 in library if no explicitly set in setup)
#define DIGITAL_POT 5 // digital potentiometer pin to control amplifier hardware gain (defaults to pin 5 in library if no explicitly set in setup)

SamdAudioSD AudioPlayer;
const char *filename = "test.wav"; // your wav file, be cautious of the 8.3 filename format and place in root of your sd card
uint32_t sampleRate = 44100; //indicate sample rate here (use audacity to convert your wav)


//-----------------------------------Functions-----------------------------------
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
  AudioPlayer.selectShutdownPin(SHUTDOWN_PIN); // used to set shutdown pin (SD) on amplifier if desired and set pinmode to output, if set .play() will power the amplifier up and down as needed
  AudioPlayer.selectDACPin(DAC_PIN); // used to set audio output pin on Arduino (defaults to pin A0)
  AudioPlayer.selectDigitalPotPin(DIGITAL_POT); // used to set the digital potentiometer pin on the arduino in order to control amplifier volume (note, if using other SPI devices all pins will have to be set high first)
  AudioPlayer.setVolume(100); // set the volume anywhere from 0 to 100 (note, if using other SPI devices all pins will have to be set high first)
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
      AudioPlayer.play("test1.wav", 0);  // plays audio on channel 1
      Serial.println("playing on ch1!");
    }
    else if ( c == 'O') 
    {
      AudioPlayer.play("test2.wav", 1);  // plays audio on channel 2
      Serial.println("playing on ch2!");
    }
    else if ( c == 's') 
    {
      AudioPlayer.play("test3.wav", 2);  // plays audio on channel 3
      Serial.println("playing on ch3!");
    }
    else if ( c == 'S') 
    {
      AudioPlayer.play("test4.wav", 3);  // plays audio on channel 4
      Serial.println("playing on ch4!");
    }
    else if ( c == 'l') 
    {
      AudioPlayer.loopChannel(0, true);  // sets flag to loop channel one as true to loop until flag is set false
      Serial.println("start looping ch1!"); 
    }
    else if ( c == 'L') 
    {
      AudioPlayer.loopChannel(0, false);  // removes flag to loop channel one to false to stop looping once current playback completes
      Serial.println("stop looping ch1!");
    }
    else if ( c == 'g') 
    {
      AudioPlayer.gainUp();  // a software control which increases volume during audio processing 
      Serial.println("increase gain!");
    }
    else if ( c == 'G') 
    {
      AudioPlayer.gainDown();   // a software control which decreases volume during audio processing 
      Serial.println("decrease gain!");
    }
    else if ( c == 'v') 
    {
      AudioPlayer.volumeUp();   // a hardware control which increases amplifier volume through use of a digital potentiometer like the MCP4151 or MCP 4152 (note, if using other SPI devices all pins will have to be set high first)
      Serial.println("increase volume!");
    }
    else if ( c == 'V') 
    {
      AudioPlayer.volumeDown();  // a hardware control which decreases amplifier volume through use of a digital potentiometer like the MCP4151 or MCP 4152 (note, if using other SPI devices all pins will have to be set high first)
      Serial.println("decrease volume!");
    }
    else if ( c == '0') 
    {
      AudioPlayer.stopChannel(0);  // stops playback on channel 1
      Serial.println("ch1 off!");
    }
    else if ( c == '1') 
    {
      AudioPlayer.stopChannel(1);  // stops playback on channel 2
      Serial.println("ch2 off!");
    }
    else if ( c == '2') 
    {
      AudioPlayer.stopChannel(2);  // stops playback on channel 3
      Serial.println("ch3 off!");
    }
    else if ( c == '3') 
    {
      AudioPlayer.stopChannel(3);  // stops playback on channel 4
      Serial.println("ch4 off!");
    }
    else if ( c == 'X') 
    {
      AudioPlayer.end();  // terminates audioplayer session
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
