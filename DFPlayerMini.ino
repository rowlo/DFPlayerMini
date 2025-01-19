#include "SoftwareSerial.h"
#include "DFPlayer_commands.h"

SoftwareSerial mySerial(6, 7);  //  RX, TX on Uno when follwing the free AZDelivery eBook
// SoftwareSerial mySerial(51, 53);  //  RX, TX on Mega2560 when using it on the Elegoo Smart Robot Car Kit V4.0
byte receive_buffer[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
char data;
byte volume = 0x00;
bool mute_state = false;

/**
 * Sends the command to the module.
 * @param CMD The command byte according to specification. You may want to use the defines from DFPlayer_commands.h.
 * @param DATA1 The first data byte.
 * @param DATA2 The second data byte.
 */
void execute_CMD(byte CMD, byte DATA1 = 0x00, byte DATA2 = 0x00) {
  
  //  Calculate the checksum (2 bytes)
  word checksum = -(Version_Byte + Command_Length + CMD + Acknowledge + DATA1 + DATA2);
  
  //  Build the command line
  byte Command_line[10] = { Start_Byte, Version_Byte,
                            Command_Length, CMD, Acknowledge, 
                            DATA1, DATA2, highByte(checksum),
                            lowByte(checksum), End_Byte};

  //  Send the command line to the module
  for(byte k = 0; k < 10; k++) {
    mySerial.write(Command_line[k]);
  }
}

/**
 * Resets the receive_buffer.
 */
void reset_rec_buf() {
  for(uint8_t i = 0; i < 10; i++) {
    receive_buffer[i] = 0;
  }
}

/**
 * Reads the received data.
 */
bool receive() {
  reset_rec_buf();
  if(mySerial.available() < 10) {
    return false;
  }  
  for(uint8_t i = 0; i < 10; i++) {
    short b = mySerial.read();
    if(b == -1) {
      return false;
    }
    receive_buffer[i] = b;
  }
  
  // when you reset the module in software,
  // receive_buffer elements are shifted
  // to correct that we do the following:
  short b = receive_buffer[0];
  for(uint8_t i = 0; i < 10; i++) {
    if(i == 9) {
      receive_buffer[i] = b;
    }
    else {
      receive_buffer[i] = receive_buffer[i+1];
    }  
  } //  end correcting receive_buffer
  return true;
}

/**
 * Attempts to receive serial data from the module. Skips the Acknowledge responses until expected command response or nothing is received.
 * The function also calls necessary delays with the default values or provided values.
 *
 * This function is the fence for print_essential_receive_buffer!
 *
 * @param commandLabel A string describing the command. If empty, only receive() is called.
 * @param expectedCommand One of the Command_* defines in DFPlayer_commands.h. The function looks for a response with that command in the CMD field.
 * @param firstDelay The delay which is called immediately when the function is called. The default is usually fine, but after chip reset you should set 1000.
 * @param furtherDelays The delays that are called before the function returns. They ensure that following commands can receive responses. In rare cases you may need to tweak that default.
 * @return false if the receive_buffer should not be printed because no commandLabel was provided, no data was received or the expected command response was not received, 
 */
bool smart_receive(const String &commandLabel = "", byte expectCommand = 0, unsigned long firstDelay = 100, unsigned long furtherDelays = 100) {
  delay(firstDelay);

  if (commandLabel == "") {
    receive();
    delay(furtherDelays);
    return false;
  }

  uint8_t skippedResponses = 0;
  uint8_t hasReceivedCommandResponse = 0;
  for (uint8_t i = 0; i < 10; i++) {
    if (!receive()) {
      Serial.println(commandLabel + "\t" + "recieved nothing");
      delay(furtherDelays);
      return false;
    }
    // break from for if response is not Acknowledge and matches expectCommand
    bool isExpectedCommand = (expectCommand != 0 && receive_buffer[Command_Index] == expectCommand) || expectCommand == 0;
    if (receive_buffer[Command_Index] != Command_Successfully_Executed && isExpectedCommand) {
      hasReceivedCommandResponse = 1;
      if (skippedResponses == 0) {
        Serial.println("recieved something that is not Acknowledge or the expected command");
      }
      break;
    }
    skippedResponses++;
    delay(furtherDelays);
  }
  if (hasReceivedCommandResponse == 0) {
    Serial.println("recieved nothing at all in 10 retries");
    delay(furtherDelays);
    return false;
  }
  return true;
}

/**
 * If commandLabel is provided it will print received CMD DATA1 DATA2 bytes in HEX.
 * The function also calls necessary delays with the default values or provided values.
 *
 * Must only be called fenced by smart_receive()!
 *
 * @param commandLabel A string describing the command. It will be printed in the first column before the TAB.
 * @param furtherDelays The delays that are called before the function returns. They ensure that following commands can receive responses. In rare cases you may need to tweak that default.
 */
void print_essential_receive_buffer(const String &commandLabel = "", unsigned long furtherDelays = 100) {
  if (commandLabel == "") {
    return false;
  }

  Serial.print(commandLabel + "\t");
  // Outputs the returend data from the module
  // To the Serial Monitor
  for(uint8_t i = 3; i < 7; i++) { // only loop from Command index to data bytes
    if (i == 4) continue; // skip Acknowledge byte
    Serial.print(receive_buffer[i], HEX);
    Serial.print("\t");
  }
  Serial.println();
  delay(furtherDelays);
}

/**
 * Attempts to receive serial data from the module. If commandLabel is provided it will print received CMD DATA1 DATA2 bytes in HEX. Skips the Acknowledge responses until expected command response or nothing is received.
 * The function also calls necessary delays with the default values or provided values.
 *
 * @param commandLabel A string describing the command. It will be printed in the first column before the TAB.
 * @param expectedCommand One of the Command_* defines in DFPlayer_commands.h. The function looks for a response with that command in the CMD field.
 * @param firstDelay The delay which is called immediately when the function is called. The default is usually fine, but after chip reset you should set 1000.
 * @param furtherDelays The delays that are called before the function returns. They ensure that following commands can receive responses. In rare cases you may need to tweak that default.
 */
void smart_receive_and_print(const String &commandLabel = "", byte expectCommand = 0, unsigned long firstDelay = 100, unsigned long furtherDelays = 100) {
  if (smart_receive(commandLabel, expectCommand, firstDelay, furtherDelays)) {
    print_essential_receive_buffer(commandLabel, furtherDelays);
  }
}

/**
 * Resets the module and sets the EQ state, volume level, and starts the key input loop.
 */
void module_init() {
  reset_chip();
  set_eq(Equalizer_Preset::Classic);
  set_volume(Volume_Preset::dB_010_Quiet_Forest);
  loop();
}

/**
 * Resets the DFPlayer mini master module.
 */
void reset_chip() {
  execute_CMD(Command_Reset_Chip); // reset the module
  smart_receive_and_print("SDON", Command_Current_Storage_Device, 1000);
}

/**
 * Sets the state of EQ.
 * @param eq An Equalizer_Preset enum literal.
 */
void set_eq(Equalizer_Preset eq) {
  execute_CMD(Command_Set_Equalizer, 0, eq); // Sets the EQ
  smart_receive_and_print();

  execute_CMD(Command_Get_Equalizer_Status);  // Get EQ state
  smart_receive_and_print("SETEQ", Command_Get_Equalizer_Status);
}

/**
 * Sets the volume level.
 * @param volume Decibels as HEX byte. You may want to use one of the presents defined in Volume_Preset enum literal.
 */
void set_volume(uint8_t volume) {
  execute_CMD(Command_Set_Volume, 0, volume); //  Set volume level
  smart_receive_and_print();

  execute_CMD(Command_Get_Volume_Status);  //  Get volume level
  smart_receive_and_print("SETVOL", Command_Get_Volume_Status);
}

/**
 * Plays first file on the storage device.
 */
void play_first() {
  execute_CMD(Command_Play_With_Index, 0, 1); // Play first file
  smart_receive_and_print();

  execute_CMD(Command_Get_Playback_Status); // Get playback status
  smart_receive_and_print("PLYFST", Command_Get_Playback_Status, 200, 200);
}

/**
 * Resumes playing current file.
 */
void resume() {
  execute_CMD(Command_Resume);
  smart_receive_and_print();

  execute_CMD(Command_Get_Current_File_Number);  // Get the current file played
  smart_receive_and_print("RESUME", Command_Get_Current_File_Number);
}

/**
 * Pauses only current file.
 */
void pause() {
  execute_CMD(Command_Pause);
  smart_receive_and_print();

  execute_CMD(Command_Get_Current_File_Number);  // Get the current file played
  smart_receive_and_print("PAUSE", Command_Get_Current_File_Number);
}

/**
 * Plays next file, after which it stops playback.
 */
void play_next() {
  execute_CMD(Command_Play_Next_File);
  smart_receive_and_print();

  execute_CMD(Command_Get_Current_File_Number); // Get the current file played
  smart_receive_and_print("NEXT", Command_Get_Current_File_Number);
}

/**
 * Plays previous file, after which it stops playback.
 */
void play_previous() {
  execute_CMD(Command_Play_Previous_File);
  smart_receive_and_print();

  execute_CMD(Command_Get_Current_File_Number); // Get the current file played
  smart_receive_and_print("PRE", Command_Get_Current_File_Number);
}

/**
 * Toggles the mute state. On mute it reads the current volume on unmute it sets the previously read volume.
 */
void mute() {
  mute_state = !mute_state;

  if(mute_state) {
    execute_CMD(Command_Get_Volume_Status);  //  Get volume level
    smart_receive_and_print("VOL", Command_Get_Volume_Status);
    volume = receive_buffer[Data2_Index];
    
    execute_CMD(Command_Set_Volume, 0, Volume_Preset::dB_000_Hearing_Threshold); //  Set volume level
    smart_receive_and_print();

    execute_CMD(Command_Get_Volume_Status);  //  Get volume level
    smart_receive_and_print("MUTE", Command_Get_Volume_Status);
  }
  else {
    execute_CMD(Command_Set_Volume, 0, volume); //  Set previous volume level
    smart_receive_and_print();

    execute_CMD(Command_Get_Volume_Status);  //  Get volume level
    smart_receive_and_print("UNMUTE", Command_Get_Volume_Status);
  }
}

/**
 * Plays an mp3 folder file by its index.
 * @param ab Two bytes that define file number in range 1-3000 (0x0BB8).
 */
void play_mp3(uint16_t ab) {
  if ((int)ab > 3000) {
    Serial.println("Invalid value for mp3 folder file index. 1 <= ab <= 3000.");
    return;
  }
  // ab=0x1234 -> a=0x12, b=0x34
  // for details see https://stackoverflow.com/a/70407902
  uint8_t a = ab / 0x100 & 0xff;
  uint8_t b = ab & 0xff;

  execute_CMD(Command_Play_MP3, a, b);
  smart_receive_and_print();

  execute_CMD(Command_Get_Playback_Status); // Get playback status
  smart_receive_and_print("MP3PLBK", Command_Get_Playback_Status);
}

/**
 * Plays file 0001 in the mp3 folder.
 */
void play_in_mp3() {
  play_mp3(1);
}

/**
 * Plays file 01 in the 02 folder.
 */
void loop_fol_two_file_one() {
  execute_CMD(Command_Loop_File_In_Folder, 2, 1);
  smart_receive_and_print();

  execute_CMD(Command_Get_Playback_Status); // Get playback status
  smart_receive_and_print("2L1PBK", Command_Get_Playback_Status);
}

/**
 * Loops current playing file.
 */
void loop_current() {
  execute_CMD(Command_Toggle_Loop_Current_File, 0, Toggle_Loop_All_Or_Current_File_Second_Byte::Start);
  smart_receive_and_print();

  execute_CMD(Command_Get_Playback_Status); // Get playback status
  smart_receive_and_print("LCRPBK", Command_Get_Playback_Status);
}

/**
 * Stops looping the current playing file.
 */
void stop_loop_current() {
  execute_CMD(Command_Toggle_Loop_Current_File, 0, Toggle_Loop_All_Or_Current_File_Second_Byte::Stop);
  smart_receive_and_print();

  execute_CMD(Command_Get_Playback_Status); // Get playback status
  smart_receive_and_print("LCRSTP", Command_Get_Playback_Status);
}

/**
 * Loops all files, playback one by one.
 */
void loop_all() {
  execute_CMD(Command_Toggle_Loop_All, 0, Toggle_Loop_All_Or_Current_File_Second_Byte::Start);
  smart_receive_and_print();

  execute_CMD(Command_Get_Playback_Status); // Get playback status
  smart_receive_and_print("LALPBK", Command_Get_Playback_Status);
}

/**
 * Stops loop all.
 */
void stop_loop_all() {
  // Hardware bug or specification error?! Command_Toggle_Loop_All with Stop on DATA2 does not stop loop all!
  // But Command_Stop_Play does.
  // execute_CMD(Command_Toggle_Loop_All, 0, Toggle_Loop_All_Or_Current_File_Second_Byte::Stop);
  execute_CMD(Command_Stop_Play);
  smart_receive_and_print();

  execute_CMD(Command_Get_Playback_Status); // Get playback status
  smart_receive_and_print("LALSTP", Command_Get_Playback_Status);
}

/**
 * Random plays all files, loops all, repeats files in playback.
 */
void random_play() {
  execute_CMD(Command_Play_All_Random);
  smart_receive_and_print();

  execute_CMD(Command_Get_Current_File_Number);  // Get current file played
  smart_receive_and_print("RANDOM", Command_Get_Current_File_Number);
}

/**
 * Plays an ADVERT folder file by its index.
 * @param ab Two bytes that define file number in range 1-3000 (0x0BB8).
 */
void play_advertisement(uint16_t ab) {
  if ((int)ab > 3000) {
    Serial.println("Invalid value for ADVERT folder file index. 1 <= ab <= 3000.");
    return;
  }
  // ab=0x1234 -> a=0x12, b=0x34
  // for details see https://stackoverflow.com/a/70407902
  uint8_t a = ab / 0x100 & 0xff;
  uint8_t b = ab & 0xff;

  // pause the playback, plays the ad,  
  // and after ad is finished playing
  // resumes the playback
  execute_CMD(Command_Play_Advertisement, a, b);
  smart_receive_and_print("ADVERT", Command_Play_Advertisement);
}

/**
 * Plays file 0001 in the ADVERT folder.
 */
void play_in_advert() {
  play_advertisement(1);
}

/**
 * Loops all files in the folder 02.
 */
void loop_folder_two() {
  execute_CMD(Command_Loop_File_In_Folder, Toggle_Loop_All_First_Byte::All_Songs, 2);
  smart_receive_and_print();

  execute_CMD(Command_Get_Current_File_Number);  // Get current file played
  smart_receive_and_print("LPFLD2", Command_Get_Current_File_Number);
}

/**
 * Stops playback of file or loop.
 */
void stop_playback() {
  execute_CMD(Command_Stop_Play);
  smart_receive_and_print();

  execute_CMD(Command_Get_Playback_Status); // Get playback status
  smart_receive_and_print("STOP", Command_Get_Playback_Status);
}

/**
 * Query status of module. This will print playback status (0x42), volume, equalizer setting,
 * playback status (0x45), files on SD card and current file played.
 */
void query_status() {
  execute_CMD(Command_Get_Playback_Status_1); // Get status of module
  smart_receive_and_print("STATUS", Command_Get_Playback_Status_1);
  
  execute_CMD(Command_Get_Volume_Status); // Get volume level
  smart_receive_and_print("VOLUME", Command_Get_Volume_Status);
  
  execute_CMD(Command_Get_Equalizer_Status); // Get EQ status
  smart_receive_and_print("EQ", Command_Get_Equalizer_Status);
  
  execute_CMD(Command_Get_Playback_Status); // Get playback status
  smart_receive_and_print("PLYBCK", Command_Get_Playback_Status);

  execute_CMD(Command_Get_Total_Number_Of_Files_On_SDcard); // Get total number of files on storage device
  smart_receive_and_print("FILES", Command_Get_Total_Number_Of_Files_On_SDcard);

  execute_CMD(Command_Get_Current_File_Number); // Get current file played
  smart_receive_and_print("CRRTRK", Command_Get_Current_File_Number);
}

void setup() {
  Serial.begin(115200);
  mySerial.begin(9600);
  delay(1000);
  
  Serial.println("\nInitialization");
  module_init();
}

void loop() {   
  while(Serial.available() > 0) {
    data = Serial.read();
    //Serial.println(data, HEX); //  for debugging
    if(data != "/n") {
      if(data == 'P') {
        Serial.println("\Resume the file");
        resume();
      }
      else if(data == 'p') {
        Serial.println("\nPause the file");
        pause();
      }
      else if(data == 'f') {
        Serial.println("\nPlay first file");
        play_first();
      }
      else if(data == 'N') {
        Serial.println("\nPlay next file");
        play_next();
      }
      else if(data == 'R') {
        Serial.println("\nPlay previous file\t");
        play_previous();
      }
      else if(data == 'm') {
        Serial.println("\nPlay in mp3 folder file 1");
        play_in_mp3();
      }
      else if(data == 'M') {
        Serial.println("\nMute/Unmute");
        mute();
      }
      else if(data == '1') { // number one
        Serial.println("\nLoop file 01 in folder 02");
        loop_fol_two_file_one();
      }
      else if(data == 'C') {
        Serial.println("\nLoop one");
        loop_current();
      }
      else if(data == 'c') {
        Serial.println("\nStop loop one");
        stop_loop_current();
      }
      else if(data == 'L') {
        Serial.println("\nLoop all");
        loop_all();
      }
      else if(data == 'l') {
        Serial.println("\nStop loop all");
        stop_loop_all();
      }
      else if(data == 'r') {
        Serial.println("\nRandom play");
        random_play();
      }
      else if(data == 'a') {
        Serial.println("\nPlay advertisement");
        play_in_advert();
      }
      else if(data == '2') {
        Serial.println("\nLoop all files in folder 2");
        loop_folder_two();
      }
      else if(data == 'S') {
        Serial.println("\nStop playback");
        stop_playback();
      }
      else if(data == 'Q') {
        Serial.println("\nQuerry status of the module");
        query_status();
      }
      else if(data == 'i') {
        Serial.println("\nInitialize module");
        module_init();
      }
    }
  }
  delay(100);
}
