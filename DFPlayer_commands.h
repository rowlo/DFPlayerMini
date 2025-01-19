#ifndef _DFPlayer_commands_H_
#define _DFPlayer_commands_H_

// command format: $SB VB LB CMD ACK DATA1 DATA2 CHKS1 CHKS2 $EB
#define Command_Index     3
#define Data2_Index       6

#define Start_Byte        0x7E
#define Version_Byte      0xFF
#define Command_Length    0x06
#define End_Byte          0xEF
//  Returns info with command 0x41 [0x01: info, 0x00: no info]
#define Acknowledge       0x01

//  Commands according to specification
#define Command_Play_Next_File                      0x01
#define Command_Play_Previous_File                  0x02
#define Command_Play_With_Index                     0x03 // 0x00 0x01 == 0001 == file 1
#define Command_Set_Volume                          0x06 // 0x00 0x1E == level 30
/**
  For sound level definitions see https://pressbooks.pub/sound/chapter/decibels-and-sound-levels/
  dB_000_Hearing_Threshold (0 dB),
  dB_010_Quiet_Forest (10 dB),
  dB_020_Watch_Ticking (20 dB),
  dB_030_Rice_Crispies (30 dB),
  dB_040_Library (40 dB),
  dB_050_Floor_Fan (50 dB),
  dB_060_Conversation (60 dB),
  dB_070_Noise_Inside_Car (70 dB),
  dB_080_Vaccum_Cleaner (80 dB),
  dB_090_Leaf_Blower (90 dB),
  dB_100_Chain_Saw (100 dB),
  dB_110_Machine_Shop (110 dB),
  dB_120_Loud_Indoor_Arena (120 dB),
  dB_130_Jet_Takeoff (130 dB)
*/
enum Volume_Preset: byte {
  dB_000_Hearing_Threshold = 0x00, //   0 dB
  dB_010_Quiet_Forest      = 0x0A, //  10 dB
  dB_020_Watch_Ticking     = 0x14, //  20 dB
  dB_030_Rice_Crispies     = 0x1E, //  30 dB
  dB_040_Library           = 0x28, //  40 dB
  dB_050_Floor_Fan         = 0x32, //  50 dB
  dB_060_Conversation      = 0x3C, //  60 dB
  dB_070_Noise_Inside_Car  = 0x46, //  70 dB
  dB_080_Vaccum_Cleaner    = 0x50, //  80 dB
  dB_090_Leaf_Blower       = 0x5A, //  90 dB
  dB_100_Chain_Saw         = 0x64, // 100 dB
  dB_110_Machine_Shop      = 0x6E, // 110 dB
  dB_120_Loud_Indoor_Arena = 0x78, // 120 dB
  dB_130_Jet_Takeoff       = 0x82, // 130 dB
};
#define Command_Set_Equalizer                       0x07
enum Equalizer_Preset: byte {
  Normal  = 0x00,
  Pop     = 0x01,
  Rock    = 0x02,
  Jazz    = 0x03,
  Classic = 0x04,
  Base    = 0x05
};
#define Command_Toggle_Loop_All                     0x11
enum Toggle_Loop_All_First_Byte: byte {
  All_Songs = 0x00,
};
enum Toggle_Loop_All_Or_Current_File_Second_Byte: byte {
  Stop  = 0x00,
  Start = 0x01
};
#define Command_Play_MP3                            0x12
#define Command_Play_Advertisement                  0x13
#define Command_Stop_Play                           0x16
#define Command_Loop_File_In_Folder                 0x17
#define Command_Play_All_Random                     0x18
#define Command_Toggle_Loop_Current_File            0x19
#define Command_Reset_Chip                          0x0C
#define Command_Resume                              0x0D
#define Command_Pause                               0x0E
#define Command_Current_Storage_Device              0x3F
#define Command_Successfully_Executed               0x41
#define Command_Get_Playback_Status_1               0x42
#define Command_Get_Volume_Status                   0x43
#define Command_Get_Equalizer_Status                0x44
#define Command_Get_Playback_Status                 0x45
#define Command_Get_Total_Number_Of_Files_On_SDcard 0x48
#define Command_Get_Current_File_Number             0x4C

#endif
