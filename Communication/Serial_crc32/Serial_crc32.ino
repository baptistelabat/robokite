#include <avr/pgmspace.h>
// This file gives an exemple of serial with verification of error with crc32
String inputString = "";   // A string to hold incoming data
String checksum_received = "";
String msg = "";
boolean stringComplete = false;  // Whether the string is complete
const uint32_t PROGMEM crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};

unsigned long crc_update(unsigned long crc, byte data)
{
    byte tbl_idx;
    tbl_idx = crc ^ (data >> (0 * 4));
    crc = pgm_read_dword_near(crc_table + (tbl_idx & 0x0f)) ^ (crc >> 4);
    tbl_idx = crc ^ (data >> (1 * 4));
    crc = pgm_read_dword_near(crc_table + (tbl_idx & 0x0f)) ^ (crc >> 4);
    return crc;
}

unsigned long crc_string(char *s)
{
  unsigned long crc = ~0L;
  while (*s)
    crc = crc_update(crc, *s++);
  crc = ~crc;
  
  return crc;
}

void setup()
{
  Serial.begin(9600);
}

void loop()
{
    inputString="";
  Serial.flush();
  while (Serial.available()) {
    // Get the new byte:
    char inChar = (char)Serial.read();
    // Add it to the inputString:
    inputString += inChar;
    // If the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
  String checkedString = remove_crc32(inputString);
  if (checkedString!="")
  {
    Serial.println(checkedString);
  }
  delay(1000);
}
  
  String remove_crc32(String inputString)
  {
  String msg = inputString.substring(0, inputString.length()-4);
  char *cmsg = &msg[0];
  unsigned long checksum = crc_string(cmsg);
  String checksum_received = inputString.substring(inputString.length()-4);
  char *cchecksum = &checksum_received[0];
  unsigned char byteArray[4];
  // convert from an unsigned long int to a 4-byte array
  byteArray[0] = (int)((checksum>> 24) & 0xFF) ;
  byteArray[1] = (int)((checksum >> 16) & 0xFF) ;
  byteArray[2] = (int)((checksum >> 8) & 0XFF);
  byteArray[3] = (int)((checksum & 0XFF));
  boolean checksum_valid=((unsigned char)cchecksum[0]==(unsigned char)byteArray[0])&
  ((unsigned char)cchecksum[1]==(unsigned char)byteArray[1])&
  ((unsigned char)cchecksum[2]==(unsigned char)byteArray[2])&
  ((unsigned char)cchecksum[3]==(unsigned char)byteArray[3]);
  if (checksum_valid)
  { 
    return msg;
  }
  else
  {
    return "";
  }
}
