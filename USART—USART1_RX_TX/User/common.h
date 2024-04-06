#ifndef __common_h__
#define __common_h__

extern const unsigned char crc8Table[256];

int     byteArrayToInt(char* byteArray);
void    intToByteArray(int number, char* byteArray);
unsigned char calculateCRC8(const unsigned char* data, int length);
void copy_struct_to_array(unsigned char* rebuff, void* structure, int struct_size);
void memcpy_big_endian(void* dest, const void* src, int n) ;
unsigned char lsb_checksum(const unsigned char* data, int length) ;



#endif