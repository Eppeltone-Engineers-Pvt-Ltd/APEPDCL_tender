#ifndef _AES_H_
#define _AES_H_

#include "r_stdint.h"
#include "..\\..\\r_dlms_typedef.h"

#ifndef ECB
  #define ECB 1
#endif

#define AES_KEY128_LEN		(16)		/* 128 bits (16 bytes) */
#define AES_KEY192_LEN		(24)		/* 192 bits (24 bytes) */
#define AES_KEY256_LEN		(32)		/* 256 bits (32 bytes) */
#define AES_EKEY128_LEN		(44)		/* 176 bytes (44*4(uint32_t)) */
#define AES_EKEY192_LEN		(52)		/* 208 bytes (52*4(uint32_t)) */
#define AES_EKEY256_LEN		(60)		/* 240 bytes (60*4(uint32_t)) */
#define AES_BLOCK_SIZE		(16)		/* 128 bits (16 bytes) */
#define AES_IVEC_SIZE		(16)		/* 128 bits (16 bytes) */

#if defined(ECB) && ECB

void AES128_ECB_encrypt(uint8_t* input, const uint8_t* key, uint8_t *output);
void AES128_ECB_decrypt(uint8_t* input, const uint8_t* key, uint8_t *output);

#endif // #if defined(ECB) && ECB





#endif //_AES_H_
