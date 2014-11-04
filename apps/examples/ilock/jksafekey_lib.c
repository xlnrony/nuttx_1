/****************************************************************************
 * examples/ilock/jksafekey_lib.c
 *
 *   Copyright (C) 2011, 2013-2014 xlnrony. All rights reserved.
 *   Author: xlnrony <xlnrony@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/stat.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sched.h>
#include <string.h>
#include <ctype.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/usb/storage.h>
#include <nuttx/usb/usbhost.h>
#include "jksafekey_lib.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

/* Sanity checking */

#ifndef CONFIG_USBHOST
#  error "CONFIG_USBHOST is not defined"
#endif

#ifdef CONFIG_USBHOST_BULK_DISABLE
#  error "bulk are disabled (CONFIG_USBHOST_BULK_DISABLE)"
#endif

#ifndef CONFIG_NFILE_DESCRIPTORS
#  error "CONFIG_NFILE_DESCRIPTORS > 0 needed"
#endif

#ifndef CONFIG_USBHOST_JKSAFEKEY
#  error "CONFIG_USBHOST_JKSAFEKEY is not defined"
#endif

#define K_ENUM_FORCE( enumName )		\
		k ## enumName ## force = INT_MAX

/*
 *  �㷨����
 */
typedef enum _ALGO_TYPE {
    ALGO_DES = 0x0000,
    ALGO_3DES_2KEY_EDE = 0x0001,
    ALGO_SSF33 = 0x0004,
    ALGO_SCB2 = 0x0008,
    ALGO_RSA = 0x0010,
    ALGO_MD2 = 0x0100,
    ALGO_MD5 = 0x0200,
    ALGO_SHA1 = 0x0300,
    K_ENUM_FORCE(ALGO_TYPE)
} ALGO_TYPE;

/*
 *  �㷨ģʽ
 */
typedef enum _ALGO_MODE{
    SYMMETRIC_ECB = 0,
    SYMMETRIC_CBC = 1,
    SYMMETRIC_CFB = 2,
    SYMMETRIC_OFB = 3,
    K_ENUM_FORCE(ALGO_MODE)
}ALGO_MODE;

/*
 *  Token״̬
 */
typedef enum _TOKEN_STATUS{
    TOKEN_OFFLINE = 0,
    TOKEN_ONLINE  = 1,
    TOKEN_LOGIN   = 2,
    K_ENUM_FORCE(TOKEN_STATUS)
}TOKEN_STATUS;

/*
 *  PADDING����
 */
typedef enum _PADDING_TYPE{
    PKCS1_PADDING       = 0x0000,
    K_ENUM_FORCE(PADDING_TYPE)
}PADDING_TYPE;

/*
 *  Token��Ϣ
 */
typedef struct _PT_Token_Info {
    unsigned char nMaxSOPINLen; /*  ����ԱPIN����󳤶� */
    unsigned char nMinSOPINLen; /*  ����ԱPIN����С���� */
    unsigned char nMaxPINLen; /*  �û�PIN�ĳ���   */
    unsigned char nMinPINLen; /*  �û�PIN�ĳ���   */
    unsigned short nAlgoCap; /*  ���������������ԣ��μ�����ĺ�  */
    char szSerialNumber[16]; /*  ���кţ���������Token��Ψһ��ʶ */
    unsigned long nTotalMemory; /*  Token�ܿռ䣬�ֽ��� */
    char szLabel[32]; /* Token��ǩ */
} PT_Token_Info, *PPT_Token_Info;

/*
 *  PIN����
 */
typedef enum _PIN_TYPE{
    SO_PIN      = 0,
    USER_PIN    = 1,
    K_ENUM_FORCE(PIN_TYPE)
}PIN_TYPE;

/*
 *  RSA��Կλ��������RSA��Կʱʹ��
 */
typedef enum _PLUGTOKEN_RSA_BITS{
    PLUGRSA_512_BIT     =512,
    PLUGRSA_1024_BIT    =1024,
    PLUGRSA_2048_BIT    =2048,
    K_ENUM_FORCE(PLUGTOKEN_RSA_BITS)
}PLUGTOKEN_RSA_BITS;

/*
 *  Token���ļ�������
 */
typedef enum _TOKEN_FILE_TYPE{
    BINARY_FILE         = 0,
    DES_KEY_FILE        = 1,    
    DES_2KEY_FILE       = 2,    
    DES_3KEY_FILE       = 3,
    SSF33_KEY_FILE      = 4,
    RSA_PRV_KEY_FILE    = 5,
	SM1_KEY_FILE       = 6,
	CERT_FILE_SIGN			= 7,
	RSA_PUB_KEY_FILE_SIGN	= 8,
	CERT_FILE_KEYX		=9,
	RSA_PUB_KEY_FILE_KEYX =10,
	CONTAINER_FILE		= 11,
    K_ENUM_FORCE(TOKEN_FILE_TYPE)
}TOKEN_FILE_TYPE;

/*
 *  Token���ļ��ķ���Ȩ��
 */
typedef enum _TOKEN_FILE_PRIVILEDGE{
    READ_PRIVILEDGE_MASK    = 0x0f,
    READ_NO_PIN             = 0x00,
    READ_PIN_PROTECT        = 0x01,
    READ_NOT_ALLOWED        = 0x02,
    WRITE_PRIVILEDGE_MASK   = 0xf0,
    WRITE_NO_PIN            = 0x00,
    WRITE_PIN_PROTECT       = 0x10,
    WRITE_NOT_ALLOWED       = 0x20,
    K_ENUM_FORCE(TOKEN_FILE_PRIVILEDGE)
}TOKEN_FILE_PRIVILEDGE;

#define MD2_NAME "MD2"
#define MD2_LONG_NAME "Message Digest 2 (MD2)"
#define MD2_MIN_BITS 128
#define MD2_MAX_BITS 128
#define MD2_BITS 128
#define MD2_OID "\x30\x20\x30\x0C\x06\x08\x2A\x86\x48\x86\xF7\x0D\x02\x02\x05\x00\x04\x10"
#define MD5_NAME "MD5"
#define MD5_LONG_NAME "Message Digest 5 (MD5)"
#define MD5_BITS 128
#define MD5_MIN_BITS 128
#define MD5_MAX_BITS 128
#define MD5_OID "\x30\x20\x30\x0C\x06\x08\x2A\x86\x48\x86\xF7\x0D\x02\x05\x05\x00\x04\x10"
#define MD5_OID_LEN 18
#define SHA_NAME "SHA-1"
#define SHA_LONG_NAME "Secure Hash Algorithm (SHA-1)"
#define SHA_BITS 160
#define SHA_MIN_BITS 160
#define SHA_MAX_BITS 160
#define SHA1_OID "\x30\x21\x30\x09\x06\x05\x2b\x0E\x03\x02\x1A\x05\x00\x04\x14"  
#define SHA1_OID_LEN    15 
#define SSL3_SHAMD5_NAME "SSL3 SHAMD5"
#define SSL3_SHAMD5_LONG_NAME "SSL3 SHAMD5"
#define SSL3_SHAMD5_BITS 288
#define SSL3_SHAMD5_MIN_BITS 288
#define SSL3_SHAMD5_MAX_BITS 288
#define SSL3_SHAMD5_OID ""
#define SSL3_SHAMD5_OID_LEN    0
#define RSA_SIGN_NAME "RSA_SIGN"
#define RSA_SIGN_LONG_NAME "RSA Signature"
#define RSA_SIGN_MIN_BITS 1024
#define RSA_SIGN_BITS 1024
#define RSA_SIGN_MAX_BITS 1024
#define RSA_KEYX_NAME "RSA_KEYX"
#define RSA_KEYX_LONG_NAME "RSA Key Exchange"
#define RSA_KEYX_MIN_BITS 1024
#define RSA_KEYX_BITS 1024
#define RSA_KEYX_MAX_BITS 1024
#define DES_NAME "DES"
#define DES_LONG_NAME "Data Encryption Standard (DES)"
#define DES_MIN_BITS 56
#define DES_BITS 56
#define DES_MAX_BITS 56
#define DES3_112_NAME "3DES TWO KEY"
#define DES3_112_LONG_NAME "Two Key Triple DES"
#define DES3_112_MIN_BITS 112
#define DES3_112_BITS 112
#define DES3_112_MAX_BITS 112
#define DES3_NAME "3DES"
#define DES3_LONG_NAME "Three Key Triple DES"
#define DES3_MIN_BITS 168
#define DES3_BITS 168
#define DES3_MAX_BITS 168
#define SSF33_NAME "SSF33"
#define SSF33_LONG_NAME "SSF33"
#define SSF33_MIN_BITS 128
#define SSF33_BITS 128
#define SSF33_MAX_BITS 128
#define SM1_NAME "SM1"
#define SM1_LONG_NAME "SM1"
#define SM1_MIN_BITS 128
#define SM1_BITS 128
#define SM1_MAX_BITS 128
#define RC2_NAME "RC2"
#define RC2_LONG_NAME "RSA DataSecurity's RC2"
#define RC2_MIN_BITS 40
#define RC2_BITS 128
#define RC2_MAX_BITS 128
#define RC4_NAME "RC4"
#define RC4_LONG_NAME "RSA Data Security's RC4"
#define RC4_MIN_BITS 40
#define RC4_BITS 128
#define RC4_MAX_BITS 128

#define ALG_SID_SSF33  33
#define ALG_SID_SM1   34
#define CALG_SSF33               (ALG_CLASS_DATA_ENCRYPT|ALG_TYPE_BLOCK|ALG_SID_SSF33)
#define CALG_SM1               (ALG_CLASS_DATA_ENCRYPT|ALG_TYPE_BLOCK|ALG_SID_SM1)

// added in 2007-11-08, 
#define ALG_SSF33			0x00000001
#define ALG_SCB2			0x00000002
#define ALG_SCB2_S			0x00000004
#define ALG_RSA1024_SIGN	0x00000008
#define ALG_RSA2048_SIGN	0x00000010
#define ALG_ECC192_SIGN		0x00000020
#define ALG_SHA1			0x00000040
#define ALG_SHA256			0x00000080


#define ROOT_FILE		0x2d03
#define Pref_FileID     0x2d00
#define TOTAL_CONTAINER_AVAILABLE   3   // ��Ĭ�������⣬���ɴ���2����Կ����,ÿ���������2����Կ��2��֤�飬һ����������7���ļ�
#define CONTAINER_NAME_LEN  0x28		// ��Կ�������������39���ַ�
#define CONTAINER_NOT_USED  0			// ��ǰ��Կ����δʹ��
#define CONTAINER_USED		1			// ��ǰ��Կ������ʹ��

#define FILE_SIGNATURE		0			// ��ǰ�ļ��洢ǩ����Կ��
#define FILE_KEYEXCHANGE	3			// ��ǰ�ļ��洢������Կ��

#define ROOT_FILE_OFFSET    0x30		// ��Կ������Ϣ���ļ�3�ĵ�48��λ�ÿ�ʼ����


//================�ڲ����� 2007.12.03.hsz====================
// ����� 2007.12.5. hsz
#define Read_Hard          0x31    // ��ȡToken��Ϣ
#define GetSState          0x32    // �õ���ǰ�İ�ȫ״̬(0 ����״̬,1 �û�,2 ����Ա)
#define GetUPINRetryNum    0x33    // �û�PIN��ʣ�����Դ��� 
#define InitEEPROM         0x34    // ��ʼ��EEPROM�洢���ռ�
//���Token��ԭ����Ϣ�����ÿ���
#define Verify_PIN         0x35    // У����������(0 �û�, 1 ����Ա) 
#define Modify_PIN         0x36    // �޸���������(0 �û�, 1 ����Ա, 2 �����û�)  
#define Create_File1       0x37    // �����ļ�
#define Delete_File1       0x38    // ɾ���ļ�
#define ReadDataBlock      0x39    // ��ȡ����������
#define WriteDataBlock     0x3A    // ���¶���������
#define Open_File1         0x3B    // ���Ҳ�����ָ���ķ���
#define Close_File1        0x3C    // �رմ򿪵ķ���
#define BOOT_MODE        0x3d    // �ָ���COS����ģʽ                        
#define RNG_FUNC           0x5e    // ��������
#define SymCipher_KEYMANG  0x70    // �Գ��㷨��Կ���� 
#define DES_FUNC           0x71    // DES����
#define RSA_REALFUNC       0x72    // RSAʵ������
#define SCB2_FUNC          0x73    // scb2�������
#define SSF33_FUNC         0x75    // ssf33�������
#define RestSState         0x77    // ���õ�ǰ�İ�ȫ״̬
//#define GetFreeSpac0       0x78    // ���ص�ǰ���õĴ洢���ռ�
#define EnumFile           0x7b     // ö���ļ�
//2010.01.28 hsz
#define ModifyRetryNum      0x3f     // �޸���ͨ�û�PIN�����Դ���
//2010.06.09 hsz
#define Chang_Write_Protect	0x5f


/****************************************************************************
 * Private Types
 ****************************************************************************/

/***************************************************/
//  USBSLOCK �������CMD���ṹ����
// ˵����CMD���̶�����Ϊ16���ֽڣ����ṹ����7816-3��
//ͬʱ���н϶�ı�����������չ��� 
//����Ӧ�ȷ����������

/***************************************************/
typedef struct tagUSLock_CMD {
    uint8_t cmd_type; // 0 ��������   RSA�� �൱��7816-3 �е�INS
    uint8_t cmd_mingxi; // 1 ������ϸ 1 ���ܡ����ܣ��õ���Կ���൱��7816-3 �е�CLA
    uint8_t cmd_Reseved0; // 2 �����ֽڣ��������Զ�����;��RSA��Կλ��
    uint8_t cmd_Reseved1; // 3 �����ֽڣ��������Զ�����;��
    uint16_t cmd_OutLength; //4//5 ����CMDϣ��Bulk-out�˵㴫�͵����ݳ��ȡ��������Ϊ0�����δ��������ݴ��͡�
    uint16_t cmd_InLength; //6��7 ����CMDӦ��Ӧ�����ݳ��ȡ��������Ϊ0�����δ��������ݴ��䡣

    union _flag {
        struct _left {
            uint8_t cmd_P1; // 8  7816-3 �е�P1
            uint8_t cmd_P2; // 9  7816-3 �е�P2
            uint8_t cmd_Reseved2; // A �����ֽڣ��������Զ�����;��
            uint8_t cmd_Reseved3; // B �����ֽڣ��������Զ�����;��
        } left;
        uint32_t para; //�����������RSAλ�����������������ݵ�ַ
    } flag;

    uint8_t cmd_Reseved4; // C �����ֽڣ��������Զ�����;��
    uint8_t cmd_Reseved5; // D �����ֽڣ��������Զ�����;��
    uint8_t cmd_Reseved6; // C �����ֽڣ��������Զ�����;��
    uint8_t cmd_Reseved7; // D �����ֽڣ��������Զ�����;��
} KEUSLock_CMD, *LPUSLock_CMD;

/***************************************************/
// USBSLOCK ״̬����STAT�����ṹ����
// ˵����STAT���̶�����Ϊ16���ֽڣ����ṹ����7816-3��ͬʱ���н϶�ı�����������չ���
//�豸Ӧ�ȷ�״̬�������ݣ����״̬��ʾ���ɹ����򲻷�������

/***************************************************/
typedef struct tagUSLock_STAT {
    uint16_t bSTATStatus; // 0,1 ���ֶ�����ָʾCMD����ִ�гɹ�����ʧ�ܡ�
    uint8_t dSTATReseved0; // 2 �����ֽڣ��������Զ�����;��
    uint8_t dSTATDataResidue; // 3 Ϊ����7816-3Э�����á�ʣ�����ݳ��ȡ������������ݴ�����̡�
    uint32_t dSTATtag; // 4,5,6,7 ����Ҫ������Ӧ��CMD��Ӧ����䡣�ɸ��������Զ��塣
    uint32_t dSTATReseved1; // 8,9,a,b �����ֽڣ��������Զ�����;��
    uint8_t dSTATReseved2; // C �����ֽڣ��������Զ�����;��
    uint8_t dSTATReseved3; // D �����ֽڣ��������Զ�����;��
    uint8_t dSTATReseved4; // E �����ֽڣ��������Զ�����;��
    uint8_t dSTATReseved5; // F �����ֽڣ��������Զ�����;��
} KEUSLock_STAT, *LPUSLock_STAT;


/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int jksafekey_write_bulk(int fd, uint8_t cdb[USBMSC_MAXCDBLEN], size_t len, FAR void * buff)
{
  ssize_t ret;
  uint8_t* p = malloc(USBMSC_MAXCDBLEN + len);
  memcpy(p, cdb, USBMSC_MAXCDBLEN);
  memcpy(p + USBMSC_MAXCDBLEN, buff, len);
  ret = write(fd, p, USBMSC_MAXCDBLEN + len);
  if (ret == USBMSC_MAXCDBLEN + len)
  	{
		ret = OK;
  	}
  free(p);
  return ret;
}

static int jksafekey_read_bulk(int fd, uint8_t cdb[USBMSC_MAXCDBLEN], size_t len, FAR void * buff)
{
  ssize_t ret;
  uint8_t *p = malloc(USBMSC_MAXCDBLEN + len);
  memcpy(p, cdb, USBMSC_MAXCDBLEN);
  memcpy(p + USBMSC_MAXCDBLEN, buff, len);
  ret = read(fd, p, USBMSC_MAXCDBLEN + len);
  if (ret == USBMSC_MAXCDBLEN + len)
  	{
  	    memcpy(buff, p + USBMSC_MAXCDBLEN, len);
		ret = OK;
  	}
  free(p);
  return ret;
}

#define SWAP(x)   ((((x) & 0xFF) << 8) | (((x) >> 8) & 0xFF))
#define SWAP32(x)   ((SWAP((x) & 0xFFFF) << 16) | SWAP(((x) >> 16) & 0xFFFF))

static PLUG_RV jksafekey_transfer(
		 int fd,	
        KEUSLock_CMD *slock_cmd, // ָ������ �������ָ��
        void* InBuffer, // ����bufferָ��
        KEUSLock_STAT *slock_stat, // ָ�����״̬����ָ��
        void* OutBuffer // ���bufferָ��
        ) {        
  uint8_t GET_PARA[USBMSC_MAXCDBLEN] = {0xf7, 0x00, 0x00, 0x86, 0x10, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t GET_BUSY[USBMSC_MAXCDBLEN] = {0xf5, 0x00, 0x00, 0x86, 0x10, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t device_busy;
  uint8_t *InputBuffer, *OutputBuffer;

  InputBuffer = (uint8_t *)malloc(sizeof(KEUSLock_CMD) + slock_cmd->cmd_OutLength);
  OutputBuffer = (uint8_t *)malloc(sizeof(KEUSLock_STAT) + slock_cmd->cmd_InLength);

  memcpy(InputBuffer, slock_cmd, sizeof(KEUSLock_CMD));
  memcpy(InputBuffer + sizeof(KEUSLock_CMD) , InBuffer, slock_cmd->cmd_OutLength);
  //�ж��Ƿ�Ϊæ
  do {
      if (jksafekey_read_bulk(fd, GET_BUSY, 1, &device_busy) < 0) {
          slock_stat->bSTATStatus = RV_FAIL;
          return RV_FAIL;
      }
  } while (device_busy == 1);
  //
  if (jksafekey_write_bulk(fd, GET_PARA, sizeof(KEUSLock_CMD) + slock_cmd->cmd_OutLength, InputBuffer) < 0) {
      slock_stat->bSTATStatus = RV_FAIL;
      return RV_FAIL;
  }
  if (jksafekey_read_bulk(fd, GET_PARA, sizeof(KEUSLock_STAT) + slock_cmd->cmd_InLength, OutputBuffer) < 0) {
      slock_stat->bSTATStatus = RV_FAIL;
      return RV_FAIL;
  }
  //��Ϊ��æ
  device_busy = 0;
  if (jksafekey_write_bulk(fd, GET_BUSY, 0, &device_busy) < 0) {
      slock_stat->bSTATStatus = RV_FAIL;
      return RV_FAIL;
  }
  memcpy(slock_stat, OutputBuffer, sizeof(KEUSLock_STAT));
  slock_stat->bSTATStatus = SWAP(slock_stat->bSTATStatus);	
  if ((slock_cmd->cmd_InLength != 0) && (slock_stat->bSTATStatus == RV_OK))
      memcpy(OutBuffer, OutputBuffer + sizeof(KEUSLock_STAT), slock_cmd->cmd_InLength);

  free(InputBuffer);
  free(OutputBuffer);

  return slock_stat->bSTATStatus;
}

//��֤�û�����

PLUG_RV jksafekey_verify_pin(int fd, char *pin) {
	PLUG_RV ret;
	uint8_t temp[16];
	KEUSLock_CMD slock_cmd = {0};
	KEUSLock_STAT slock_stat = {0};

	int len = strlen(pin);
	if (len < 4) 
	  {
	    return RV_PIN_INCORRECT;
	  }

	memset(&slock_cmd, 0, 16);
	slock_cmd.cmd_type = Verify_PIN;
	slock_cmd.cmd_mingxi = 0;
	slock_cmd.cmd_OutLength = 16;
	
	memset(temp, 0, 16);
	memcpy(temp, pin, len);
	
	if ((ret = jksafekey_transfer(fd, &slock_cmd, temp, &slock_stat, NULL)) != RV_OK)
	  {
	    return ret;
	  }

	return RV_OK;
}

//��ȡ����������(����1024�ֽڵ������������)

static PLUG_RV jksafekey_read_binary(int fd, uint16_t fileid, uint8_t* data, uint32_t offset, uint32_t len) {
  PLUG_RV ret;
  unsigned char temp[2];
  KEUSLock_CMD slock_cmd = {0};
  KEUSLock_STAT slock_stat = {0};
  int datalen1 = len, offset1 = offset;

  if ((fileid > 0x2D21) || (fileid < 0x2D01)) return RV_NOT_SUPPORT;
  //���ļ�
  slock_cmd.cmd_type = Open_File1;
  slock_cmd.cmd_OutLength = 2;
    *((uint16_t *) (temp)) = fileid & 0xff;

  if ((ret = jksafekey_transfer(fd, &slock_cmd, temp, &slock_stat, NULL)) != RV_OK)
    return ret;
  //��ȡ�ļ�
  memset(&slock_cmd, 0, sizeof(KEUSLock_CMD));
  slock_cmd.cmd_type = ReadDataBlock;
//�������1024�ֽڵ����
  while (datalen1 > 1024) {
    slock_cmd.flag.para = offset1;
    slock_cmd.cmd_InLength = 1024;
    if ((ret = jksafekey_transfer(fd, &slock_cmd, NULL, &slock_stat, data)) != RV_OK)
        return ret;
    data += 1024;
    offset1 += 1024;
    datalen1 -= 1024;
  }
  if (datalen1) {
    slock_cmd.flag.para = offset1;
    slock_cmd.cmd_InLength = datalen1;
    if ((ret = jksafekey_transfer(fd, &slock_cmd, NULL, &slock_stat, data)) != RV_OK)
        return ret;
  }
  //�ر��ļ�
  memset(&slock_cmd, 0, 16);
  slock_cmd.cmd_type = Close_File1;
  return jksafekey_transfer(fd, &slock_cmd, NULL, &slock_stat, NULL);
}

static PLUG_RV jksafekey_get_data(int fd, int filetype, uint8_t* data, uint32_t* len) {
    PLUG_RV ret;
    uint8_t tmp[4];
    int fileid;
    switch (filetype) {
        case CERT_FILE_SIGN:
        case CERT_FILE_KEYX:
            fileid = 0x0B + (filetype == CERT_FILE_SIGN ? 3 : 6);
            ret = jksafekey_read_binary(fd, Pref_FileID + fileid, tmp, 0, 4);

            if (tmp[0] == 0)
                return RV_FAIL;
            if (!data) {
                *len = (uint32_t) tmp[2]*256 + (uint32_t) tmp[3] + 4;
                break;
            }
            ret = jksafekey_read_binary(fd, Pref_FileID + fileid, data, 0, *len);
            break;
        case RSA_PUB_KEY_FILE_SIGN:
            fileid = 0x0B + 2;
            ret = jksafekey_read_binary(fd, Pref_FileID + fileid, data, 0, *len);
            if (data[0] == 0)
                return RV_FAIL;
            break;
        case RSA_PUB_KEY_FILE_KEYX:
            fileid = 0x0B + 5;
            ret = jksafekey_read_binary(fd, Pref_FileID + fileid, data, 0, *len);
            if (data[0] == 0)
                return RV_FAIL;
            break;
        case CONTAINER_FILE:
            fileid = 0x0B;
            ret = jksafekey_read_binary(fd, Pref_FileID + fileid, data, 0, *len); //CONTAINER_NAME_LEN);
            break;
        default:
            fileid = 0;
    }
    if (fileid) {
        return ret;
    }
    return RV_FAIL;
}

PLUG_RV jksafekey_get_pubkey(int fd, uint8_t keyspec, uint8_t* data) {
    uint32_t keylen = 128;
    return jksafekey_get_data(fd, (keyspec == AT_KEYEXCHANGE ? RSA_PUB_KEY_FILE_KEYX : RSA_PUB_KEY_FILE_SIGN), data, &keylen);
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/


