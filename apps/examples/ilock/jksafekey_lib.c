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
 *  算法类型
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
 *  算法模式
 */
typedef enum _ALGO_MODE{
    SYMMETRIC_ECB = 0,
    SYMMETRIC_CBC = 1,
    SYMMETRIC_CFB = 2,
    SYMMETRIC_OFB = 3,
    K_ENUM_FORCE(ALGO_MODE)
}ALGO_MODE;

/*
 *  Token状态
 */
typedef enum _TOKEN_STATUS{
    TOKEN_OFFLINE = 0,
    TOKEN_ONLINE  = 1,
    TOKEN_LOGIN   = 2,
    K_ENUM_FORCE(TOKEN_STATUS)
}TOKEN_STATUS;

/*
 *  PADDING类型
 */
typedef enum _PADDING_TYPE{
    PKCS1_PADDING       = 0x0000,
    K_ENUM_FORCE(PADDING_TYPE)
}PADDING_TYPE;

/*
 *  Token信息
 */
typedef struct _PT_Token_Info {
    unsigned char nMaxSOPINLen; /*  管理员PIN的最大长度 */
    unsigned char nMinSOPINLen; /*  管理员PIN的最小长度 */
    unsigned char nMaxPINLen; /*  用户PIN的长度   */
    unsigned char nMinPINLen; /*  用户PIN的长度   */
    unsigned short nAlgoCap; /*  密码运算能力属性，参见上面的宏  */
    char szSerialNumber[16]; /*  序列号，用于区分Token的唯一标识 */
    unsigned long nTotalMemory; /*  Token总空间，字节数 */
    char szLabel[32]; /* Token标签 */
} PT_Token_Info, *PPT_Token_Info;

/*
 *  PIN类型
 */
typedef enum _PIN_TYPE{
    SO_PIN      = 0,
    USER_PIN    = 1,
    K_ENUM_FORCE(PIN_TYPE)
}PIN_TYPE;

/*
 *  RSA密钥位数，生成RSA密钥时使用
 */
typedef enum _PLUGTOKEN_RSA_BITS{
    PLUGRSA_512_BIT     =512,
    PLUGRSA_1024_BIT    =1024,
    PLUGRSA_2048_BIT    =2048,
    K_ENUM_FORCE(PLUGTOKEN_RSA_BITS)
}PLUGTOKEN_RSA_BITS;

/*
 *  Token中文件的类型
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
 *  Token中文件的访问权限
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
#define TOTAL_CONTAINER_AVAILABLE   3   // 除默认容器外，最多可创建2个密钥容器,每个容器存放2对密钥，2个证书，一个容器名共7个文件
#define CONTAINER_NAME_LEN  0x28		// 密钥容器的名称最大39个字符
#define CONTAINER_NOT_USED  0			// 当前密钥容器未使用
#define CONTAINER_USED		1			// 当前密钥容器已使用

#define FILE_SIGNATURE		0			// 当前文件存储签名密钥对
#define FILE_KEYEXCHANGE	3			// 当前文件存储加密密钥对

#define ROOT_FILE_OFFSET    0x30		// 密钥容器信息从文件3的第48个位置开始设置


//================内部函数 2007.12.03.hsz====================
// 命令定义 2007.12.5. hsz
#define Read_Hard          0x31    // 获取Token信息
#define GetSState          0x32    // 得到当前的安全状态(0 匿名状态,1 用户,2 管理员)
#define GetUPINRetryNum    0x33    // 用户PIN码剩余重试次数 
#define InitEEPROM         0x34    // 初始化EEPROM存储器空间
//清除Token中原有信息并设置口令
#define Verify_PIN         0x35    // 校验密码命令(0 用户, 1 管理员) 
#define Modify_PIN         0x36    // 修改密码命令(0 用户, 1 管理员, 2 解锁用户)  
#define Create_File1       0x37    // 创建文件
#define Delete_File1       0x38    // 删除文件
#define ReadDataBlock      0x39    // 读取二进制数据
#define WriteDataBlock     0x3A    // 更新二进制数据
#define Open_File1         0x3B    // 查找并激活指定的分区
#define Close_File1        0x3C    // 关闭打开的分区
#define BOOT_MODE        0x3d    // 恢复到COS下载模式                        
#define RNG_FUNC           0x5e    // 获得随机数
#define SymCipher_KEYMANG  0x70    // 对称算法密钥管理 
#define DES_FUNC           0x71    // DES运算
#define RSA_REALFUNC       0x72    // RSA实际运算
#define SCB2_FUNC          0x73    // scb2运算测试
#define SSF33_FUNC         0x75    // ssf33运算测试
#define RestSState         0x77    // 重置当前的安全状态
//#define GetFreeSpac0       0x78    // 返回当前可用的存储器空间
#define EnumFile           0x7b     // 枚举文件
//2010.01.28 hsz
#define ModifyRetryNum      0x3f     // 修改普通用户PIN码重试次数
//2010.06.09 hsz
#define Chang_Write_Protect	0x5f


/****************************************************************************
 * Private Types
 ****************************************************************************/

/***************************************************/
//  USBSLOCK 命令包（CMD）结构定义
// 说明：CMD包固定长度为16个字节，本结构兼容7816-3。
//同时还有较多的保留字用于扩展命令。 
//主机应先发命令，后发数据

/***************************************************/
typedef struct tagUSLock_CMD {
    uint8_t cmd_type; // 0 命令类型   RSA； 相当于7816-3 中的INS
    uint8_t cmd_mingxi; // 1 命令明细 1 加密、解密，得到密钥；相当于7816-3 中的CLA
    uint8_t cmd_Reseved0; // 2 保留字节，可用于自定义用途。RSA公钥位数
    uint8_t cmd_Reseved1; // 3 保留字节，可用于自定义用途。
    uint16_t cmd_OutLength; //4//5 本次CMD希望Bulk-out端点传送的数据长度。如果长度为0，本次传输无数据传送。
    uint16_t cmd_InLength; //6，7 本次CMD应回应的数据长度。如果长度为0，本次传输无数据传输。

    union _flag {
        struct _left {
            uint8_t cmd_P1; // 8  7816-3 中的P1
            uint8_t cmd_P2; // 9  7816-3 中的P2
            uint8_t cmd_Reseved2; // A 保留字节，可用于自定义用途。
            uint8_t cmd_Reseved3; // B 保留字节，可用于自定义用途。
        } left;
        uint32_t para; //命令参数，如RSA位数，数据索引，数据地址
    } flag;

    uint8_t cmd_Reseved4; // C 保留字节，可用于自定义用途。
    uint8_t cmd_Reseved5; // D 保留字节，可用于自定义用途。
    uint8_t cmd_Reseved6; // C 保留字节，可用于自定义用途。
    uint8_t cmd_Reseved7; // D 保留字节，可用于自定义用途。
} KEUSLock_CMD, *LPUSLock_CMD;

/***************************************************/
// USBSLOCK 状态包（STAT）包结构定义
// 说明：STAT包固定长度为16个字节，本结构兼容7816-3。同时还有较多的保留字用于扩展命令。
//设备应先返状态，后返数据；如果状态表示不成功，则不返回数据

/***************************************************/
typedef struct tagUSLock_STAT {
    uint16_t bSTATStatus; // 0,1 该字段用来指示CMD命令执行成功或者失败。
    uint8_t dSTATReseved0; // 2 保留字节，可用于自定义用途。
    uint8_t dSTATDataResidue; // 3 为兼容7816-3协议设置。剩余数据长度。用来监视数据传输过程。
    uint32_t dSTATtag; // 4,5,6,7 备需要根据相应的CMD响应，填充。可根据命令自定义。
    uint32_t dSTATReseved1; // 8,9,a,b 保留字节，可用于自定义用途。
    uint8_t dSTATReseved2; // C 保留字节，可用于自定义用途。
    uint8_t dSTATReseved3; // D 保留字节，可用于自定义用途。
    uint8_t dSTATReseved4; // E 保留字节，可用于自定义用途。
    uint8_t dSTATReseved5; // F 保留字节，可用于自定义用途。
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
        KEUSLock_CMD *slock_cmd, // 指向输入 命令包的指针
        void* InBuffer, // 输入buffer指针
        KEUSLock_STAT *slock_stat, // 指向输出状态包的指针
        void* OutBuffer // 输出buffer指针
        ) {        
  uint8_t GET_PARA[USBMSC_MAXCDBLEN] = {0xf7, 0x00, 0x00, 0x86, 0x10, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t GET_BUSY[USBMSC_MAXCDBLEN] = {0xf5, 0x00, 0x00, 0x86, 0x10, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t device_busy;
  uint8_t *InputBuffer, *OutputBuffer;

  InputBuffer = (uint8_t *)malloc(sizeof(KEUSLock_CMD) + slock_cmd->cmd_OutLength);
  OutputBuffer = (uint8_t *)malloc(sizeof(KEUSLock_STAT) + slock_cmd->cmd_InLength);

  memcpy(InputBuffer, slock_cmd, sizeof(KEUSLock_CMD));
  memcpy(InputBuffer + sizeof(KEUSLock_CMD) , InBuffer, slock_cmd->cmd_OutLength);
  //判断是否为忙
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
  //设为不忙
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

//验证用户口令

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

//读取二进制数据(大于1024字节的情况，待处理)

static PLUG_RV jksafekey_read_binary(int fd, uint16_t fileid, uint8_t* data, uint32_t offset, uint32_t len) {
  PLUG_RV ret;
  unsigned char temp[2];
  KEUSLock_CMD slock_cmd = {0};
  KEUSLock_STAT slock_stat = {0};
  int datalen1 = len, offset1 = offset;

  if ((fileid > 0x2D21) || (fileid < 0x2D01)) return RV_NOT_SUPPORT;
  //打开文件
  slock_cmd.cmd_type = Open_File1;
  slock_cmd.cmd_OutLength = 2;
    *((uint16_t *) (temp)) = fileid & 0xff;

  if ((ret = jksafekey_transfer(fd, &slock_cmd, temp, &slock_stat, NULL)) != RV_OK)
    return ret;
  //读取文件
  memset(&slock_cmd, 0, sizeof(KEUSLock_CMD));
  slock_cmd.cmd_type = ReadDataBlock;
//处理大于1024字节的情况
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
  //关闭文件
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


