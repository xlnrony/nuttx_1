#ifndef __INCLUDE_VERSION_H
#define __INCLUDE_VERSION_H

#include <nuttx/version.h>
#include <stdint.h>

#ifdef CONFIG_VERSION

/************************************************************************
 * Pre-processor Definitions
 ************************************************************************/

/************************************************************************
 * Public Type Declarations
 ************************************************************************/

/************************************************************************
 * Public Variables
 ************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

EXTERN uint16_t version(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif
#endif /* __INCLUDE_VERSION_H */
