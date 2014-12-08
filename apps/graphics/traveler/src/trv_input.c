/****************************************************************************
 * apps/graphics/traveler/trv_input.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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

#include "trv_types.h"
#include "trv_world.h"
#include "trv_trigtbl.h"
#include "trv_input.h"

#if defined(CONFIG_GRAPHICS_TRAVELER_JOYSTICK)
#  include <fcntl.h>
#  include <errno.h>

#if defined(CONFIG_GRAPHICS_TRAVELER_AJOYSTICK)
#  include <fixedmath.h>
#  include <nuttx/input/ajoystick.h>
#elif defined(CONFIG_GRAPHICS_TRAVELER_DJOYSTICK)
#  include <nuttx/input/djoystick.h>
#endif

#elif defined(CONFIG_GRAPHICS_TRAVELER_NX_XYINPUT)
#endif

/****************************************************************************
 * Pre-processor Definitions
 *************************************************************************/

/****************************************************************************
 * Private Types
 *************************************************************************/

struct trv_input_info_s
{
#if defined(CONFIG_GRAPHICS_TRAVELER_JOYSTICK)
  int fd;                   /* Open driver descriptor */
#ifdef CONFIG_GRAPHICS_TRAVELER_AJOYSTICK
  int16_t centerx;          /* Center X position */
  b16_t leftslope;          /* Slope for left of center */
  b16_t rightslope;         /* Slope for left of center */
  int16_t centery;          /* Center Y position */
  b16_t fwdslope;           /* Slope for forward from center */
  b16_t backslope;          /* Slope for backward from center */
#endif
#elif defined(CONFIG_GRAPHICS_TRAVELER_NX_XYINPUT)
  trv_coord_t xpos;         /* Reported X position */
  trv_coord_t ypos;         /* Reported Y position */
  uint8_t buttons;          /* Report button set */
#endif
};

/****************************************************************************
 * Public Data
 *************************************************************************/

/* Report positional inputs */

struct trv_input_s g_trv_input;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_GRAPHICS_TRAVELER_NX_XYINPUT
static struct trv_input_info_s g_trv_input_info;
#endif

/****************************************************************************
 * Private Data
 *************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: trv_joystick_calibrate
 *
 * Description:
 *   Calibrate the joystick
 *
 ****************************************************************************/

#ifdef CONFIG_GRAPHICS_TRAVELER_AJOYSTICK
static void trv_joystick_calibrate(void)
{
#warning Missing logic"
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: trv_input_initialize
 *
 * Description:
 *   Initialize the selected input device
 *
 ****************************************************************************/

void trv_input_initialize(void)
{
#if defined(CONFIG_GRAPHICS_TRAVELER_JOYSTICK)
  /* Open the joy stick device */

  g_trv_input_info.fd = open(CONFIG_GRAPHICS_TRAVELER_JOYDEV, O_RDONLY);
  if (g_trv_input_info.fd < 0)
    {
      trv_abort("ERROR: Failed to open %s: %d\n",
                CONFIG_GRAPHICS_TRAVELER_JOYDEV, errno);
    }

#ifdef CONFIG_GRAPHICS_TRAVELER_AJOYSTICK
  /* Calibrate the analog joystick device */

  trv_joystick_calibrate();
#endif

#elif defined(CONFIG_GRAPHICS_TRAVELER_NX_XYINPUT)
  /* Set the position to the center of the display at eye-level */
#warning Missing logic
#endif
}

/****************************************************************************
 * Name: trv_input_read
 *
 * Description:
 *   Read the next input from the input device
 *
 ****************************************************************************/

void trv_input_read(void)
{
#if defined(CONFIG_GRAPHICS_TRAVELER_JOYSTICK)
#if defined(CONFIG_GRAPHICS_TRAVELER_AJOYSTICK)
  struct ajoy_sample_s sample;
  int16_t move_rate;
  int16_t turn_rate;
  ssize_t nread;

  /* Read data from the analog joystick */

  nread = read(g_trv_input_info.fd, &sample, sizeof(struct ajoy_sample_s));
  if (nread < 0)
    {
      trv_abort("ERROR: Joystick read error: %d\n", errno);
    }
  else if (nread != sizeof(struct ajoy_sample_s))
    {
      trv_abort("ERROR: Unexpected joystick read size: %ld\n", (long)nread);
    }

  /* Determine the input data to return to the POV logic */
#warning Missing logic

#elif defined(CONFIG_GRAPHICS_TRAVELER_DJOYSTICK)
  struct djoy_buttonset_t buttonset;
  ssize_t nread;

  /* Read data from the discrete joystick */

  nread = read(g_trv_input_info.fd, &buttonset, sizeof(djoy_buttonset_t));
  if (nread < 0)
    {
      trv_abort("ERROR: Joystick read error: %d\n", errno);
    }
  else if (nread != sizeof(struct djoy_buttonset_t))
    {
      trv_abort("ERROR: Unexpected joystick read size: %ld\n", (long)nread);
    }

  /* Determine the input data to return to the POV logic */

  if ((buttonset & DJOY_BUTTON_RUN_BIT) != 0)
    {
      /* Run faster/step higher */

      g_trv_input.stepheight = g_run_step_height;
      move_rate              = 2 * STEP_DISTANCE;
      turn_rate              = ANGLE_9;
    }
  else
    {
      /* Normal walking rate and stepping height */

      g_trv_input.stepheight = g_walk_step_height;
      move_rate              = STEP_DISTANCE;
      turn_rate              = ANGLE_6;
    }

  /* Move forward or backward OR look up or down */

  g_trv_input.pitchrate = 0;
  g_trv_input.fwdrate   = 0;

  switch (buttonset & (DJOY_UP_BIT | DJOY_DOWN_BIT))
    {
      case 0:
      case (DJOY_UP_BIT | DJOY_DOWN_BIT):
        /* Don't move, don't nod */

        break;

      case DJOY_UP_BIT:
        if ((buttonset & DJOY_BUTTON_FIRE_BIT) != 0)
          {
            /* Look upward */

            g_trv_input.pitchrate = turn_rate;
          }
        else
          {
            /* Move forward */

            g_trv_input.fwdrate   = move_rate;
          }
        break;

      case DJOY_DOWN_BIT:
        if ((buttonset & DJOY_BUTTON_FIRE_BIT) != 0)
          {
            /* Look downward */

            g_trv_input.pitchrate = -turn_rate;
          }
        else
          {
            /* Move Backward */

            g_trv_input.fwdrate = -move_rate;
          }
        break;
    }

  /* Move forward or backward OR look up or down */

  g_trv_input.yawrate  = 0;
  g_trv_input.leftrate = 0;

  switch (buttonset & (DJOY_LEFT_BIT | DJOY_RIGHT_BIT))
    {
      case 0:
      case (DJOY_LEFT_BIT | DJOY_RIGHT_BIT):
        /* Don't move, don't nod */

        break;

      case DJOY_LEFT_BIT:
        if ((buttonset & DJOY_BUTTON_FIRE_BIT) != 0)
          {
            /* Turn left */

            g_trv_input.yawrate = turn_rate;
          }
        else
          {
            /* Move left */

            g_trv_input.leftrate   = move_rate;
          }
        break;

      case DJOY_RIGHT_BIT:
        if ((buttonset & DJOY_BUTTON_FIRE_BIT) != 0)
          {
            /* Turn right */

            g_trv_input.yawrate = -turn_rate;
          }
        else
          {
            /* Move right */

            g_trv_input.leftrate = -move_rate;
          }
        break;
    }

  g_trv_input.leftrate = ((buttonset & DJOY_BUTTON_SELECT) != 0);

#endif /* CONFIG_GRAPHICS_TRAVELER_DJOYSTICK */
#elif defined(CONFIG_GRAPHICS_TRAVELER_NX_XYINPUT)
  /* Make position decision based on last sampled X/Y input data */
#warning Missing logic
#endif
}

/****************************************************************************
 * Name: trv_input_terminate
 *
 * Description:
 *   Terminate input and free resources
 *
 ****************************************************************************/

void trv_input_terminate(void)
{
#ifdef CONFIG_GRAPHICS_TRAVELER_JOYSTICK
#endif
#warning Missing Logic
}

/****************************************************************************
 * Name: trv_input_xyinput
 *
 * Description:
 *   Receive X/Y input from NX
 *
 ****************************************************************************/

#ifdef CONFIG_GRAPHICS_TRAVELER_NX_XYINPUT
void trv_input_xyinput(trv_coord_t xpos, trv_coord_t xpos, uint8_t buttons)
{
  /* Just save the positional data and button presses for now.  We will
   * decide what to do with the data when we are polled for new input.
   */

  g_trv_input_info.xpos    = xpos;
  g_trv_input_info.ypos    = ypos;
  g_trv_input_info.buttons = buttons;
}
#endif
