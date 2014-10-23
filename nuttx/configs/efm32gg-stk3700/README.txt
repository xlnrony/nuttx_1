README
======

  This README file discusses the port of NuttX to the Silicon Labs EFM32™
  Giant Gecko Starter Kit (EFM32GG-STK3400). The Giant Gecko Starter Kit
  features:

    • EFM32GG990F1024 MCU with 1 MB flash and 128 kB RAM
    •   32.768 kHz crystal (LXFO)
    •   48 MHz crystal (HXFO)
    • 32 MB NAND flash
    • Advanced Energy Monitoring
    • Touch slider
    • 8x20 LCD
    • 2 user LEDs
    • 2 user buttons
    • USB interface for Host/Device/OTG
    • Ambient light sensor and inductive-capacitive metal sensor
    • EFM32 OPAMP footprint
    • 20 pin expansion header
    • Breakout pads for easy access to I/O pins
    • Power sources (USB and CR2032 battery)
    • Backup Capacitor for RTC mode
    • Integrated Segger J-Link USB debugger/emulator

STATUS
======

  This README now exists only as some analysis for a port to the EFM32 Giant
  Gecko Starter Kit.  That port has not yet been developed and I do not now
  have hardware in hand.  So the status is partially analyzed, but not yet implemented.

LEDs and Buttons
================

  LEDs
  ----
  The EFM32 Giant Gecko Start Kit has two yellow LEDs marke LED0 and LED1.
  These LEDs are controlled by GPIO pins on the EFM32.  The LEDs are
  connected to pins PE2 and PE3 in an active high configuration:

  ------------------------------------- --------------------
  EFM32 PIN                             BOARD SIGNALS
  ------------------------------------- --------------------
  E2/BCK_VOUT/EBI_A09 #0/               MCU_PE2 UIF_LED0
    TIM3_CC2 #1/U1_TX #3/ACMP0_O #1
  E3/BCK_STAT/EBI_A10 #0/U1_RX #3/      MCU_PE3 UIF_LED1
    ACMP1_O #1
  ------------------------------------- --------------------

  All LEDs are grounded and so are illuminated by outputting a high
  value to the LED.

  These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
  defined.  In that case, the usage by the board port is defined in
  include/board.h and src/efm32_autoleds.c.  The LEDs are used to
  encode OS-related events as follows:

  These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
  defined.  In that case, the usage by the board port is defined in
  include/board.h and src/up_leds.c. The LEDs are used to encode
  OS-related events as follows:

    SYMBOL               Meaning                      LED state
                                                    LED0     LED1
    -------------------  -----------------------  -------- --------
    LED_STARTED          NuttX has been started     OFF      OFF
    LED_HEAPALLOCATE     Heap has been allocated    OFF      OFF
    LED_IRQSENABLED      Interrupts enabled         OFF      OFF
    LED_STACKCREATED     Idle stack created         ON       OFF
    LED_INIRQ            In an interrupt              No change
    LED_SIGNAL           In a signal handler          No change
    LED_ASSERTION        An assertion failed          No change
    LED_PANIC            The system has crashed     OFF      Blinking
    LED_IDLE             STM32 is is sleep mode       Not used

  Buttons
  -------
  The EFM32 Giant Gecko Start Kit has two buttons marked PB0 and PB1. They
  are connected to the EFM32, and are debounced by RC filters with a time
  constant of 1ms. The buttons are connected to pins PB9 and PB10:

  ------------------------------------- --------------------
  EFM32 PIN                             BOARD SIGNALS
  ------------------------------------- --------------------
  B9/EBI_A03/U1_TX #2                   MCU_PB9  UIF_PB0
  B10/EBI_A04/U1_RX #2                  MCU_PB10 UIF_PB1
  ------------------------------------- --------------------

  Buttons are connected to ground so they will read low when closed.

Serial Console
==============


   Default Serial Console
   ----------------------
   UART0 is configured as the default serial console at 115200 8N1
   on pins PE0 and PE1.

   Communication through the Board Controller
   ------------------------------------------
   The kit contains a board controller that is responsible for performing
   various board level tasks, such as handling the debugger and the Advanced
   Energy Monitor. An interface is provided between the EFM32 and the board
   controller in the form of a UART connection. The connection is enabled by
   setting the EFM_BC_EN (PF7) line high, and using the lines EFM_BC_TX
   (PE0) and EFM_BC_RX (PE1) for communicating.

Configurations
==============
  Each EFM32 Giant Gecko Starter Kit configuration is maintained in a sub-
  directory and can be selected as follow:

    cd tools
    ./configure.sh efm32gg-stk3700/<subdir>
    cd -
    . ./setenv.sh

  If this is a Windows native build, then configure.bat should be used
  instead of configure.sh:

    configure.bat efm32gg-stk3700\<subdir>

  Where <subdir> is one of the following:

  nsh:
  ---
    Configures the NuttShell (nsh) located at apps/examples/nsh.  The
    Configuration enables the serial interfaces on UART0.  Support for
    built-in applications is enabled, but in the base configuration no
    built-in applications are selected (see NOTES below).

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          and misc/tools/

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. By default, this configuration uses the CodeSourcery toolchain
       for Windows and builds under Cygwin (or probably MSYS).  That
       can easily be reconfigured, of course.

       CONFIG_HOST_WINDOWS=y                   : Builds under Windows
       CONFIG_WINDOWS_CYGWIN=y                 : Using Cygwin
       CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYW=y : CodeSourcery for Windows
