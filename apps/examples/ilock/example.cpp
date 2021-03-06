#if 0
int ilock_main(int argc, char *argv[])
{
//  pid_t pid;
//  ssize_t nbytes;
  int fd;
  int ret;
  bool connected = false;
  int rhpndx;
  static FAR struct usbhost_connection_s *g_usbconn;

  /* First, register all of the USB host HID keyboard class driver */

  printf("epass3003_main: Register class drivers\n");
  ret = usbhost_epass3003init();
  if (ret != OK)
    {
      printf("epass3003_main: Failed to register the KBD class\n");
    }

  /* Then get an instance of the USB host interface.  The platform-specific
   * code must provide a wrapper called arch_usbhost_initialize() that will
   * perform the actual USB host initialization. */

  printf("epass3003_main: Initialize epass3003 driver\n");
  g_usbconn = usbhost_initialize(0);
  if (g_usbconn)
    {
      /* Start a thread to handle device connection. */

      printf("epass3003_main: Start epass3003_waiter\n");

      /* Wait for the device to change state. REVISIT: This will not handle
       * USB implementations (such as the the SAMA5) which have multiple
       * downstream, root hub ports.  In such cases, connected must be an array
       * with dimension equal to the number of root hub ports. */

      rhpndx = usbhost_connection_wait(g_usbconn, &connected);
      DEBUGASSERT(rhpndx == OK);

      connected = !connected;
      printf("epass3003_waiter: %s\n",
             connected ? "connected" : "disconnected");

      /* Did we just become connected? */

      if (!connected)
        {
          /* Yes.. enumerate the newly connected device */
          printf
          ("epass3003_main: epass3003_waiter returned but not connected\n");
          return 0;
        }

      ret = usbhost_connection_enumerate(g_usbconn, rhpndx);
      if (ret < 0)
        {
          printf("usbhost_connection_enumerate failed: %d\n", ret);
          fflush(stdout);
          return 0;
        }
      printf("Opening device %s\n", CONFIG_EXAMPLES_EPASS3003_DEVNAME);
      fd = open(CONFIG_EXAMPLES_EPASS3003_DEVNAME, O_RDWR);
      if (fd < 0)
        {
          printf("Failed: %d\n", errno);
          fflush(stdout);
          return 0;
        }

      printf("Device %s opened\n", CONFIG_EXAMPLES_EPASS3003_DEVNAME);
      fflush(stdout);

      uint8_t txbuf[] = "\x00\x84\x00\x00\x08";
      size_t txpktlen = 5;
      uint8_t rxbuf[128] = { 0 };
      size_t rxlen = 128;
      char onebyte[4] = { 0 };
      char rxfmtbuf[384] = { 0 };

      ret = epass3003_transmit_apdu(fd, txbuf, txpktlen, rxbuf, &rxlen);
      if (ret < 0)
        {
          printf("epass3003_transmit_apdu failed: %d\n", ret);
          fflush(stdout);
          goto errout;
        }

      int i;
      for (i = 0; i < rxlen; i++)
        {
          sprintf(onebyte, "%02x ", rxbuf[i]);
          strcat(rxfmtbuf, onebyte);
        }
      printf("epass3003_transmit_apdu result:%s\n", rxfmtbuf);

errout:
      printf("Closing device %s\n", CONFIG_EXAMPLES_EPASS3003_DEVNAME);
      fflush(stdout);
      close(fd);

    }

  return 0;
}

#endif

#if 0
int ilock_main(int argc, char *argv[])
{
  int fd;
  int errval = 0;
  int ret;

  if (argc != 3)
    {
      printf("gpio:main: Need two extra parameters\n");
      errval = 3;
      goto errout;
    }

  printf("gpio_main: Hardware initialized. Opening the gpio device: %s\n",
         argv[1]);

  fd = open(argv[1], 0);
  if (fd < 0)
    {
      printf("adc_main: open %s failed: %d\n", argv[1], errno);
      errval = 1;
      goto errout;
    }

  fflush(stdout);

  ret = ioctl(fd, GPIOC_WRITE, *argv[2] - '0');
  if (ret < 0)
    {
      int errcode = errno;
      printf("gpio_main: GPIOC_WRITE ioctl failed: %d\n", errcode);
      errval = 2;
      goto errout_with_dev;
    }

  getchar();

  close(fd);
  fflush(stdout);

  return OK;

  /* Error exits */

errout_with_dev:
  close(fd);

errout:
  printf("Terminating!\n");
  fflush(stdout);
  return errval;
}
#endif

#if 0
int ilock_main(int argc, char *argv[])
{
  int fd;
  PLUG_RV ret;

  printf("Opening device %s\n", CONFIG_EXAMPLES_JKSAFEKEY_DEVNAME);
  fd = open(CONFIG_EXAMPLES_JKSAFEKEY_DEVNAME, O_RDWR);
  if (fd < 0)
    {
      printf("Failed: %d\n", errno);
      fflush(stdout);
      return 0;
    }

  printf("Device %s opened\n", CONFIG_EXAMPLES_JKSAFEKEY_DEVNAME);
  fflush(stdout);

#  if 0
  ret = jksafekey_verify_pin(fd, "123466");
  if (ret < 0)
    {
      printf("jksafekey_verify_pin failed: %d\n", ret);
      fflush(stdout);
      goto errout;
    }
#  endif

  uint8_t pubkey[128] = { 0 };

  ret = jksafekey_get_pubkey(fd, AT_SIGNATURE, pubkey);
  if (ret != RV_OK)
    {
      printf("jksafekey_get_pubkey failed: %d\n", ret);
      fflush(stdout);
      goto errout;
    }

  int i;
  char onebyte[4] = { 0 };
  char rxfmtbuf[384] = { 0 };

  for (i = 0; i < 128; i++)
    {
      sprintf(onebyte, "%02x ", pubkey[i]);
      strcat(rxfmtbuf, onebyte);
    }
  printf("jksafekey_get_pubkey result:%s\n", rxfmtbuf);

errout:
  printf("Closing device %s\n", CONFIG_EXAMPLES_JKSAFEKEY_DEVNAME);
  fflush(stdout);
  close(fd);

  return 0;
}
#endif

#if 0
int ilock_main(int argc, char *argv[])
{
  struct adc_msg_s sample;
  size_t readsize;
  ssize_t nbytes;

  int mvfd;
  int mgfd;
  int timfd;

  int errval = 0;
  int ret;

  printf("Opening the gpio device: %s\n", CONFIG_MAGNET_VCC_DEVNAME);
  mvfd = open(CONFIG_MAGNET_VCC_DEVNAME, 0);
  if (mvfd < 0)
    {
      printf("open %s failed: %d\n", CONFIG_MAGNET_VCC_DEVNAME, errno);
      errval = 1;
      goto errout;
    }

  printf("Opening the gpio device: %s\n", CONFIG_MAGNET_GND_DEVNAME);
  mgfd = open(CONFIG_MAGNET_GND_DEVNAME, 0);
  if (mgfd < 0)
    {
      printf("open %s failed: %d\n", CONFIG_MAGNET_GND_DEVNAME, errno);
      errval = 1;
      goto errout_with_mvfd;
    }

  fflush(stdout);

  ret = ioctl(mvfd, GPIOC_WRITE, 1);
  if (ret < 0)
    {
      printf("GPIOC_WRITE ioctl failed: %d\n", errno);
      errval = 2;
      goto errout_with_mgfd;
    }

  sleep(1);

  ret = ioctl(mgfd, GPIOC_WRITE, 0);
  if (ret < 0)
    {
      printf("GPIOC_WRITE ioctl failed: %d\n", errno);
      errval = 2;
      goto errout_with_mgfd;
    }

  sleep(1);

  printf("Opening the TIM device: %s\n", CONFIG_MAGNET_LX_DEVNAME);
  timfd = open(CONFIG_MAGNET_LX_DEVNAME, O_RDONLY);
  if (timfd < 0)
    {
      printf("open %s failed: %d\n", CONFIG_MAGNET_LX_DEVNAME, errno);
      errval = 2;
      goto errout_with_mgfd;
    }

  readsize = sizeof(struct adc_msg_s);
  nbytes = read(timfd, &sample, readsize);
  if (nbytes < 0)
    {
      errval = errno;
      if (errval != EINTR)
        {
          printf("read %s failed: %d\n", CONFIG_MAGNET_LX_DEVNAME, errval);
          errval = 3;
          goto errout_with_timfd;
        }

      printf("Interrupted read...\n");
    }
  else if (nbytes == 0)
    {
      printf("No data read, Ignoring\n");
    }
  else
    {
      printf("channel: %d value: %d\n", sample.am_channel, sample.am_data);
    }
  fflush(stdout);

  errval = OK;

  /* Error exits */

errout_with_timfd:
  close(timfd);
errout_with_mgfd:
  close(mgfd);
errout_with_mvfd:
  close(mvfd);

errout:
  printf("Terminating!\n");
  fflush(stdout);
  return errval;
}
#endif
