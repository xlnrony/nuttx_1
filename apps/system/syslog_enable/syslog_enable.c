#include <nuttx/config.h>

#include <unistd.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <syslog.h>

int syslog_enable_main(int argc, char **argv)
{
  /* Parse command line arguments */
  if (argc != 2)
  	{
		fprintf(stderr, "ERROR: There must be two arguments\n");
		return 0;
  	}

  if (strcmp(argv[1], "on") == 0)
	{
		syslog_enable(true);
	}
  else if (strcmp(argv[1], "off") == 0)
	{
		syslog_enable(false);
	}
  else
	{
		fprintf(stderr, "ERROR: Parameter only be on or off\n");
	}
  
  return 0;
}
