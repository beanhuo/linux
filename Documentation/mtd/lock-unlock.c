/*
 * Copyright 2017, Micron, Inc.
 * Written by Bean Huo <beanhuo@micron.com>
 * Build a mtd device simple debug tool.
 */

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/types.h>

#define len 128

struct erase_info_user
{
  unsigned int start;
  unsigned int length;
};

struct mtd_info_user
{
  unsigned char type;
  unsigned int flags;
  unsigned int size;		/* Total size of the MTD */
  unsigned int erasesize;
  unsigned int writesize;
  unsigned int oobsize;		/* Amount of OOB data per block (e.g. 16) */
  unsigned long long padding;	/* Old obsolete field; do not use */
};
/* Get basic MTD characteristics info (better to use sysfs) */
#define MEMGETINFO 		_IOR('M', 1, struct mtd_info_user)
/* Erase segment of MTD */
#define MEMERASE 		_IOW('M', 2, struct erase_info_user)
/* Lock a chip (for MTD that supports it) */
#define MEMLOCK                 _IOW('M', 5, struct erase_info_user)
/* Unlock a chip (for MTD that supports it) */
#define MEMUNLOCK               _IOW('M', 6, struct erase_info_user)

int fd;
int flag = 0;
char *mtd_d = "/dev/mtd0";
int val = 0;
char pbuf[len];

struct mtd_info_user mtd;
struct erase_info_user erase_info;

char data[len] = {
  0x11, 0x22, 0x11, 0x22, 0x11, 0x22, 0x11, 0x22, 0x11, 0x22, 0x11, 0x22,
  0x11, 0x22,
  0x11, 0x22, 0x11, 0x22, 0x11, 0x22, 0x11, 0x22, 0x11, 0x22, 0x11, 0x22,
  0x11, 0x22,
  0x11, 0x22, 0x11, 0x22, 0x11, 0x22, 0x11, 0x22, 0x11, 0x22, 0x11, 0x22,
  0x11, 0x22,
  0x11, 0x22, 0x11, 0x22, 0x11, 0x22, 0x11, 0x22, 0x11, 0x22, 0x11, 0x22,
  0x11, 0x22,
  0x11, 0x22, 0x11, 0x22, 0x11, 0x22, 0x11, 0x22, 0x11, 0x22, 0x11, 0x22,
  0x11, 0x22,
  0x11, 0x22, 0x11, 0x22, 0x11, 0x22, 0x11, 0x22, 0x11, 0x22, 0x11, 0x22,
  0x11, 0x22,
  0x11, 0x22, 0x11, 0x22, 0x11, 0x22, 0x11, 0x22, 0x11, 0x22, 0x11, 0x22,
  0x11, 0x22,
  0x11, 0x22, 0x11, 0x22, 0x11, 0x22, 0x11, 0x22, 0x11, 0x22, 0x11, 0x22,
  0x11, 0x22,
  0x11, 0x22, 0x11, 0x22, 0x11, 0x22, 0x11, 0x22, 0x11, 0x22, 0x11, 0x22,
  0x11, 0x22,
  0x11, 0x22
};


void
read_mtdinfo (void)
{

  fd = open (mtd_d, O_RDWR | O_SYNC);
  if (fd < 0) {
    fprintf (stderr, "Could not open mtd device: %s\n", mtd_d);
    exit (1);
  }

  if (ioctl (fd, MEMGETINFO, &mtd)) {
    fprintf (stderr, "Could not get MTD device info from %s\n", mtd_d);
    close (fd);
    exit (1);
  }

  printf ("%s size is 0x%x, erase size 0x%x, write size 0x%x. \n", mtd_d,
	  mtd.size, mtd.erasesize, mtd.writesize);

}

void
erase_chip (void)
{

  printf ("Erasing whole mtd......\n");
  erase_info.length = mtd.erasesize;
  erase_info.start = 0;

  for (erase_info.start; erase_info.start < mtd.size;
       erase_info.start += mtd.erasesize) {
    if (ioctl (fd, MEMERASE, &erase_info)) {
      fprintf (stderr, "Failed to erase block on %s at 0x%x\n", mtd_d,
	       erase_info.start);
      close (fd);
      exit (1);
    }
  }
  printf ("Erase done!\n");
}

void
lock (void)
{
  erase_info.length = mtd.erasesize * 16;
  erase_info.start = 0;
  if (ioctl (fd, MEMLOCK, &erase_info) < 0)
    printf ("Sector 0 lock failed!\n");
  else
    printf ("Sector 0 lock done!\n");
}

void
erase (void)
{

  erase_info.length = mtd.erasesize * 16;
  erase_info.start = 0;
  if (ioctl (fd, MEMERASE, &erase_info))
    printf ("erase failed!\n");
  else {
    printf ("erase succeeded!\n");
  }
}

void
program (void)
{

  lseek (fd, 0, SEEK_SET);
  val = write (fd, data, len);
  if (val < 0 || val < len)
    printf ("write data failed!\n");
  else {
    printf ("write data succeeded!\n");
  }
}

void
read_data (void)
{

  memset (pbuf, 0x00, len);
  lseek (fd, 0, SEEK_SET);
  if (read (fd, pbuf, len) != len) {
    printf ("read() failed\n");
  }
  else {
    int i;
    printf ("programmed data:\n");
    for (i = 0; i < len; i++) {
      if (!(i + 1) % 10)
	printf ("\n");
      printf ("0x%x ", pbuf[i]);

    }
  }

}

void
unlock (void)
{
  erase_info.length = mtd.erasesize * 16;
  erase_info.start = 0;

  printf ("sector 0 unlocked!\n");
  if (ioctl (fd, MEMUNLOCK, &erase_info) < 0)
    printf ("sector 0 unlock failed\n");
  else
    printf ("sector 0 unlock done\n");

}

int
main (int argc, char *argv[])
{
  read_mtdinfo ();
  switch ((int) argv[1][0]) {
  case 'e':
    erase ();
    break;
  case 'l':
    lock ();
    break;
  case 'u':
    unlock ();
    break;
  case 'w':
    program ();
    break;
  case 'r':
    read_data ();
    break;
  default:
    printf ("Wrong input!\n");
    break;
  }
  exit (0);
  close (fd);
  return 0;
}
