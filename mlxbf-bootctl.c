/*
 * Copyright (c) 2017-2018, Mellanox Technologies Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#define _GNU_SOURCE  // asprintf
#include <errno.h>
#include <getopt.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <linux/limits.h>
#include <linux/major.h>

/* Boot FIFO constants */
#define BOOT_FIFO_ADDR 0x0408
#define SEGMENT_HEADER_LEN 8
#define MAX_SEG_LEN ((1 << 20) - SEGMENT_HEADER_LEN)
#define SEGMENT_IS_END (1UL << 63)

void die(const char* fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  vfprintf(stderr, fmt, ap);
  putc('\n', stderr);
  exit(1);
}

#ifndef OUTPUT_ONLY

#include <linux/mmc/ioctl.h>

/*
 * The Linux MMC driver doesn't export its ioctl command values, so we
 * copy them from include/linux/mmc/mmc.h and include/linux/mmc/core.h.
 */
#define MMC_SWITCH                6   /* ac   [31:0] See below   R1b */
#define MMC_SEND_EXT_CSD          8   /* adtc                    R1  */
#define MMC_RSP_PRESENT	(1 << 0)
#define MMC_RSP_CRC	(1 << 2)		/* expect valid crc */
#define MMC_RSP_BUSY	(1 << 3)		/* card may send busy */
#define MMC_RSP_OPCODE	(1 << 4)		/* response contains opcode */
#define MMC_RSP_SPI_S1	(1 << 7)		/* one status byte */
#define MMC_RSP_SPI_BUSY (1 << 10)		/* card may send busy */
#define MMC_RSP_SPI_R1	(MMC_RSP_SPI_S1)
#define MMC_RSP_R1	(MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE)
#define MMC_RSP_R1B	(MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE|MMC_RSP_BUSY)
#define MMC_RSP_SPI_R1B	(MMC_RSP_SPI_S1|MMC_RSP_SPI_BUSY)
#define MMC_CMD_AC	(0 << 5)
#define MMC_CMD_ADTC	(1 << 5)
#define MMC_SWITCH_MODE_WRITE_BYTE	0x03	/* Set target to value */
#define EXT_CSD_CMD_SET_NORMAL		(1<<0)

/* EXT_CSD register offset. */
#define EXT_CSD_RST_N			162	/* R/W */
#define EXT_CSD_BOOT_BUS_WIDTH		177	/* R/W */
#define EXT_CSD_PART_CONFIG		179	/* R/W */
#define EXT_CSD_BOOT_SIZE_MULT		226	/* R/W */

/* BOOT_BUS_WIDTH register definition. */
#define EXT_CSD_BOOT_BUS_WIDTH_MASK_ALL	0x7
#define EXT_CSD_BOOT_BUS_WIDTH_MASK	0x3
#define EXT_CSD_BOOT_BUS_WIDTH_RESET_MASK	0x4
#define EXT_CSD_BOOT_BUS_WIDTH_X8	0x6

/* EXT_CSD_RST_N register definition. */
#define EXT_CSD_RST_N_MASK		0x3
#define EXT_CSD_RST_N_ENABLE		0x1

/* Program constants */
#define EMMC_MIN_BOOT_SIZE 0x20000
#define EMMC_BLOCK_SIZE 512

#define SYS_PATH1 "/sys/bus/platform/drivers/mlx-bootctl"
#define SYS_PATH2 "/sys/bus/platform/devices/MLNXBF04:00"
#define SECOND_RESET_ACTION_PATH "second_reset_action"
#define POST_RESET_WDOG_PATH "post_reset_wdog"
#define LIFECYCLE_STATE_PATH "lifecycle_state"
#define SECURE_BOOT_FUSE_STATE_PATH "secure_boot_fuse_state"

/* Program variables */
const char *mmc_path = "/dev/mmcblk0";

/* Run an MMC_IOC_CMD ioctl on mmc_path */
void mmc_command(struct mmc_ioc_cmd *idata)
{
  static int mmc_fd = -1;
  if (mmc_fd < 0)
  {
    mmc_fd = open(mmc_path, O_RDWR);
    if (mmc_fd < 0)
      die("%s: %m", mmc_path);
  }
  if (ioctl(mmc_fd, MMC_IOC_CMD, idata) < 0)
    die("%s: mmc ioctl: %m", mmc_path);
}

uint8_t* get_ext_csd(void)
{
  static uint8_t ext_csd[EMMC_BLOCK_SIZE]
    __attribute__((aligned(EMMC_BLOCK_SIZE))) = { 0 };
  struct mmc_ioc_cmd idata = {
    .write_flag = 0,
    .opcode = MMC_SEND_EXT_CSD,
    .arg = 0,
    .flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC,
    .blksz = EMMC_BLOCK_SIZE,
    .blocks = 1
  };

  mmc_ioc_cmd_set_data(idata, ext_csd);
  mmc_command(&idata);

  return ext_csd;
}

/* Return the current partition (0 or 1) that we will boot from. */
int get_boot_partition(void)
{
  uint8_t *ext_csd = get_ext_csd();
  int part = ((ext_csd[EXT_CSD_PART_CONFIG] >> 3) & 0x7) - 1;

  /* Set part to 0 by default if it is -1 (boot disabled). */
  if (part < 0)
    part = 0;

  return part;
}

/* Set which partition to boot from. */
void set_boot_partition(int part)
{
  int value = ((part + 1) & 0x7) << 3;  /* Adjust for 1-based numbering */
  struct mmc_ioc_cmd idata = {
    .write_flag = 1,
    .opcode = MMC_SWITCH,
    .arg = ((MMC_SWITCH_MODE_WRITE_BYTE << 24) |
            (EXT_CSD_PART_CONFIG << 16) |
            (value << 8) |
            EXT_CSD_CMD_SET_NORMAL),
    .flags = MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC
  };

  mmc_command(&idata);
}

/* Get the boot partition size */
uint64_t get_boot_partition_size(void)
{
  uint64_t part_size;
  uint8_t *ext_csd = get_ext_csd();
  part_size = (ext_csd[EXT_CSD_BOOT_SIZE_MULT]) *
                  EMMC_MIN_BOOT_SIZE;

  return part_size;
}

/* Return the current boot bus width. */
int get_boot_bus_width(void)
{
  uint8_t *ext_csd = get_ext_csd();
  return ext_csd[EXT_CSD_BOOT_BUS_WIDTH] & EXT_CSD_BOOT_BUS_WIDTH_MASK_ALL;
}

/* Set the boot-bus-width to 8-bit mode in ECSD. */
void set_boot_bus_width(void)
{
  uint8_t *ext_csd = get_ext_csd();
  int value = (ext_csd[EXT_CSD_BOOT_BUS_WIDTH] &
    ~EXT_CSD_BOOT_BUS_WIDTH_MASK_ALL) | EXT_CSD_BOOT_BUS_WIDTH_X8;
  struct mmc_ioc_cmd idata = {
    .write_flag = 1,
    .opcode = MMC_SWITCH,
    .arg = ((MMC_SWITCH_MODE_WRITE_BYTE << 24) |
            (EXT_CSD_BOOT_BUS_WIDTH << 16) |
            (value << 8) |
            EXT_CSD_CMD_SET_NORMAL),
    .flags = MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC
  };

  mmc_command(&idata);
}

void enable_rst_n(void)
{
  uint8_t *ext_csd = get_ext_csd();
  int value = (ext_csd[EXT_CSD_RST_N] & ~EXT_CSD_RST_N_MASK) |
    EXT_CSD_RST_N_ENABLE;
  struct mmc_ioc_cmd idata = {
    .write_flag = 1,
    .opcode = MMC_SWITCH,
    .arg = ((MMC_SWITCH_MODE_WRITE_BYTE << 24) |
            (EXT_CSD_RST_N << 16) |
            (value << 8) |
            EXT_CSD_CMD_SET_NORMAL),
    .flags = MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC
  };

  mmc_command(&idata);
}

FILE *open_sysfs(const char *name, const char *attr)
{
  FILE *f;
  char path[PATH_MAX];

  sprintf(path, "%s/%s", SYS_PATH1, name);
  f = fopen(path, attr);
  if (f == NULL) {
    sprintf(path, "%s/%s", SYS_PATH2, name);
    f = fopen(path, attr);
  }

  if (f == NULL)
    die("%s: %m", name);

  return f;
}

int get_watchdog(void)
{
  FILE *f = open_sysfs(POST_RESET_WDOG_PATH, "r");
  int watchdog;
  if (fscanf(f, "%d", &watchdog) != 1)
    die("%s: failed to read integer", POST_RESET_WDOG_PATH);
  fclose(f);
  return watchdog;
}

void set_watchdog(int interval)
{
  FILE *f = open_sysfs(POST_RESET_WDOG_PATH, "w");
  if (fprintf(f, "%d\n", interval) < 0)
    die("%s: failed to set watchdog to '%d'", POST_RESET_WDOG_PATH, interval);
  fclose(f);
}

void set_second_reset_action(const char *action)
{
  FILE *f = open_sysfs(SECOND_RESET_ACTION_PATH, "w");
  if (fprintf(f, "%s\n", action) < 0)
    die("%s: failed to set action to '%s'", SECOND_RESET_ACTION_PATH, action);
  fclose(f);
}

// Return the (malloced) string describing the lifecycle state (w line return)
char *get_lifecycle_state(void)
{
  FILE *f = open_sysfs(LIFECYCLE_STATE_PATH, "r");
  char *buf = NULL;
  size_t len = 0;

  if (getline(&buf, &len, f) == -1)
    die("%s: failed to get lifecycle state", LIFECYCLE_STATE_PATH);
  fclose(f);

  return buf;
}

// Return number of remaining free secure boot fuse versions
int get_free_sbfuse_slots(void)
{
  int free_slot = 0;
  FILE *f = open_sysfs(SECURE_BOOT_FUSE_STATE_PATH, "r");
  char *buf = NULL, *p;
  size_t len = 0;

  while (getline(&buf, &len, f) != -1)
  {
    p = buf;
    while ((p = strstr(p, "Free")) != NULL) {
      free_slot++;
      p += strlen("Free");
    }
  }
  free(buf);
  fclose(f);

  return free_slot;
}

void show_status(void)
{
  // Display the default boot partition
  int part = get_boot_partition();
  printf("primary: %sboot%d\n", mmc_path, part);
  printf("backup: %sboot%d\n", mmc_path, part ^ 1);

  // Display the boot bus width setting
  int boot_bus_width = get_boot_bus_width();
  printf("boot-bus-width: x%d\n",
    (boot_bus_width & EXT_CSD_BOOT_BUS_WIDTH_MASK) ?
    (boot_bus_width & EXT_CSD_BOOT_BUS_WIDTH_MASK) * 4 : 1);
  printf("reset to x1 after reboot: %s\n",
    (boot_bus_width & EXT_CSD_BOOT_BUS_WIDTH_RESET_MASK)? "FALSE" : "TRUE");

  // Display the watchdog value
  int watchdog = get_watchdog();
  printf("watchdog-swap: ");
  if (watchdog == 0)
    printf("disabled\n");
  else
    printf("%d\n", watchdog);

  // Display the secure boot fuse states
  char *lifecycle_state = get_lifecycle_state();
  printf("lifecycle state: %s", lifecycle_state);
  free(lifecycle_state);
  printf("secure boot key free slots: %d\n", get_free_sbfuse_slots());
}

#endif  // OUTPUT_ONLY

// Read as much as possible despite EINTR or partial reads, and die on error.
ssize_t read_or_die(const char* filename, int fd, void* buf, size_t count)
{
  ssize_t n = 0;
  while (count > 0)
  {
    ssize_t rc = read(fd, buf, count);
    if (rc < 0)
    {
      if (errno == EINTR)
        continue;
      die("%s: can't read: %m", filename);
    }
    if (rc == 0)
      break;

    n += rc;
    buf += rc;
    count -= rc;
  }
  return n;
}

// Write everything passed in despite EINTR or partial reads, and die on error.
ssize_t write_or_die(const char* filename,
                     int fd, const void* buf, size_t count)
{
  ssize_t n = count;
  while (count > 0)
  {
    ssize_t rc = write(fd, buf, count);
    if (rc < 0)
    {
      if (errno == EINTR)
        continue;
      die("%s: can't write: %m", filename);
    }
    if (rc == 0)
      die("%s: write returned zero", filename);

    buf += rc;
    count -= rc;
  }
  return n;
}

/*
 * Generate the boot stream segment header.
 *
 * is_end: 1 if this segment is the last segment of the boot code, else 0.
 * channel: The channel number to write to.
 * address: The register address to write to.
 * length: The length of this segment in bytes; max is MAX_SEG_LEN.
 *
 * We ignore endian issues here since if the tool is built natively,
 * this is likely correct anyway, and if built cross, we don't have a
 * way to know the endianness of the arm cores anyway.
 */
uint64_t gen_seg_header(int is_end, int channel, int address, size_t length)
{
  return (((is_end & 0x1UL) << 63) |
          ((channel & 0xfUL) << 45) |
          ((address & 0xfff8UL) << 29) |
          (((length + SEGMENT_HEADER_LEN) >> 3) & 0x1ffffUL));
}

uint64_t get_segment_length(uint64_t segheader)
{
  uint64_t length = (segheader & 0x1ffffUL) << 3;
  if (length == 0)
    length = MAX_SEG_LEN;
  else
    length -= SEGMENT_HEADER_LEN;
  return length;
}

bool validate_seg_header(uint64_t segheader)
{
  return (((segheader >> 29) & 0xfff8UL) == BOOT_FIFO_ADDR);
}

void read_bootstream(const char *bootstream, const char *bootfile,
                     uint64_t part_size)
{
  // Copy the contents of the bootfile device to a bootstream
  printf("Copy bootstream from %s to %s\n", bootfile, bootstream);
  int ifd = open(bootfile, O_RDONLY);
  if (ifd < 0)
    die("%s: %m", bootfile);
  int ofd = open(bootstream, O_WRONLY | O_CREAT | O_TRUNC, 0666);
  if (ofd < 0)
    die("%s: %m", bootstream);

  char *buf = malloc(MAX_SEG_LEN);
  if (buf == NULL)
    die("out of memory");

  uint64_t header;
  // Read and discard the header word
  read_or_die(bootfile, ifd, &header, sizeof(header));
  uint64_t segheader = 0, total_size = 0;

  while (!(segheader & SEGMENT_IS_END))
  {
    read_or_die(bootfile, ifd, &segheader, sizeof(segheader));
    if (!validate_seg_header(segheader))
      die("Invalid segment header");
    uint64_t seg_size = get_segment_length(segheader);
    read_or_die(bootfile, ifd, buf, seg_size);
    write_or_die(bootstream, ofd, buf, seg_size);
    total_size += seg_size;
    if (total_size > part_size)
      die("No valid bfb present");
  }

  if (close(ifd) < 0)
    die("%s: close: %m", bootstream);
  if (close(ofd) < 0)
    die("%s: close: %m", bootfile);
  free(buf);
}

void write_bootstream(const char *bootstream, const char *bootfile, int flags)
{
  int sysfd = -1;
  char *sysname;

  // Reset the force_ro setting if need be
  if (strncmp(bootfile, "/dev/", 5) == 0)
  {
    if (asprintf(&sysname, "/sys/block/%s/force_ro", &bootfile[5]) <= 0)
      die("unexpected failure in asprintf (%s/%d)", __FILE__, __LINE__);

    sysfd = open(sysname, O_RDWR | O_SYNC);
    if (sysfd >= 0)
    {
      char status;
      if (read_or_die(sysname, sysfd, &status, 1) != 1)
        die("%s: unexpected EOF on read", sysname);
      if (status == '1')
      {
        char disabled = '0';
        if (lseek(sysfd, 0, SEEK_SET) != 0)
          die("%s: can't seek back to start: %m", sysname);
        write_or_die(sysname, sysfd, &disabled, 1);
      }
      else
      {
        close(sysfd);
        sysfd = -1;
        free(sysname);
        sysname = NULL;
      }
    }
    else
    {
      if (errno != ENOENT)
        die("%s: open: %m", sysname);
      printf("WARNING: No matching %s for %s\n", sysname, bootfile);
      free(sysname);
      sysname = NULL;
    }
  }

  // Copy the bootstream to the bootfile device
  int ifd = open(bootstream, O_RDONLY);
  if (ifd < 0)
    die("%s: %m", bootstream);
  int ofd = open(bootfile, O_WRONLY | flags, 0666);
  if (ofd < 0)
    die("%s: %m", bootfile);
  struct stat st;
  if (fstat(ifd, &st) < 0)
    die("%s: stat: %m", bootstream);
  size_t bytes_left = st.st_size;

  char *buf = malloc(MAX_SEG_LEN);
  if (buf == NULL)
    die("out of memory");

  // Write the bootstream header word first.  This has the byte to
  // be displayed in the rev_id register as the low 8 bits (zero for now).
  uint64_t header = 0;
  write_or_die(bootfile, ofd, &header, sizeof(header));

  while (bytes_left > 0)
  {
    size_t seg_size = (bytes_left <= MAX_SEG_LEN) ? bytes_left : MAX_SEG_LEN;
    bytes_left -= seg_size;

    // Generate the segment header.
    size_t pad_size = seg_size % 8 ? (8 - seg_size % 8) : 0;
    uint64_t segheader = gen_seg_header(bytes_left == 0, 1, BOOT_FIFO_ADDR,
                                        seg_size + pad_size);
    write_or_die(bootfile, ofd, &segheader, sizeof(segheader));

    // Copy the segment plus any padding.
    read_or_die(bootstream, ifd, buf, seg_size);
    memset(buf + seg_size, 0, pad_size);
    write_or_die(bootfile, ofd, buf, seg_size + pad_size);
  }

  if (close(ifd) < 0)
    die("%s: close: %m", bootstream);
  if (close(ofd) < 0)
    die("%s: close: %m", bootfile);
  free(buf);

  // Put back the force_ro setting if need be
  if (sysfd >= 0)
  {
    if (lseek(sysfd, 0, SEEK_SET) != 0)
      die("%s: can't seek back to start: %m", sysname);
    char enabled = '1';
    write_or_die(sysname, sysfd, &enabled, 1);
    close(sysfd);
    free(sysname);
  }
}

#ifdef OUTPUT_ONLY

int main(int argc, char **argv)
{
  static struct option long_options[] = {
    { "bootstream", required_argument, NULL, 'b' },
    { "output", required_argument, NULL, 'o' },
    { "help", no_argument, NULL, 'h' },
    { NULL, 0, NULL, 0 }
  };
  static const char short_options[] = "b:o:h";
  static const char help_text[] =
   "syntax: mlx-bootctl [--help|-h] --bootstream|-b BFBFILE --output|-o OUTPUT";

  const char *bootstream = NULL;
  const char *output_file = NULL;
  int opt;

  while ((opt = getopt_long(argc, argv, short_options, long_options, NULL))
         != -1)
  {
    switch (opt)
    {
    case 'b':
      bootstream = optarg;
      break;

    case 'o':
      output_file = optarg;
      break;

    case 'h':
    default:
      die(help_text);
      break;
    }
  }

  if (bootstream == NULL || output_file == NULL)
    die("mlx-bootctl: Must specify --output and --bootstream");

  write_bootstream(bootstream, output_file, O_CREAT | O_TRUNC);
  return 0;
}

#else

int main(int argc, char **argv)
{
  static struct option long_options[] = {
    { "swap", no_argument, NULL, 's' },
    { "watchdog-swap", required_argument, NULL, 'w' },
    { "nowatchdog-swap", no_argument, NULL, 'n' },
    { "bootstream", required_argument, NULL, 'b' },
    { "overwrite-current", no_argument, NULL, 'c' },
    { "device", required_argument, NULL, 'd' },
    { "output", required_argument, NULL, 'o' },
    { "read", required_argument, NULL, 'r' },
    { "help", no_argument, NULL, 'h' },
    { NULL, 0, NULL, 0 }
  };
  static const char short_options[] = "sb:d:o:r:he";
  static const char help_text[] =
    "syntax: mlxbf-bootctl [--help|-h] [--swap|-s] [--device|-d MMCFILE]\n"
    "                      [--output|-o OUTPUT] [--read|-r INPUT]\n"
    "                      [--bootstream|-b BFBFILE] [--overwrite-current]\n"
    "                      [--watchdog-swap interval | --nowatchdog-swap]";

  const char *watchdog_swap = NULL;
  const char *bootstream = NULL;
  const char *output_file = NULL;
  const char *input_file = NULL;
  bool watchdog_disable = false;
  bool swap = false;
  int which_boot = 1;   // alternate boot partition by default
  int opt;

  while ((opt = getopt_long(argc, argv, short_options, long_options, NULL))
         != -1)
  {
    switch (opt)
    {
    case 's':
      swap = true;
      break;

    case 'w':
      watchdog_swap = optarg;
      watchdog_disable = false;
      break;

    case 'n':
      watchdog_swap = NULL;
      watchdog_disable = true;
      break;

    case 'b':
      bootstream = optarg;
      break;

    case 'c':
      which_boot = 0;    // overwrite current boot partition (dangerous)
      break;

    case 'd':
      mmc_path = optarg;
      break;

    case 'o':
      output_file = optarg;
      break;

    case 'r':
      input_file = optarg;
      break;

    case 'e':
      enable_rst_n();
      break;

    case 'h':
    default:
      die(help_text);
      break;
    }
  }

  if (!bootstream && !swap && watchdog_swap == NULL && !watchdog_disable)
  {
    show_status();
    return 0;
  }

  if (bootstream)
  {
    if (input_file)
    {
      uint64_t boot_part_size = get_boot_partition_size();
      read_bootstream(bootstream, input_file, boot_part_size);
    }
    else if (output_file)
    {
      // Write the bootstream to the given file, creating it if needed
      write_bootstream(bootstream, output_file, O_CREAT | O_TRUNC);
    }
    else
    {
      // Make sure the file will fit inside the boot partition
      uint64_t boot_part_size = get_boot_partition_size();
      struct stat st;
      stat(bootstream, &st);
      if (st.st_size > boot_part_size)
        die("Size of bootstream exceeds boot partition size");

      // Get the active partition and write to the appropriate *bootN file
      // Must save/restore boot partition, which I/O otherwise resets to zero.
      int boot_part = get_boot_partition();
      char *bootfile;
      if (asprintf(&bootfile, "%sboot%d", mmc_path, boot_part ^ which_boot) <= 0)
        die("unexpected failure in asprintf (%s/%d)", __FILE__, __LINE__);

      write_bootstream(bootstream, bootfile, O_SYNC);
      // The eMMC driver works in an asynchronous way, thus any commands sent
      // should occur after the write to the bootstream has retired. Otherwise
      // a blk_update_request I/O error would be raised and the success of the
      // command cannot be guaranteed. The check for the status of the eMMC
      // at user level is non-trival, thus a delay is added here instead.
      // @TODO: Update the delay to check for the eMMC status.
      sleep(1);
      set_boot_partition(boot_part);
      set_boot_bus_width();
      enable_rst_n();
    }
  }

  if (swap)
    set_boot_partition(get_boot_partition() ^ 1);

  if (watchdog_swap != NULL)
  {
    // Enable reset watchdog to swap eMMC on reset after watchdog interval
    char *end;
    int watchdog = strtol(watchdog_swap, &end, 0);
    if (end == watchdog_swap || *end != '\0')
      die("watchdog-swap argument ('%s') must be an integer", watchdog_swap);
    set_watchdog(watchdog);
    set_second_reset_action("swap_emmc");
    enable_rst_n();
  }

  if (watchdog_disable)
  {
    // Disable reset watchdog and don't adjust reset actions at reset time
    set_watchdog(0);
    set_second_reset_action("none");
  }

  return 0;
}

#endif // OUTPUT_ONLY
