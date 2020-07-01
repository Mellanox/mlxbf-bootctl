mlxbf-bootctl
=============

Background
----------

The BlueField boot flow comprises 4 main phases:

- Hardware loads ARM Trusted Firmware (ATF)
- ATF loads Unified Extensible Firmware Interface (UEFI); together ATF
  and UEFI make up the booter software
- UEFI loads the operating system, such as the Linux kernel
- The operating system loads applications and user data

When booting from eMMC, these stages make use of two different types
of storage within the eMMC part:

- ATF and UEFI come from a special area known as an eMMC boot
  partition.  Data from a boot partition is automatically streamed
  from the eMMC device to the eMMC controller under hardware control
  during the initial bootup.  Each eMMC device has two boot
  partitions, and the partition that is used to stream the boot data
  is chosen by a non-volatile configuration register in the eMMC.

- The operating system, applications, and user data come from the
  remainder of the chip, known as the user area.  This area is
  accessed via block-size reads and writes, done by a device driver or
  similar software routine.

In most deployments, BlueField's ARM cores are expected to obtain
their software stack from an on-board Embedded Multi-Media Card (eMMC)
device.  Even in environments where the final OS kernel is not kept on
eMMC -- for instance, systems which boot over a network -- the initial
booter code will still come from eMMC.

Most software stacks need to be modified or upgraded at some point in
their lifetime.  Doing this on a fielded BlueField system involves
some risk; if there's a severe problem with the new software, system
operation may be impaired to the extent that it is impossible to do
further modifications to fix the problem.  Ideally, one would
like to be able to install a new software version on a BlueField
system, try it out, and then fall back to a previous version if the
new one does not work.  In some environments, it's important that this
fallback operation happen automatically, since there may be no
physical access to the system.  In others, there may be an external
agent, like a service processor, which could manage the process.


Solution Overview
-----------------

In order to satisfy the requests listed above, we do the following:

- Provide two software partitions on the eMMC, 0 and 1.  At any given
  time, one area is designated the primary partition, and the
  other the backup partition.  The primary partition is the one we'll
  boot from on the next reboot or reset.

- Allow software running on the ARM cores to declare that the primary
  partition is now the backup partition, and vice versa.  (For
  brevity, in the remainder of this document, we'll term this
  "swapping the partitions", although we're just modifying a pointer;
  the data on the partitions does not move.)

- Allow an external agent, like a service processor, to swap the
  primary and backup partitions.

- Allow software running on the ARM cores to reboot the system, while
  starting an upgrade watchdog timer.  If the upgrade watchdog
  expires, the system will reboot, but the primary and backup
  partitions will first be swapped.

The mlxbf-bootctl Program
-------------------------

Access to all the boot partition management is via a program shipped
with the BlueField software called "mlxbf-bootctl".  The binary is
shipped as part of our Yocto image (in /sbin) and the sources are
shipped in the "src" directory in the BlueField Runtime Distribution.
A simple `make` command will build the utility.

The mlxbf-bootctl syntax is as follows:

```
  syntax: mlxbf-bootctl [--help|-h] [--swap|-s] [--device|-d MMCFILE]
                        [--output|-o OUTPUT] [--read|-r INPUT]
                        [--bootstream|-b BFBFILE] [--overwrite-current]
                        [--watchdog-swap interval | --nowatchdog-swap]

  --device: Use a device other than the default /dev/mmcblk0

  --bootstream: Write the specified bootstream to the alternate
    partition of the device.  This queries the base device
    (e.g. /dev/mmcblk0) for the alternate partition, and uses that
    information to open the appropriate boot partition device,
    e.g. /dev/mmcblk0boot0.

  --overwrite-current: Used with "--bootstream" to overwrite the current
    boot partition instead of the alternate one (not recommended)

  --output: Used with "--bootstream" to specify a file to write the boot
    partition data to (creating it if necessary), rather than using an
    existing master device and deriving the boot partition device.

  --read: Read a bootstream and convert it back to a BFBFILE specified
    by --bootstream.  For example use "mlxbf-bootctl --read /dev/mmcblk0boot0
    --bootstream current.bfb" to read the current bfb installed on boot
    partition zero.

  --watchdog-swap: Arrange to start the ARM watchdog with a countdown
    of the specified number of seconds until it fires; also, set the
    boot software so that it will swap the primary and alternate
    partitions at the next reset.

  --nowatchdog-swap: Ensure that after the next reset, no watchdog
    will be started, and no swapping of boot partitions will happen.
```

Updating the Boot Partition
---------------------------

To update the boot partition on the ARM cores, let us assume we have a
new bootstream file, e.g. "bootstream.new".  We would like to install
and validate this new bootstream.  To just update and hope for the
best, you can do:

```bash
   mlxbf-bootctl --bootstream bootstream.new --swap
   reboot
```

This will write the new bootstream to the alternate boot partition,
swap alternate and primary so that the new bootstream will be used on
the next reboot, then reboot to use it.

(You can also use `--overwrite-current` instead of `--swap`, which will
just overwrite your current boot partition, but this is not
recommended as there is no easy way to recover if the new booter code
does not bring the system up.)

Safely Updating with a BMC
--------------------------

With a BMC (board management processor) you can do better.  The ARM
cores simply notify the BMC prior to the reboot that an upgrade is
about to happen.  Software running on the BMC can then be implemented
that will watch the ARM cores after reboot.  If after some time the
BMC has not seen the ARM cores come up properly (for whatever
definition is appropriate for the application in question), it can use
its USB debug connection to the ARM cores to properly reset the ARM
cores, first setting a suitable mode bit that the ARM booter will
respond to by switching the primary and alternate boot partitions as
part of resetting into its original state.

Safely Updating from the ARM Cores
----------------------------------

Without a BMC, you can use the ARM watchdog to achieve similar
results.  If something goes wrong on the next reboot and the system
does not come up properly, it will reboot and return to the original
configuration.  In this case, you might run:

```bash
  mlxbf-bootctl --bootstream bootstream.new --swap --watchdog-swap 60
  reboot
```

With these commands, you will reboot the system, and if it hangs for
60 seconds or more, the watchdog will fire and reset the chip, the
booter will swap the partitions back again to the way they were
before, and the system will reboot back with the original boot
partition data.  Similarly, if the system comes up but panics and
resets, the booter will again swap the boot partition back to the way
it was before.

You must ensure that Linux after the reboot is configured to boot up
with the `sbsa_gwdt` driver enabled.  This is the SBSA (Server Base
System Architecture) Generic WatchDog Timer.  As soon as the driver is
loaded, it will start refreshing the watchdog and preventing it from
firing, which will allow your system to finish booting up safely.  In
the example above, we allow 60 seconds from system reset until the
Linux watchdog kernel driver is loaded.  At that point, your
application may open /dev/watchdog explicitly, and the application
would then become responsible for refreshing the watchdog frequently
enough to keep the system from rebooting.

For documentation on the Linux watchdog subsystem, see the Linux
watchdog documentation:

https://www.kernel.org/doc/Documentation/watchdog/watchdog-api.txt

For example, to disable the watchdog completely, you can run:

```bash
  echo V > /dev/watchdog
```

You can incorporate other features of the ARM generic watchdog into
your application code using the programming API as well, if you wish.

Once the system has booted up, in addition to disabling or
reconfiguring the watchdog itself if you wish, you should also clear
the "swap on next reset" functionality from the booter by running:

```bash
  mlxbf-bootctl --nowatchdog-swap
```

Otherwise, next time you reset the system (via reboot, external reset,
etc) it will assume a failure or watchdog reset occurred and swap the
eMMC boot partition automatically.

The above steps can be done manually, or can be done automatically by
software running in the newly-booted system.

Changing the Kernel or Userspace
--------------------------------

The above solutions simply update the boot partition to hold new
booter software (ATF and UEFI).  If you want to also provide a new
kernel image and/or new userspace, you should partition your eMMC into
multiple partitions appropriately.  For example, you might have a
single FAT partition that UEFI can read the kernel image file from,
but the new bootstream contains a UEFI bootpath pointing to an updated
kernel image.  Similarly, you might have two Linux partitions, and
your upgrade procedure would write a new filesystem into the "idle"
Linux partition, then reboot with the bootstream holding kernel boot
arguments that direct it to boot from the previously idle partition.
The details on how exactly to do this depend on the specifics of how
and what need to be upgraded for the specific application, but in
principle any component of the system can be safely upgraded using
this type of approach.
