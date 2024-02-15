/* nvme_pci.c
 *
 * 2023-01-22 Shlomo Pongratz
 *
 * NVME_PCI debug registers interface.
 *
 * This tool provides a debug interface for reading and writing
 * to PCI registers via the device base address registers (BARs).
 * The tool uses the PCI resource nodes automatically created
 * by recently Linux kernels.
 *
 * The readline library is used for the command line interface
 * so that up-arrow command recall works. Command-line history
 * is not implemented. Use -lreadline -lcurses when building.
 *
 * ----------------------------------------------------------------
 */
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <unistd.h>
#include <byteswap.h>
#include <unistd.h>
#include <stdint.h>

//~ /* From QEMU include/block/nvme.h*/
enum NvmeCapShift {
    CAP_MQES_SHIFT     = 0,
    CAP_CQR_SHIFT      = 16,
    CAP_AMS_SHIFT      = 17,
    CAP_TO_SHIFT       = 24,
    CAP_DSTRD_SHIFT    = 32,
    CAP_NSSRS_SHIFT    = 36,
    CAP_CSS_SHIFT      = 37,
    CAP_BPS_SHIFT      = 45,
    CAP_MPSMIN_SHIFT   = 48,
    CAP_MPSMAX_SHIFT   = 52,
    CAP_PMRS_SHIFT     = 56,
    CAP_CMBS_SHIFT     = 57,
};

enum NvmeCapMask {
    CAP_MQES_MASK      = 0xffff,
    CAP_CQR_MASK       = 0x1, 
    CAP_AMS_MASK       = 0x3, 
    CAP_TO_MASK        = 0xff,
    CAP_DSTRD_MASK     = 0xf, 
    CAP_NSSRS_MASK     = 0x1, 
    CAP_CSS_MASK       = 0xff,
    CAP_BPS_MASK       = 0x1,
    CAP_MPSMIN_MASK    = 0xf, 
    CAP_MPSMAX_MASK    = 0xf, 
    CAP_PMRS_MASK      = 0x1, 
    CAP_CMBS_MASK      = 0x1, 
};

#define NVME_CAP_MQES(cap)  (((cap) >> CAP_MQES_SHIFT)   & CAP_MQES_MASK)
#define NVME_CAP_CQR(cap)   (((cap) >> CAP_CQR_SHIFT)    & CAP_CQR_MASK)
#define NVME_CAP_AMS(cap)   (((cap) >> CAP_AMS_SHIFT)    & CAP_AMS_MASK)
#define NVME_CAP_TO(cap)    (((cap) >> CAP_TO_SHIFT)     & CAP_TO_MASK)
#define NVME_CAP_DSTRD(cap) (((cap) >> CAP_DSTRD_SHIFT)  & CAP_DSTRD_MASK)
#define NVME_CAP_NSSRS(cap) (((cap) >> CAP_NSSRS_SHIFT)  & CAP_NSSRS_MASK)
#define NVME_CAP_CSS(cap)   (((cap) >> CAP_CSS_SHIFT)    & CAP_CSS_MASK)
#define NVME_CAP_BPS(cap)   (((cap) >> CAP_BPS_SHIFT)    & CAP_BPS_MASK)
#define NVME_CAP_MPSMIN(cap)(((cap) >> CAP_MPSMIN_SHIFT) & CAP_MPSMIN_MASK)
#define NVME_CAP_MPSMAX(cap)(((cap) >> CAP_MPSMAX_SHIFT) & CAP_MPSMAX_MASK)
#define NVME_CAP_PMRS(cap)  (((cap) >> CAP_PMRS_SHIFT)   & CAP_PMRS_MASK)
#define NVME_CAP_CMBS(cap)  (((cap) >> CAP_CMBS_SHIFT)   & CAP_CMBS_MASK)


#define SQ0TDBL_OFFSET 0x1000

/* PCI device */
typedef struct {
	/* Base address region */
	unsigned int bar;

	/* Slot info */
	unsigned int domain;
	unsigned int bus;
	unsigned int slot;
	unsigned int function;

	/* Resource filename */
	char         filename[100];

	/* File descriptor of the resource */
	int          fd;

	/* Memory mapped resource */
	unsigned char *maddr;
	unsigned int   size;
	unsigned int   offset;

	/* PCI physical address */
	unsigned int   phys;

	/* Address to pass to read/write (includes offset) */
	void *addr;
} device_t;

/* spdk/xnvme/include/libxnvme_spec.h */
struct nvme_ctrlr_bar {
	uint64_t cap; 
	uint32_t vs;
	uint32_t intms;
	uint32_t intmc;
	uint32_t cc;
	uint32_t rsvd24;
	uint32_t csts;
	uint32_t nssr;
	uint32_t aqa; 
	uint64_t asq; 
	uint64_t acq; 
	uint32_t cmbloc;
	uint32_t cmbsz;
	uint32_t bpinfo;
	uint32_t bprsel;
	uint64_t bpmbl;
	uint64_t cmbmsc;
	uint32_t cmbsts;
	uint8_t rsvd92[3492];
	uint32_t pmrcap;
	uint32_t pmrctl;
	uint32_t pmrsts;
	uint32_t pmrebs;
	uint32_t pmrswtp;
	uint32_t pmrmscl;
	uint32_t pmrmscu;
	uint8_t css[484];
} __attribute__((__packed__));


/* Usage */
static void show_usage(void)
{
	printf("\nUsage: pci_debug -s <device>\n"\
		 "  -h            Help (this message)\n"\
		 "  -s <device>   Slot/device (as per lspci)\n");
}

void print_cap(uint64_t cap)
{
	printf("\tMQES   0x%04lx  Maximum Queue Entries Supported %lu\n", NVME_CAP_MQES(cap), NVME_CAP_MQES(cap));
	printf("\tCQR    0x%04lx  Contiguous Queues Required      %lu\n", NVME_CAP_CQR(cap), NVME_CAP_CQR(cap));
	printf("\tAMS    0x%04lx  Arbitration Mechanism Supported %lu\n", NVME_CAP_AMS(cap), NVME_CAP_AMS(cap));
	printf("\tTO     0x%04lx  Timeout                         %lu\n", NVME_CAP_TO(cap), NVME_CAP_TO(cap));
	printf("\tDSTRD  0x%04lx  Doorbell Stride                 %lu\n", NVME_CAP_DSTRD(cap), 1l << (NVME_CAP_DSTRD(cap) + 2));
	printf("\tNSSRS  0x%04lx  NVM Subsystem Reset Supported   %lu\n", NVME_CAP_NSSRS(cap), NVME_CAP_NSSRS(cap));
	printf("\tCSS    0x%04lx  Command Sets Supported          %lu\n", NVME_CAP_CSS(cap), NVME_CAP_CSS(cap));
	printf("\tBPS    0x%04lx  Boot Partition Support          %lu\n", NVME_CAP_BPS(cap), NVME_CAP_BPS(cap));
	printf("\tMPSMIN 0x%04lx  Memory Page Size Minimum        %lu\n", NVME_CAP_MPSMIN(cap), NVME_CAP_MPSMIN(cap));
	printf("\tMPSMAX 0x%04lx  Memory Page Size Maximum        %lu\n", NVME_CAP_MPSMAX(cap), NVME_CAP_MPSMAX(cap));
	printf("\tPMRS   0x%04lx  Persistemt Memory Region        %lu\n", NVME_CAP_PMRS(cap), NVME_CAP_PMRS(cap));
	printf("\tCMBS   0x%04lx  Controller Memory Buffers       %lu\n", NVME_CAP_CMBS(cap), NVME_CAP_CMBS(cap));
}

void print_doorbels(device_t *dev, unsigned long stride)
{
	int i;

	for (i = 0; i < 1/* num_queues */; ++i) {
		int j;
		uint32_t *SQ0TDBL = dev->addr + SQ0TDBL_OFFSET;
		printf("QID Submission Queue %d Tail\n", i);
		for (j = 0; j < stride; j++) {
			printf("0x%08x\n", *SQ0TDBL++);
		}
		printf("QID Completion Queue %d Head\n",i);
		for (j = 0; j < stride; j++) {
			printf("0x%08x\n", *SQ0TDBL++);
		}
	}
}

void parse_controller(device_t *dev)
{
	struct nvme_ctrlr_bar *regs = dev->addr;
	unsigned long number_of_processors = sysconf(_SC_NPROCESSORS_ONLN);
	unsigned long num_queues;
	unsigned long stride;

	printf("CAP    - Capabilities                              0x%0lX\n", regs->cap);
	print_cap(regs->cap);
	printf("VS     - Version                                   0x%x - %u\n", regs->vs, regs->vs);
	printf("INTMS  - Interrupt Mask Set                        0x0%X\n", regs->intms);
	printf("INTMC  - Interrupt Mask Clear                      0x0%X\n", regs->intmc);
	printf("CC     - Controller Configuration                  0x%0X\n", regs->cc);
	printf("CSTS   - Controller Status                         0x%0X\n", regs->csts);
	printf("NSSR   - NVM Subsystem Reset (Optionsl)            0x%0X\n", regs->nssr);
	printf("AQA    - Admin Queue Attributes                    0x%0X\n", regs->aqa);
	printf("ASQ    - Admin Queue Subission Queue Base Address  0x%0lX\n", regs->asq);
	printf("ACQ    - Admin Queue Completion Queue Base Address 0x%0lX\n", regs->acq);
	printf("CMBLOC - Controller Memory Buffer Location         0x%0X (Optional)\n", regs->cmbloc);
	printf("CMBSZ  - Controller Memory Buffer Size             0x%0X (Optional)\n", regs->cmbsz);
	printf("BPINFO - Boot Partition Informastion               0x%0X (Optional)\n", regs->bpinfo);
	printf("BPRSEL - Boot Partition Read Select                0x%X (Optional)\n", regs->bprsel);
	printf("BPMBL  - Boot Partition Memory Buffer Location     0x%lX (Optional)\n", regs->bpmbl);

	/* Maximum Queue Entries Supported (MQES): This field indicates the 
	 * maximum individual queue size that the controller supports. For NVMe over PCIe 
	 * implementations, this value applies to the I/O Submission Queues and I/O 
	 * Completion Queues that the host creates. For NVMe over Fabrics 
	 * implementations, this value applies to only the I/O Submission Queues that the 
	 * host creates. This is a 0's based value.
	 * The minimum value is 1h, indicating two  entries.
	 */ 
	num_queues = NVME_CAP_MQES(regs->cap);
	printf("Number of Supported Queues %lu\n", num_queues);
	printf("Number of cores %lu\n", number_of_processors);

	num_queues = number_of_processors < num_queues ? number_of_processors : num_queues;
	num_queues++; // Add addmin queue

	stride = 1 << (NVME_CAP_DSTRD(regs->cap) + 2);
	printf("Stride %lu\n", stride);

	print_doorbels(dev, stride);
}

void display_help(device_t *dev)
{
	printf("\n");
	printf("  ?                         Help\n");
	printf("  d[width] addr len         Display memory starting from addr\n");
	printf("                            [width]\n");
	printf("                              8   - 8-bit access\n");
	printf("                              16  - 16-bit access\n");
	printf("                              32  - 32-bit access (default)\n");
	printf("  c[width] addr val         Change memory at addr to val\n");
	printf("  e                         Print the endian access mode\n");
	printf("  e[mode]                   Change the endian access mode\n");
	printf("                            [mode]\n");
	printf("                              b - big-endian (default)\n");
	printf("                              l - little-endian\n");
	printf("  f[width] addr val len inc  Fill memory\n");
	printf("                              addr - start address\n");
	printf("                              val  - start value\n");
	printf("                              len  - length (in bytes)\n");
	printf("                              inc  - increment (defaults to 1)\n");
	printf("  q                          Quit\n");
	printf("\n  Notes:\n");
	printf("    1. addr, len, and val are interpreted as hex values\n");
	printf("       addresses are always byte based\n");
	printf("\n");
}

int main(int argc, char *argv[])
{
	int opt;
	char *slot = 0;
	int status;
	struct stat statbuf;
	device_t device;
	device_t *dev = &device;

	/* Clear the structure fields */
	memset(dev, 0, sizeof(device_t));

	dev->bar = 0;

	while ((opt = getopt(argc, argv, "hs:")) != -1) {
		switch (opt) {
		case 'h':
			/* Display help */
			display_help(dev);
			return 0;
		case 's':
			slot = optarg;
			break;
		default:
			show_usage();
			return -1;
		}
	}
	if (slot == 0) {
		show_usage();
		return -1;
	}

	/* ------------------------------------------------------------
	 * Open and map the PCI region
	 * ------------------------------------------------------------
	 */

	/* Extract the PCI parameters from the slot string */
	status = sscanf(slot, "%2x:%2x.%1x",
			&dev->bus, &dev->slot, &dev->function);
	if (status != 3) {
		printf("Error parsing slot information!\n");
		show_usage();
		return -1;
	}

	/* Convert to a sysfs resource filename and open the resource */
	snprintf(dev->filename, 99, "/sys/bus/pci/devices/%04x:%02x:%02x.%1x/resource%d",
			dev->domain, dev->bus, dev->slot, dev->function, dev->bar);
	dev->fd = open(dev->filename, O_RDWR | O_SYNC);
	if (dev->fd < 0) {
		printf("Open failed for file '%s': errno %d, %s\n",
			dev->filename, errno, strerror(errno));
		return -1;
	}

	/* PCI memory size */
	status = fstat(dev->fd, &statbuf);
	if (status < 0) {
		printf("fstat() failed: errno %d, %s\n",
			errno, strerror(errno));
		return -1;
	}
	dev->size = statbuf.st_size;

	/* Map */
	dev->maddr = (unsigned char *)mmap(
		NULL,
		(size_t)(dev->size),
		PROT_READ|PROT_WRITE,
		MAP_SHARED,
		dev->fd,
		0);
	if (dev->maddr == (unsigned char *)MAP_FAILED) {
//		printf("failed (mmap returned MAP_FAILED)\n");
		printf("BARs that are I/O ports are not supported by this tool\n");
		dev->maddr = 0;
		close(dev->fd);
		return -1;
	}

	/* Device regions smaller than a 4k page in size can be offset
	 * relative to the mapped base address. The offset is
	 * the physical address modulo 4k
	 */
	{
		char configname[100];
		int fd;

		snprintf(configname, 99, "/sys/bus/pci/devices/%04x:%02x:%02x.%1x/config",
				dev->domain, dev->bus, dev->slot, dev->function);
		fd = open(configname, O_RDWR | O_SYNC);
		if (dev->fd < 0) {
			printf("Open failed for file '%s': errno %d, %s\n",
				configname, errno, strerror(errno));
			return -1;
		}

		status = lseek(fd, 0x10 + 4*dev->bar, SEEK_SET);
		if (status < 0) {
			printf("Error: configuration space lseek failed\n");
			close(fd);
			return -1;
		}
		status = read(fd, &dev->phys, 4);
		if (status < 0) {
			printf("Error: configuration space read failed\n");
			close(fd);
			return -1;
		}
		dev->offset = ((dev->phys & 0xFFFFFFF0) % 0x1000);
		dev->addr = dev->maddr + dev->offset;
		close(fd);
	}


	/* ------------------------------------------------------------
	 * Tests
	 * ------------------------------------------------------------
	 */

	printf("\n");
	printf("PCI debug\n");
	printf("---------\n\n");
	printf(" - accessing BAR%d\n", dev->bar);
	printf(" - region size is %d-bytes\n", dev->size);
	printf(" - offset into region is %d-bytes\n", dev->offset);

	/* Process commands */
	parse_controller(dev);

	/* Cleanly shutdown */
	munmap(dev->maddr, dev->size);
	close(dev->fd);
	return 0;
}
