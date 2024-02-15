#CC:=gcc
#CFLAGS = -Wall
#LDFLAGS += -L/usr/lib/x86_64-linux-gnu/ -L/lib/x86_64-linux-gnu/
#LDFLAGS += -lreadline -lcurses
INSTALL_DIR = /usr/bin/

default: nvme_pci.c
	gcc -Wall nvme_pci.c -L/usr/lib/x86_64-linux-gnu/ -L/lib/x86_64-linux-gnu/ -o nvme_pci


clean:
	rm nvme_pci

