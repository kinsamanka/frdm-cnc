#
# !!!! Do NOT edit this makefile with an editor which replace tabs by spaces !!!!   
#
##############################################################################################
#
# On command line:
#
# make all = Create project
#
# make clean = Clean project files.
#
# To rebuild project do "make clean" and "make all".
#
# Provided originally by Hussam Al-Hertani
# Modified by Erich Styger
# Use at your own risk!!!!!
##############################################################################################
# Start of default section
#
TRGT = /home/tmp/freescale/bare-metal-arm/gcc-arm/bin/arm-none-eabi-
CC   = $(TRGT)gcc
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
S19  = $(CP) -O srec
BIN  = $(CP) -O binary -S
MCU  = cortex-m0

# List all default C defines here, like -D_DEBUG=1
DDEFS =

# List all default ASM defines here, like -D_DEBUG=1
DADEFS =

# List all default directories to look for include files here
DINCDIR =

# List the default directory to look for the libraries here
DLIBDIR =

# List all default libraries here
DLIBS =

#
# End of default section
##############################################################################################

##############################################################################################
# Start of user section
#

#
# Define project name
PROJECT = Linuxcnc

# List all user C define here, like -D_DEBUG=1
UDEFS =

# Define ASM defines here
UADEFS =

LIBSDIR    = 
LINKER = ./

# List C source files here
SRC  = ./main.c
SRC += ./kinetis_sysinit.c
SRC += ./stepgen.c

# List ASM source files here
ASRC = ./startup.S
 
# List all user directories here
UINCDIR = ./

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS =

# Define optimisation level here
OPT = -O3
#
# End of user defines
##############################################################################################
#
# Define linker script file here
#
LDSCRIPT = $(LINKER)/MKL25Z128_flash.ld
FULL_PRJ = $(PROJECT)_flash

INCDIR  = $(patsubst %,-I%,$(DINCDIR) $(UINCDIR))
LIBDIR  = $(patsubst %,-L%,$(DLIBDIR) $(ULIBDIR))

DEFS    = $(DDEFS) $(UDEFS)

ADEFS   = $(DADEFS) $(UADEFS)
OBJS    = $(ASRC:.S=.o) $(SRC:.c=.o)
LIBS    = $(DLIBS) $(ULIBS)
MCFLAGS = -mcpu=$(MCU)

ASFLAGS = $(MCFLAGS) -g -gdwarf-2 -mthumb -Wa,-amhls=$(<:.S=.lst) $(ADEFS)
CPFLAGS = $(MCFLAGS) $(OPT) -gdwarf-2 -mthumb -fomit-frame-pointer -Wall -Wstrict-prototypes -fverbose-asm -Wa,-ahlms=$(<:.c=.lst) $(DEFS)
LDFLAGS = $(MCFLAGS) -mthumb -nostartfiles -T$(LDSCRIPT) -Wl,-Map=$(FULL_PRJ).map,--cref,--no-warn-mismatch $(LIBDIR)

# Generate dependency information
CPFLAGS += -MD -MP -MF .dep/$(@F).d

#
# makefile rules
#

all: $(OBJS) $(FULL_PRJ).elf $(FULL_PRJ).s19 $(FULL_PRJ).bin
	$(TRGT)size $(PROJECT)_flash.elf

%o: %c
	$(CC) -c $(CPFLAGS) -I . $(INCDIR) $< -o $@

%o: %S
	$(AS) -c $(ASFLAGS) $< -o $@

%elf: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) $(LIBS) -o $@

%s19: %elf
	$(S19) $< $@
	
%bin: %elf
	$(BIN)  $< $@
	
clean:
	rm -rf $(OBJS)
	rm -rf $(FULL_PRJ).elf
	rm -rf $(FULL_PRJ).map
	rm -rf $(FULL_PRJ).hex
	rm -rf $(FULL_PRJ).bin
	rm -rf $(SRC:.c=.lst)
	rm -rf $(ASRC:.S=.lst)
	rm -rf  .dep
	rm -rf *.s19

-include $(shell mkdir .dep) $(wildcard .dep/*)

