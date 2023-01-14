TOOLSET    ?= arm-none-eabi-
OBJDIR     ?= build
FWNAME     ?= button

FWCC        = $(TOOLSET)gcc
FWOBJCOPY   = $(TOOLSET)objcopy
FWSIZE      = $(TOOLSET)size

#RM = rm -f
#RMF = rm -rf
#FixPath = $(subst /,\, $1)
#MD = mkdir -p 

RM = del /Q
FixPath = $(subst /,\, $1)
MD = MD
RMF = RD /S /Q

FWCFLAGS   ?= -std=c11 -fdata-sections -ffunction-sections -Wall
FWLDFLAGS  ?= -specs=nano.specs -nostartfiles -Wl,--gc-sections

root_source_dir	= src
root_source_dir	+= ebus
search_wildcards = $(addsuffix /*.s,$(root_source_dir))
search_wildcards += $(addsuffix /*.c,$(root_source_dir))
FWSRC   = $(wildcard $(search_wildcards))

FWINCS      = ebus src .
FWOBJS      = $(addprefix $(OBJDIR)/, $(addsuffix .o, $(basename $(FWSRC))))

FWDEFS     ?= STM32F0 STM32F030x6
FWCPU      ?= -mthumb -mcpu=cortex-m0
FWLDSCRIPT ?= scripts/stm32f030x4.ld

SRCPATH     = $(sort $(dir $(FWSRC)))
DSTPATH     = $(addprefix $(OBJDIR)/,$(SRCPATH))

.PHONY: clean release 

all: release

release: $(FWNAME).srec $(FWNAME).bin
release: FWCFLAGS += -Os

$(DSTPATH):
	@$(MD) $(call FixPath, $@)
#	@$(MD) $@
$(FWOBJS): | $(DSTPATH)

clean:
#	@ECHO $(call FixPath, $(FWOBJS)) del files
	@$(RM) $(call FixPath, $(FWOBJS))
	@$(RM) $(call FixPath, $(FWNAME).*)
	@$(RMF) $(OBJDIR)
%.srec: %.elf
	@echo creating $@
	@$(FWOBJCOPY) -O srec $< $@

%.bin: %.elf
	@echo creating $@
	@$(FWOBJCOPY) -O binary $< $@

$(FWNAME).elf: $(FWOBJS) $(FWLDSCRIPT)
	@echo building firmware
	@$(FWCC) $(FWCPU) $(FWLDFLAGS) -Wl,--script=$(FWLDSCRIPT) $(FWOBJS) -Wl,-Map=$(FWNAME).map -o $@
	@$(FWSIZE) $@

$(OBJDIR)/%.o: %.c
	@echo compiling $<
	@$(FWCC) $(FWCPU) $(FWCFLAGS) $(addprefix -D,$(FWDEFS)) $(addprefix -I,$(FWINCS)) -o $@ -c $<

$(OBJDIR)/%.o: %.s
	@echo assembling $<
	@$(FWCC) $(addprefix -D,$(FWDEFS)) $(addprefix -I,$(FWINCS)) -o $@ -c $<

$(OBJDIR)/%.o: %.S
	@echo assembling $<
	@$(FWCC) $(addprefix -D,$(FWDEFS)) $(addprefix -I,$(FWINCS)) -o $@ -c $<
