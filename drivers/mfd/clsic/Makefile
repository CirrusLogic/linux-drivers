#
# Makefile for the CLSIC device family
#

clsic-objs			:= clsic-core.o clsic-msg.o clsic-irq.o

clsic-objs			+= clsic-bootsrv.o clsic-syssrv.o
clsic-objs			+= clsic-rassrv.o clsic-rassrv-irq.o clsic-tables.o
clsic-objs			+= clsic-tacna.o

obj-$(CONFIG_MFD_CLSIC)		+= clsic.o
obj-$(CONFIG_MFD_CLSIC_SPI)	+= clsic-spi.o

# include tracepoint support
CFLAGS_clsic-trace.o		= -I$(src)
clsic-objs			+= clsic-trace.o
