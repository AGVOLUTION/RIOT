PKG_NAME    = nanomodbus
PKG_URL     = https://github.com/debevv/nanoMODBUS
PKG_VERSION = 91d6782930ee263bc760f27b0cbc5b82773c5f0d  # v1.23.0
PKG_LICENSE = MIT

include $(RIOTBASE)/pkg/pkg.mk

# Third-party code: it casts the uint8_t frame buffer to uint16_t*. Harmless on
# Cortex-M (hardware unaligned halfword access); silence RIOT's strict warning.
CFLAGS += -Wno-cast-align

# nanomodbus ships a single translation unit (nanomodbus.c) in the repo root.
all:
	$(QQ)"$(MAKE)" -C $(PKG_SOURCE_DIR) -f $(RIOTBASE)/Makefile.base \
		MODULE=$(PKG_NAME) SRC=nanomodbus.c
