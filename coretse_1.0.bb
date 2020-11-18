SUMMARY = "Example of how to build an external Linux kernel module"
LICENSE = "MIT"

inherit module

SRC_URI = "file://Makefile \
           file://altera_msgdma.c \
           file://altera_msgdma.h \
           file://altera_msgdmahw.h        \
           file://altera_sgdma.c           \
           file://altera_sgdma.h           \
           file://altera_sgdmahw.h         \
           file://altera_tse_ethtool.c     \
           file://altera_tse.h             \
           file://altera_tse_main.c        \
           file://altera_utils.c           \
           file://altera_utils.h           \
           file://core_tse.c               \
           file://core_tse.h               \
           file://coretse_regs.h           \
           file://coretse_types.h          \
           file://coretse_user_config.h    \
           file://crc32.c                  \
           file://crc32.h                  \
           file://vsc8575_phy.c            \
           file://cpu_types.h              \
           file://phy.h                    \
           file://MIT                      \
          "

LIC_FILES_CHKSUM = "file://MIT;md5=e8f57dd048e186199433be2c41bd3d6d"

S = "${WORKDIR}"

# The inherit of module.bbclass will automatically name module packages with
# "kernel-module-" prefix as required by the oe-core build environment.

RPROVIDES_${PN} += "kernel-module-coretse"
