#
_XDCBUILDCOUNT = 0
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = C:/Users/allef.silva/lnls-elp/ARM/.config
override XDCROOT = C:/ti/xdctools_3_25_06_96
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = C:/Users/allef.silva/lnls-elp/ARM/.config;C:/ti/xdctools_3_25_06_96/packages;..
HOSTOS = Windows
endif
