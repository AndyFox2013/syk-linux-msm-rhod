
# core framebuffer
#
obj-y := msm_fb.o

# MDP DMA/PPP engine
#
obj-y += mdp.o mdp_ppp.o
obj-$(CONFIG_FB_MSM_LEGACY_MDP) += mdp_hw_legacy.o
obj-$(CONFIG_MSM_MDP22) += mdp_ppp22.o

# MDDI interface
#
obj-y += mddi.o

# MDDI client/panel drivers
#
obj-y += mddi_client_simple.o
obj-y += mddi_client_toshiba.o
obj-y += mddi_client_novb9f6_5582.o

