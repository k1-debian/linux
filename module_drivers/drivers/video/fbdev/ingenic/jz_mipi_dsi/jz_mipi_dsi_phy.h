#ifndef __JZ_MIPI_DSI_PHY_H__
#define __JZ_MIPI_DSI_PHY_H__

int init_dsi_phy(struct dsi_device *dsi);

#ifdef CONFIG_SOC_X2600
	void lvds_rx_enable(struct dsi_device *dsi);
#endif

#endif  // __JZ_MIPI_DSI_PHY_H__
