// SPDX-License-Identifier: GPL-2.0
/*
 * NitroSPI-Simple SPI controller driver
 *
 * Copyright (C) 2019 Antmicro Ltd. <www.antmicro.com>
 * Copyright (C) 2021 Sylvain Munaut <tnt@246tNt.com>
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>

#define DRIVER_NAME "no2spi-simple"

struct no2spi_regs {
	uint32_t csr;
	uint32_t data;
} __attribute__((packed,aligned(4)));

struct no2spi_hw {
	struct spi_master *master;
	volatile struct no2spi_regs __iomem *regs;
};

static void no2spi_set_cs(struct spi_device *spi, bool cs_n)
{
	struct no2spi_hw *hw = spi_master_get_devdata(spi->master);

	/* Set chip select */
	hw->regs->csr = 0x80 | (cs_n ? 0 : BIT(spi->chip_select));
}

static int no2spi_transfer_one(struct spi_master *master, struct spi_device *spi,
			struct spi_transfer *t)
{
	struct no2spi_hw *hw = spi_master_get_devdata(master);
	const u8 *tx = t->tx_buf;
	u8 *rx = t->rx_buf;
	int32_t v;
	int i;

	/* Send buffer */
	for (i = 0; i < t->len; i++) {
		hw->regs->data = tx ? *tx++ : 0x00;
		while ((v = hw->regs->data) < 0)
			cpu_relax();
		if (rx)
			*rx++ = v;
	}

	return 0;
}

static int no2spi_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct no2spi_hw *hw;
	struct spi_master *master;
	struct resource *res;
	int ret;
	u32 val;

	master = spi_alloc_master(&pdev->dev, sizeof(*hw));
	if (!master)
		return -ENOMEM;

	master->dev.of_node = pdev->dev.of_node;
	master->bus_num = pdev->id;
	master->mode_bits = SPI_MODE_0;
	master->bits_per_word_mask = SPI_BPW_MASK(8);
	master->set_cs = no2spi_set_cs;
	master->transfer_one = no2spi_transfer_one;
	master->flags = 0;

	/* get sck frequency */
	ret = of_property_read_u32(node, "no2fpga,sck-frequency", &val);
	if (ret)
		goto err;
	master->min_speed_hz = val;
	master->max_speed_hz = val;

	/* get num cs */
	ret = of_property_read_u32(node, "no2fpga,num-cs", &val);
	if (ret)
		goto err;
	master->num_chipselect = val;

	hw = spi_master_get_devdata(master);
	hw->master = master;

	/* get base address */
	hw->regs = devm_platform_get_and_ioremap_resource(pdev, 0, NULL);
	if (IS_ERR((void*)hw->regs)) {
		ret = PTR_ERR((void*)hw->regs);
		goto err;
	}

	/* register controller */
	ret = devm_spi_register_master(&pdev->dev, master);
	if (ret)
		goto err;

	return 0;

err:
	spi_master_put(master);
	return ret;
}

static const struct of_device_id no2spi_match[] = {
	{ .compatible = "no2fpga,no2spi-simple" },
	{}
};
MODULE_DEVICE_TABLE(of, no2spi_match);

static struct platform_driver no2spi_driver = {
	.probe = no2spi_probe,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(no2spi_match)
	}
};
module_platform_driver(no2spi_driver)

MODULE_AUTHOR("Sylvain Munaut <tnt@246tNt.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
