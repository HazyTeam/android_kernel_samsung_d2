/*
    comedi/drivers/cb_pcidio.c
    A Comedi driver for PCI-DIO24H & PCI-DIO48H of ComputerBoards (currently MeasurementComputing)

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 2000 David A. Schleef <ds@schleef.org>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

*/
/*
Driver: cb_pcidio
Description: ComputerBoards' DIO boards with PCI interface
Devices: [Measurement Computing] PCI-DIO24 (cb_pcidio), PCI-DIO24H, PCI-DIO48H
Author: Yoshiya Matsuzaka
Updated: Mon, 29 Oct 2007 15:40:47 +0000
Status: experimental

This driver has been modified from skel.c of comedi-0.7.70.

Configuration Options:
  [0] - PCI bus of device (optional)
  [1] - PCI slot of device (optional)
  If bus/slot is not specified, the first available PCI device will
  be used.

Passing a zero for an option is the same as leaving it unspecified.
*/

/*------------------------------ HEADER FILES ---------------------------------*/
#include "../comedidev.h"
#include "8255.h"

/*-------------------------- MACROS and DATATYPES -----------------------------*/
#define PCI_VENDOR_ID_CB	0x1307

/*
 * Board descriptions for two imaginary boards.  Describing the
 * boards in this way is optional, and completely driver-dependent.
 * Some drivers use arrays such as this, other do not.
 */
struct pcidio_board {
	const char *name;	/*  name of the board */
	int dev_id;
	int n_8255;		/*  number of 8255 chips on board */
};

static const struct pcidio_board pcidio_boards[] = {
	{
	 .name = "pci-dio24",
	 .dev_id = 0x0028,
	 .n_8255 = 1,
	 },
	{
	 .name = "pci-dio24h",
	 .dev_id = 0x0014,
	 .n_8255 = 1,
	 },
	{
	 .name = "pci-dio48h",
	 .dev_id = 0x000b,
	 .n_8255 = 2,
	 },
};

/*
 * Useful for shorthand access to the particular board structure
 */
#define thisboard ((const struct pcidio_board *)dev->board_ptr)

static struct pci_dev *pcidio_find_pci_dev(struct comedi_device *dev,
					   struct comedi_devconfig *it)
{
	struct pci_dev *pcidev = NULL;
	int bus = it->options[0];
	int slot = it->options[1];
	int i;

	for_each_pci_dev(pcidev) {
		if (bus || slot) {
			if (bus != pcidev->bus->number ||
				slot != PCI_SLOT(pcidev->devfn))
				continue;
		}
		if (pcidev->vendor != PCI_VENDOR_ID_CB)
			continue;
		for (i = 0; i < ARRAY_SIZE(pcidio_boards); i++) {
			if (pcidio_boards[i].dev_id != pcidev->device)
				continue;

			dev->board_ptr = pcidio_boards + i;
			return pcidev;
		}
	}
	dev_err(dev->class_dev,
		"No supported board found! (req. bus %d, slot %d)\n",
		bus, slot);
	return NULL;
}

static int pcidio_attach(struct comedi_device *dev, struct comedi_devconfig *it)
{
	struct pci_dev *pcidev;
	int i;
	int ret;

	pcidev = pcidio_find_pci_dev(dev, it);
	if (!pcidev)
		return -EIO;
	comedi_set_hw_dev(dev, &pcidev->dev);

/*
 * Initialize dev->board_name.  Note that we can use the "thisboard"
 * macro now, since we just initialized it in the last line.
 */
	dev->board_name = thisboard->name;

	devpriv->pci_dev = pcidev;
	dev_dbg(dev->hw_dev, "Found %s on bus %i, slot %i\n", thisboard->name,
		devpriv->pci_dev->bus->number,
		PCI_SLOT(devpriv->pci_dev->devfn));
	if (comedi_pci_enable(pcidev, thisboard->name))
		return -EIO;

	/*
	 * Use PCI BAR 2 region if non-zero length, else use PCI BAR 1 region.
	 * PCI BAR 1 is only used for older PCI-DIO48H boards.  At some point
	 * the PCI-DIO48H was redesigned to use the same PCI interface chip
	 * (and same PCI BAR region) as the other boards.
	 */
	devpriv->dio_reg_base =
	    pci_resource_start(devpriv->pci_dev,
			       (pci_resource_len(pcidev, 2) ? 2 : 1));

/*
 * Allocate the subdevice structures.  alloc_subdevice() is a
 * convenient macro defined in comedidev.h.
 */
	if (alloc_subdevices(dev, thisboard->n_8255) < 0)
		return -ENOMEM;

	for (i = 0; i < thisboard->n_8255; i++) {
		subdev_8255_init(dev, dev->subdevices + i,
				 NULL, devpriv->dio_reg_base + i * 4);
		dev_dbg(dev->hw_dev, "subdev %d: base = 0x%lx\n", i,
			devpriv->dio_reg_base + i * 4);
	}

	return 1;
}

static void pcidio_detach(struct comedi_device *dev)
{
	struct pci_dev *pcidev = comedi_to_pci_dev(dev);

	if (pcidev) {
		if (dev->iobase)
			comedi_pci_disable(pcidev);
		pci_dev_put(pcidev);
	}
	if (dev->subdevices) {
		int i;
		for (i = 0; i < thisboard->n_8255; i++)
			subdev_8255_cleanup(dev, dev->subdevices + i);
	}
}

static struct comedi_driver cb_pcidio_driver = {
	.driver_name	= "cb_pcidio",
	.module		= THIS_MODULE,
	.attach		= pcidio_attach,
	.detach		= pcidio_detach,
};

static int __devinit cb_pcidio_pci_probe(struct pci_dev *dev,
						const struct pci_device_id *ent)
{
	return comedi_pci_auto_config(dev, &cb_pcidio_driver);
}

static void __devexit cb_pcidio_pci_remove(struct pci_dev *dev)
{
	comedi_pci_auto_unconfig(dev);
}

static DEFINE_PCI_DEVICE_TABLE(cb_pcidio_pci_table) = {
	{ PCI_DEVICE(PCI_VENDOR_ID_CB, 0x0028) },
	{ PCI_DEVICE(PCI_VENDOR_ID_CB, 0x0014) },
	{ PCI_DEVICE(PCI_VENDOR_ID_CB, 0x000b) },
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, cb_pcidio_pci_table);

static struct pci_driver cb_pcidio_pci_driver = {
	.name		= "cb_pcidio",
	.id_table	= cb_pcidio_pci_table,
	.probe		= cb_pcidio_pci_probe,
	.remove		= __devexit_p(cb_pcidio_pci_remove),
};
module_comedi_pci_driver(cb_pcidio_driver, cb_pcidio_pci_driver);

MODULE_AUTHOR("Comedi http://www.comedi.org");
MODULE_DESCRIPTION("Comedi low-level driver");
MODULE_LICENSE("GPL");
