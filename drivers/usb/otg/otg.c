
int otg_send_event(enum usb_otg_event event)
{
	struct usb_phy *phy = usb_get_transceiver();
	int ret = -ENOTSUPP;

	if (phy && phy->otg && phy->otg->send_event)
		ret = phy->otg->send_event(phy->otg, event);

	if (phy)
		usb_put_transceiver(phy);

	return ret;
}
EXPORT_SYMBOL(otg_send_event);
