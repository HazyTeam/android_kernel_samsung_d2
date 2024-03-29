/*
 * Copyright (c) 2010 Broadcom Corporation
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/netdevice.h>
#include <brcmu_wifi.h>
#include <brcmu_utils.h>
#include "dhd.h"
#include "dhd_bus.h"
#include "dhd_proto.h"
#include "dhd_dbg.h"
#include "fwil.h"
#include "tracepoint.h"

#define PKTFILTER_BUF_SIZE		128
#define BRCMF_DEFAULT_BCN_TIMEOUT	3
#define BRCMF_DEFAULT_SCAN_CHANNEL_TIME	40
#define BRCMF_DEFAULT_SCAN_UNASSOC_TIME	40
#define BRCMF_DEFAULT_PACKET_FILTER	"100 0 0 0 0x01 0x00"

#ifdef DEBUG
static const char brcmf_version[] =
	"Dongle Host Driver, version " BRCMF_VERSION_STR "\nCompiled on "
	__DATE__ " at " __TIME__;
#else
static const char brcmf_version[] =
	"Dongle Host Driver, version " BRCMF_VERSION_STR;
#endif


bool brcmf_c_prec_enq(struct device *dev, struct pktq *q,
		      struct sk_buff *pkt, int prec)
{
	struct sk_buff *p;
	int eprec = -1;		/* precedence to evict from */
	bool discard_oldest;
	struct brcmf_bus *bus_if = dev_get_drvdata(dev);
	struct brcmf_pub *drvr = bus_if->drvr;

	/* Fast case, precedence queue is not full and we are also not
	 * exceeding total queue length
	 */
	if (!pktq_pfull(q, prec) && !pktq_full(q)) {
		brcmu_pktq_penq(q, prec, pkt);
		return true;
	}

	/* Determine precedence from which to evict packet, if any */
	if (pktq_pfull(q, prec))
		eprec = prec;
	else if (pktq_full(q)) {
		p = brcmu_pktq_peek_tail(q, &eprec);
		if (eprec > prec)
			return false;
	}

	/* Evict if needed */
	if (eprec >= 0) {
		/* Detect queueing to unconfigured precedence */
		discard_oldest = ac_bitmap_tst(drvr->wme_dp, eprec);
		if (eprec == prec && !discard_oldest)
			return false;	/* refuse newer (incoming) packet */
		/* Evict packet according to discard policy */
		p = discard_oldest ? brcmu_pktq_pdeq(q, eprec) :
			brcmu_pktq_pdeq_tail(q, eprec);
		if (p == NULL)
			brcmf_err("brcmu_pktq_penq() failed, oldest %d\n",
				  discard_oldest);

		brcmu_pkt_buf_free_skb(p);
	}

	/* Enqueue */
	p = brcmu_pktq_penq(q, prec, pkt);
	if (p == NULL)
		brcmf_err("brcmu_pktq_penq() failed\n");

	return p != NULL;
}

/* Convert user's input in hex pattern to byte-size mask */
static int brcmf_c_pattern_atoh(char *src, char *dst)
{
	int i;
	if (strncmp(src, "0x", 2) != 0 && strncmp(src, "0X", 2) != 0) {
		brcmf_err("Mask invalid format. Needs to start with 0x\n");
		return -EINVAL;
	}
	src = src + 2;		/* Skip past 0x */
	if (strlen(src) % 2 != 0) {
		brcmf_err("Mask invalid format. Length must be even.\n");
		return -EINVAL;
	}
	for (i = 0; *src != '\0'; i++) {
		unsigned long res;
		char num[3];
		strncpy(num, src, 2);
		num[2] = '\0';
		if (kstrtoul(num, 16, &res))
			return -EINVAL;
		dst[i] = (u8)res;
		src += 2;
	}
	return i;
}

static void
brcmf_c_pktfilter_offload_enable(struct brcmf_if *ifp, char *arg, int enable,
				 int master_mode)
{
	unsigned long res;
	char *argv;
	char *arg_save = NULL, *arg_org = NULL;
	s32 err;
	struct brcmf_pkt_filter_enable_le enable_parm;

	arg_save = kstrdup(arg, GFP_ATOMIC);
	if (!arg_save)
		goto fail;

	arg_org = arg_save;

	argv = strsep(&arg_save, " ");

	if (argv == NULL) {
		brcmf_err("No args provided\n");
		goto fail;
	}

	/* Parse packet filter id. */
	enable_parm.id = 0;
	if (!kstrtoul(argv, 0, &res))
		enable_parm.id = cpu_to_le32((u32)res);

	/* Enable/disable the specified filter. */
	enable_parm.enable = cpu_to_le32(enable);

	err = brcmf_fil_iovar_data_set(ifp, "pkt_filter_enable", &enable_parm,
				       sizeof(enable_parm));
	if (err)
		brcmf_err("Set pkt_filter_enable error (%d)\n", err);

	/* Control the master mode */
	err = brcmf_fil_iovar_int_set(ifp, "pkt_filter_mode", master_mode);
	if (err)
		brcmf_err("Set pkt_filter_mode error (%d)\n", err);

fail:
	kfree(arg_org);
}

static void brcmf_c_pktfilter_offload_set(struct brcmf_if *ifp, char *arg)
{
	struct brcmf_pkt_filter_le *pkt_filter;
	unsigned long res;
	int buf_len;
	s32 err;
	u32 mask_size;
	u32 pattern_size;
	char *argv[8], *buf = NULL;
	int i = 0;
	char *arg_save = NULL, *arg_org = NULL;

	arg_save = kstrdup(arg, GFP_ATOMIC);
	if (!arg_save)
		goto fail;

	arg_org = arg_save;

	buf = kmalloc(PKTFILTER_BUF_SIZE, GFP_ATOMIC);
	if (!buf)
		goto fail;

	argv[i] = strsep(&arg_save, " ");
	while (argv[i]) {
		i++;
		if (i >= 8) {
			brcmf_err("Too many parameters\n");
			goto fail;
		}
		argv[i] = strsep(&arg_save, " ");
	}

	if (i != 6) {
		brcmf_err("Not enough args provided %d\n", i);
		goto fail;
	}

	pkt_filter = (struct brcmf_pkt_filter_le *)buf;

	/* Parse packet filter id. */
	pkt_filter->id = 0;
	if (!kstrtoul(argv[0], 0, &res))
		pkt_filter->id = cpu_to_le32((u32)res);

	/* Parse filter polarity. */
	pkt_filter->negate_match = 0;
	if (!kstrtoul(argv[1], 0, &res))
		pkt_filter->negate_match = cpu_to_le32((u32)res);

	/* Parse filter type. */
	pkt_filter->type = 0;
	if (!kstrtoul(argv[2], 0, &res))
		pkt_filter->type = cpu_to_le32((u32)res);

	/* Parse pattern filter offset. */
	pkt_filter->u.pattern.offset = 0;
	if (!kstrtoul(argv[3], 0, &res))
		pkt_filter->u.pattern.offset = cpu_to_le32((u32)res);

	/* Parse pattern filter mask. */
	mask_size = brcmf_c_pattern_atoh(argv[4],
			(char *)pkt_filter->u.pattern.mask_and_pattern);

	/* Parse pattern filter pattern. */
	pattern_size = brcmf_c_pattern_atoh(argv[5],
		(char *)&pkt_filter->u.pattern.mask_and_pattern[mask_size]);

	if (mask_size != pattern_size) {
		brcmf_err("Mask and pattern not the same size\n");
		goto fail;
	}

	pkt_filter->u.pattern.size_bytes = cpu_to_le32(mask_size);
	buf_len = offsetof(struct brcmf_pkt_filter_le,
			   u.pattern.mask_and_pattern);
	buf_len += mask_size + pattern_size;

	err = brcmf_fil_iovar_data_set(ifp, "pkt_filter_add", pkt_filter,
				       buf_len);
	if (err)
		brcmf_err("Set pkt_filter_add error (%d)\n", err);

fail:
	kfree(arg_org);

	kfree(buf);
}

static void brcmf_c_arp_offload_set(struct brcmf_pub *drvr, int arp_mode)
{
	char iovbuf[32];
	int retcode;
	__le32 arp_mode_le;

	arp_mode_le = cpu_to_le32(arp_mode);
	brcmf_c_mkiovar("arp_ol", (char *)&arp_mode_le, 4, iovbuf,
			sizeof(iovbuf));
	retcode = brcmf_proto_cdc_set_dcmd(drvr, 0, BRCMF_C_SET_VAR,
				   iovbuf, sizeof(iovbuf));
	retcode = retcode >= 0 ? 0 : retcode;
	if (retcode)
		brcmf_dbg(TRACE, "failed to set ARP offload mode to 0x%x, retcode = %d\n",
			  arp_mode, retcode);
	else
		brcmf_dbg(TRACE, "successfully set ARP offload mode to 0x%x\n",
			  arp_mode);
}

static void brcmf_c_arp_offload_enable(struct brcmf_pub *drvr, int arp_enable)
{
	char iovbuf[32];
	int retcode;
	__le32 arp_enable_le;

	arp_enable_le = cpu_to_le32(arp_enable);

	brcmf_c_mkiovar("arpoe", (char *)&arp_enable_le, 4,
			iovbuf, sizeof(iovbuf));
	retcode = brcmf_proto_cdc_set_dcmd(drvr, 0, BRCMF_C_SET_VAR,
				   iovbuf, sizeof(iovbuf));
	retcode = retcode >= 0 ? 0 : retcode;
	if (retcode)
		brcmf_dbg(TRACE, "failed to enable ARP offload to %d, retcode = %d\n",
			  arp_enable, retcode);
	else
		brcmf_dbg(TRACE, "successfully enabled ARP offload to %d\n",
			  arp_enable);
}

int brcmf_c_preinit_dcmds(struct brcmf_pub *drvr)
{
	char iovbuf[BRCMF_EVENTING_MASK_LEN + 12];	/*  Room for
				 "event_msgs" + '\0' + bitvec  */
	char buf[128], *ptr;
	u32 dongle_align = drvr->bus_if->align;
	u32 glom = 0;
	__le32 roaming_le = cpu_to_le32(1);
	__le32 bcn_timeout_le = cpu_to_le32(3);
	__le32 scan_assoc_time_le = cpu_to_le32(40);
	__le32 scan_unassoc_time_le = cpu_to_le32(40);
	int i;

	mutex_lock(&drvr->proto_block);

	/* Set Country code */
	if (drvr->country_code[0] != 0) {
		if (brcmf_proto_cdc_set_dcmd(drvr, 0, BRCMF_C_SET_COUNTRY,
					      drvr->country_code,
					      sizeof(drvr->country_code)) < 0)
			brcmf_dbg(ERROR, "country code setting failed\n");
	}

	/* query for 'ver' to get version info from firmware */
	memset(buf, 0, sizeof(buf));
	ptr = buf;
	brcmf_c_mkiovar("ver", NULL, 0, buf, sizeof(buf));
	brcmf_proto_cdc_query_dcmd(drvr, 0, BRCMF_C_GET_VAR, buf, sizeof(buf));
	strsep(&ptr, "\n");
	/* Print fw version info */
	brcmf_dbg(ERROR, "Firmware version = %s\n", buf);

	/* Match Host and Dongle rx alignment */
	brcmf_c_mkiovar("bus:txglomalign", (char *)&dongle_align, 4, iovbuf,
		    sizeof(iovbuf));
	brcmf_proto_cdc_set_dcmd(drvr, 0, BRCMF_C_SET_VAR, iovbuf,
				  sizeof(iovbuf));

	/* disable glom option per default */
	brcmf_c_mkiovar("bus:txglom", (char *)&glom, 4, iovbuf, sizeof(iovbuf));
	brcmf_proto_cdc_set_dcmd(drvr, 0, BRCMF_C_SET_VAR, iovbuf,
				  sizeof(iovbuf));

	/* Setup timeout if Beacons are lost and roam is off to report
		 link down */
	brcmf_c_mkiovar("bcn_timeout", (char *)&bcn_timeout_le, 4, iovbuf,
		    sizeof(iovbuf));
	brcmf_proto_cdc_set_dcmd(drvr, 0, BRCMF_C_SET_VAR, iovbuf,
				  sizeof(iovbuf));

	/* Enable/Disable build-in roaming to allowed ext supplicant to take
		 of romaing */
	brcmf_c_mkiovar("roam_off", (char *)&roaming_le, 4,
		      iovbuf, sizeof(iovbuf));
	brcmf_proto_cdc_set_dcmd(drvr, 0, BRCMF_C_SET_VAR, iovbuf,
				  sizeof(iovbuf));

	/* Setup event_msgs */
	brcmf_c_mkiovar("event_msgs", drvr->eventmask, BRCMF_EVENTING_MASK_LEN,
		      iovbuf, sizeof(iovbuf));
	brcmf_proto_cdc_set_dcmd(drvr, 0, BRCMF_C_SET_VAR, iovbuf,
				  sizeof(iovbuf));

	brcmf_proto_cdc_set_dcmd(drvr, 0, BRCMF_C_SET_SCAN_CHANNEL_TIME,
		 (char *)&scan_assoc_time_le, sizeof(scan_assoc_time_le));
	brcmf_proto_cdc_set_dcmd(drvr, 0, BRCMF_C_SET_SCAN_UNASSOC_TIME,
		 (char *)&scan_unassoc_time_le, sizeof(scan_unassoc_time_le));

	/* Set and enable ARP offload feature */
	brcmf_c_arp_offload_set(drvr, BRCMF_ARPOL_MODE);
	brcmf_c_arp_offload_enable(drvr, true);

	/* Set up pkt filter */
	for (i = 0; i < drvr->pktfilter_count; i++) {
		brcmf_c_pktfilter_offload_set(drvr, drvr->pktfilter[i]);
		brcmf_c_pktfilter_offload_enable(drvr, drvr->pktfilter[i],
						 0, true);
	}

	mutex_unlock(&drvr->proto_block);

	return 0;
}

#ifdef CONFIG_BRCM_TRACING
void __brcmf_err(const char *func, const char *fmt, ...)
{
	struct va_format vaf = {
		.fmt = fmt,
	};
	va_list args;

	va_start(args, fmt);
	vaf.va = &args;
	pr_err("%s: %pV", func, &vaf);
	trace_brcmf_err(func, &vaf);
	va_end(args);
}
#endif
#if defined(CONFIG_BRCM_TRACING) || defined(CONFIG_BRCMDBG)
void __brcmf_dbg(u32 level, const char *func, const char *fmt, ...)
{
	struct va_format vaf = {
		.fmt = fmt,
	};
	va_list args;

	va_start(args, fmt);
	vaf.va = &args;
	if (brcmf_msg_level & level)
		pr_debug("%s %pV", func, &vaf);
	trace_brcmf_dbg(level, func, &vaf);
	va_end(args);
}
#endif
