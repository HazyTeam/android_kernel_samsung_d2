#ifndef __NET_IPIP_H
#define __NET_IPIP_H 1

#include <linux/if_tunnel.h>
#include <net/gro_cells.h>
#include <net/ip.h>

/* Keep error state on tunnel for 30 sec */
#define IPTUNNEL_ERR_TIMEO	(30*HZ)

/* 6rd prefix/relay information */
struct ip_tunnel_6rd_parm {
	struct in6_addr		prefix;
	__be32			relay_prefix;
	u16			prefixlen;
	u16			relay_prefixlen;
};

struct ip_tunnel {
	struct ip_tunnel __rcu	*next;
	struct net_device	*dev;

	int			err_count;	/* Number of arrived ICMP errors */
	unsigned long		err_time;	/* Time when the last ICMP error arrived */

	/* These four fields used only by GRE */
	__u32			i_seqno;	/* The last seen seqno	*/
	__u32			o_seqno;	/* The last output seqno */
	int			hlen;		/* Precalculated GRE header length */
	int			mlink;

	struct ip_tunnel_parm	parms;

	/* for SIT */
#ifdef CONFIG_IPV6_SIT_6RD
	struct ip_tunnel_6rd_parm	ip6rd;
#endif
	struct ip_tunnel_prl_entry __rcu *prl;		/* potential router list */
	unsigned int			prl_count;	/* # of entries in PRL */

	struct gro_cells		gro_cells;
};

struct ip_tunnel_prl_entry {
	struct ip_tunnel_prl_entry __rcu *next;
	__be32				addr;
	u16				flags;
	struct rcu_head			rcu_head;
};

#define __IPTUNNEL_XMIT(stats1, stats2) do {				\
	int err;							\
	int pkt_len = skb->len - skb_transport_offset(skb);		\
									\
	skb->ip_summed = CHECKSUM_NONE;					\
	ip_select_ident(skb, NULL);				\
									\
	err = ip_local_out(skb);					\
	if (likely(net_xmit_eval(err) == 0)) {				\
		u64_stats_update_begin(&(stats1)->syncp);		\
		(stats1)->tx_bytes += pkt_len;				\
		(stats1)->tx_packets++;					\
		u64_stats_update_end(&(stats1)->syncp);			\
	} else {							\
		(stats2)->tx_errors++;					\
		(stats2)->tx_aborted_errors++;				\
	}								\
} while (0)

#define IPTUNNEL_XMIT() __IPTUNNEL_XMIT(txq, stats)

static inline void tunnel_ip_select_ident(struct sk_buff *skb,
					  const struct iphdr  *old_iph,
					  struct dst_entry *dst)
{
	struct iphdr *iph = ip_hdr(skb);

	/* Use inner packet iph-id if possible. */
	if (skb->protocol == htons(ETH_P_IP) && old_iph->id)
		iph->id	= old_iph->id;
	else
		__ip_select_ident(iph, dst,
				  (skb_shinfo(skb)->gso_segs ?: 1) - 1);
}
#endif
