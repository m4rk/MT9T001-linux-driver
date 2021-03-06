/*
 * Host AP (software wireless LAN access point) user space daemon for
 * Host AP kernel driver / IEEE 802.11 authentication (ACL)
 * Copyright (c) 2003, Jouni Malinen <jkmaline@cc.hut.fi>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation. See README and COPYING for
 * more details.
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>

#include "hostapd.h"
#include "ieee802_11.h"
#include "ieee802_11_auth.h"
#include "radius.h"
#include "radius_client.h"
#include "eloop.h"

#define RADIUS_ACL_TIMEOUT 30


struct hostapd_cached_radius_acl {
	time_t timestamp;
	macaddr addr;
	int accepted; /* HOSTAPD_ACL_* */
	struct hostapd_cached_radius_acl *next;
	u32 session_timeout;
};


struct hostapd_acl_query_data {
	time_t timestamp;
	u8 radius_id;
	macaddr addr;
	u8 *auth_msg; /* IEEE 802.11 authentication frame from station */
	size_t auth_msg_len;
	struct hostapd_acl_query_data *next;
};


static void hostapd_acl_cache_free(struct hostapd_cached_radius_acl *acl_cache)
{
	struct hostapd_cached_radius_acl *prev;

	while (acl_cache) {
		prev = acl_cache;
		acl_cache = acl_cache->next;
		free(prev);
	}
}


static int hostapd_acl_cache_get(hostapd *hapd, u8 *addr, u32 *session_timeout)
{
	struct hostapd_cached_radius_acl *entry;
	time_t now;

	time(&now);
	entry = hapd->acl_cache;

	while (entry) {
		if (memcmp(entry->addr, addr, ETH_ALEN) == 0) {
			if (now - entry->timestamp > RADIUS_ACL_TIMEOUT)
				return -1; /* entry has expired */
			if (entry->accepted == HOSTAPD_ACL_ACCEPT_TIMEOUT)
				*session_timeout = entry->session_timeout;
			return entry->accepted;
		}

		entry = entry->next;
	}

	return -1;
}


static void hostapd_acl_query_free(struct hostapd_acl_query_data *query)
{
	if (query == NULL)
		return;
	free(query->auth_msg);
	free(query);
}


static int hostapd_radius_acl_query(hostapd *hapd, u8 *addr,
				    struct hostapd_acl_query_data *query)
{
	struct radius_msg *msg;
	char buf[128];

	query->radius_id = radius_client_get_id(hapd);
	msg = radius_msg_new(RADIUS_CODE_ACCESS_REQUEST, query->radius_id);
	if (msg == NULL)
		return -1;

	radius_msg_make_authenticator(msg, addr, ETH_ALEN);

	snprintf(buf, sizeof(buf), RADIUS_ADDR_FORMAT, MAC2STR(addr));
	if (!radius_msg_add_attr(msg, RADIUS_ATTR_USER_NAME, buf,
				 strlen(buf))) {
		printf("Could not add User-Name\n");
		goto fail;
	}

	if (!radius_msg_add_attr_user_password(
		    msg, buf, strlen(buf),
		    hapd->conf->auth_server->shared_secret,
		    hapd->conf->auth_server->shared_secret_len)) {
		printf("Could not add User-Password\n");
		goto fail;
	}

	if (!radius_msg_add_attr(msg, RADIUS_ATTR_NAS_IP_ADDRESS,
				 (u8 *) &hapd->conf->own_ip_addr, 4)) {
		printf("Could not add NAS-IP-Address\n");
		goto fail;
	}

	snprintf(buf, sizeof(buf), RADIUS_802_1X_ADDR_FORMAT ":%s",
		 MAC2STR(hapd->own_addr), hapd->conf->ssid);
	if (!radius_msg_add_attr(msg, RADIUS_ATTR_CALLED_STATION_ID,
				 buf, strlen(buf))) {
		printf("Could not add Called-Station-Id\n");
		goto fail;
	}

	snprintf(buf, sizeof(buf), RADIUS_802_1X_ADDR_FORMAT,
		 MAC2STR(addr));
	if (!radius_msg_add_attr(msg, RADIUS_ATTR_CALLING_STATION_ID,
				 buf, strlen(buf))) {
		printf("Could not add Calling-Station-Id\n");
		goto fail;
	}

	if (!radius_msg_add_attr_int32(msg, RADIUS_ATTR_NAS_PORT_TYPE,
				       RADIUS_NAS_PORT_TYPE_IEEE_802_11)) {
		printf("Could not add NAS-Port-Type\n");
		goto fail;
	}

	snprintf(buf, sizeof(buf), "CONNECT 11Mbps 802.11b");
	if (!radius_msg_add_attr(msg, RADIUS_ATTR_CONNECT_INFO,
				 buf, strlen(buf))) {
		printf("Could not add Connect-Info\n");
		goto fail;
	}

	radius_client_send(hapd, msg, RADIUS_AUTH);
	return 0;

 fail:
	radius_msg_free(msg);
	free(msg);
	return -1;
}


int hostapd_allowed_address(hostapd *hapd, u8 *addr, u8 *msg, size_t len,
			    u32 *session_timeout)
{
	if (hostapd_maclist_found(hapd->conf->accept_mac,
				  hapd->conf->num_accept_mac, addr))
		return 1;

	if (hostapd_maclist_found(hapd->conf->deny_mac,
				  hapd->conf->num_deny_mac, addr))
		return 0;

	if (hapd->conf->macaddr_acl == ACCEPT_UNLESS_DENIED)
		return 1;
	if (hapd->conf->macaddr_acl == DENY_UNLESS_ACCEPTED)
		return 0;

	if (hapd->conf->macaddr_acl == USE_EXTERNAL_RADIUS_AUTH) {
		struct hostapd_acl_query_data *query;

		/* Check whether ACL cache has an entry for this station */
		int res = hostapd_acl_cache_get(hapd, addr, session_timeout);
		if (res == HOSTAPD_ACL_ACCEPT ||
		    res == HOSTAPD_ACL_ACCEPT_TIMEOUT)
			return res;
		if (res == HOSTAPD_ACL_REJECT)
			return HOSTAPD_ACL_REJECT;

		query = hapd->acl_queries;
		while (query) {
			if (memcmp(query->addr, addr, ETH_ALEN) == 0) {
				/* pending query in RADIUS retransmit queue;
				 * do not generate a new one */
				return HOSTAPD_ACL_PENDING;
			}
			query = query->next;
		}

		if (!hapd->conf->auth_server)
			return HOSTAPD_ACL_REJECT;

		/* No entry in the cache - query external RADIUS server */
		query = malloc(sizeof(*query));
		if (query == NULL) {
			printf("malloc for query data failed\n");
			return HOSTAPD_ACL_REJECT;
		}
		memset(query, 0, sizeof(*query));
		time(&query->timestamp);
		memcpy(query->addr, addr, ETH_ALEN);
		if (hostapd_radius_acl_query(hapd, addr, query)) {
			printf("Failed to send Access-Request for ACL "
			       "query.\n");
			hostapd_acl_query_free(query);
			return HOSTAPD_ACL_REJECT;
		}

		query->auth_msg = malloc(len);
		if (query->auth_msg == NULL) {
			printf("Failed to allocate memory for auth frame.\n");
			hostapd_acl_query_free(query);
			return HOSTAPD_ACL_REJECT;
		}
		memcpy(query->auth_msg, msg, len);
		query->auth_msg_len = len;
		query->next = hapd->acl_queries;
		hapd->acl_queries = query;

		/* Queued data will be processed in hostapd_acl_recv_radius()
		 * when RADIUS server replies to the sent Access-Request. */
		return HOSTAPD_ACL_PENDING;
	}

	return 0;
}


static void hostapd_acl_expire_cache(hostapd *hapd, time_t now)
{
	struct hostapd_cached_radius_acl *prev, *entry, *tmp;

	prev = NULL;
	entry = hapd->acl_cache;

	while (entry) {
		if (now - entry->timestamp > RADIUS_ACL_TIMEOUT) {
			HOSTAPD_DEBUG(HOSTAPD_DEBUG_MINIMAL,
				      "Cached ACL entry for " MACSTR
				      " has expired.\n", MAC2STR(entry->addr));
			if (prev)
				prev->next = entry->next;
			else
				hapd->acl_cache = entry->next;

			tmp = entry;
			entry = entry->next;
			free(tmp);
			continue;
		}

		prev = entry;
		entry = entry->next;
	}
}


static void hostapd_acl_expire_queries(hostapd *hapd, time_t now)
{
	struct hostapd_acl_query_data *prev, *entry, *tmp;

	prev = NULL;
	entry = hapd->acl_queries;

	while (entry) {
		if (now - entry->timestamp > RADIUS_ACL_TIMEOUT) {
			HOSTAPD_DEBUG(HOSTAPD_DEBUG_MINIMAL,
				      "ACL query for " MACSTR
				      " has expired.\n", MAC2STR(entry->addr));
			if (prev)
				prev->next = entry->next;
			else
				hapd->acl_queries = entry->next;

			tmp = entry;
			entry = entry->next;
			hostapd_acl_query_free(tmp);
			continue;
		}

		prev = entry;
		entry = entry->next;
	}
}


static void hostapd_acl_expire(void *eloop_ctx, void *timeout_ctx)
{
	hostapd *hapd = eloop_ctx;
	time_t now;

	time(&now);
	hostapd_acl_expire_cache(hapd, now);
	hostapd_acl_expire_queries(hapd, now);

	eloop_register_timeout(10, 0, hostapd_acl_expire, hapd, NULL);
}


/* Return 0 if RADIUS message was a reply to ACL query (and was processed here)
 * or -1 if not. */
static RadiusRxResult
hostapd_acl_recv_radius(hostapd *hapd,
			struct radius_msg *msg, struct radius_msg *req,
			u8 *shared_secret, size_t shared_secret_len,
			void *data)
{
	struct hostapd_acl_query_data *query, *prev;
	struct hostapd_cached_radius_acl *cache;

	query = hapd->acl_queries;
	prev = NULL;
	while (query) {
		if (query->radius_id == msg->hdr->identifier)
			break;
		prev = query;
		query = query->next;
	}
	if (query == NULL)
		return RADIUS_RX_UNKNOWN;

	HOSTAPD_DEBUG(HOSTAPD_DEBUG_MINIMAL, "Found matching Access-Request "
		      "for RADIUS message (id=%d)\n", query->radius_id);

	if (radius_msg_verify_acct(msg, shared_secret, shared_secret_len,
				   req)) {
		printf("Incoming RADIUS packet did not have correct "
		       "authenticator - dropped\n");
		return RADIUS_RX_UNKNOWN;
	}

	if (msg->hdr->code != RADIUS_CODE_ACCESS_ACCEPT &&
	    msg->hdr->code != RADIUS_CODE_ACCESS_REJECT) {
		printf("Unknown RADIUS message code %d to ACL query\n",
		       msg->hdr->code);
		return RADIUS_RX_UNKNOWN;
	}

	/* Insert Accept/Reject info into ACL cache */
	cache = malloc(sizeof(*cache));
	if (cache == NULL) {
		printf("Failed to add ACL cache entry\n");
		goto done;
	}
	memset(cache, 0, sizeof(*cache));
	time(&cache->timestamp);
	memcpy(cache->addr, query->addr, sizeof(cache->addr));
	if (msg->hdr->code == RADIUS_CODE_ACCESS_ACCEPT) {
		if (radius_msg_get_attr_int32(msg, RADIUS_ATTR_SESSION_TIMEOUT,
					      &cache->session_timeout) == 0)
			cache->accepted = HOSTAPD_ACL_ACCEPT_TIMEOUT;
		else
			cache->accepted = HOSTAPD_ACL_ACCEPT;
	} else
		cache->accepted = HOSTAPD_ACL_REJECT;
	cache->next = hapd->acl_cache;
	hapd->acl_cache = cache;

	/* Re-send original authentication frame for 802.11 processing */
	HOSTAPD_DEBUG(HOSTAPD_DEBUG_MINIMAL, "Re-sending authentication frame "
		      "after successful RADIUS ACL query\n");
	ieee802_11_mgmt(hapd, query->auth_msg, query->auth_msg_len,
			WLAN_FC_STYPE_AUTH);

 done:
	if (prev == NULL)
		hapd->acl_queries = query->next;
	else
		prev->next = query->next;

	hostapd_acl_query_free(query);

	return RADIUS_RX_PROCESSED;
}


int hostapd_acl_init(hostapd *hapd)
{
	if (radius_client_register(hapd, RADIUS_AUTH, hostapd_acl_recv_radius,
				   NULL))
		return -1;

	eloop_register_timeout(10, 0, hostapd_acl_expire, hapd, NULL);

	return 0;
}


void hostapd_acl_deinit(hostapd *hapd)
{
	struct hostapd_acl_query_data *query, *prev;

	hostapd_acl_cache_free(hapd->acl_cache);

	query = hapd->acl_queries;
	while (query) {
		prev = query;
		query = query->next;
		hostapd_acl_query_free(prev);
	}
}
