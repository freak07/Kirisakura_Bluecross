#pragma once

#include <stdint.h>
#include <stdbool.h>

struct ResolverParamsParcel
{
  int32_t netId;
  int32_t sampleValiditySeconds;
  int32_t successThreshold;
  int32_t minSamples;
  int32_t maxSamples;
  int32_t baseTimeoutMsec;
  int32_t retryCount;
  char **servers;         // NULL terminated array of zero-terminated UTF-8 strings
  char **domains;         // NULL terminated array of zero-terminated UTF-8 strings
  char *tlsName;          // zero-terminated UTF-8 string
  char **tlsServers;      // NULL terminated array of zero-terminated UTF-8 strings
  char **tlsFingerprints; // NULL terminated array of zero-terminated UTF-8 strings
};

/*
 * return code == 0 if no error, see binder_status_t in binder_ndk.h
 */

void *dnsResolver_getHandle(void) __attribute__((__warn_unused_result__));
void dnsResolver_decRef(void *handle);
int32_t dnsResolver_isAlive(void *handle, bool *result);
int32_t dnsResolver_setResolverConfiguration(void *handle, const struct ResolverParamsParcel *params);
int32_t dnsResolver_createNetworkCache(void *handle, int32_t netId);
void dnsResolver_dump(void *handle, int fd);
int32_t dnsResolver_ping(void *handle);
