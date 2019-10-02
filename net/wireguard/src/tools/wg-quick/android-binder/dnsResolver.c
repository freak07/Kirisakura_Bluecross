#include <uchar.h>
#include <string.h>

#include "dnsResolver.h"
#include "log_helpers.h"
#include "binder_ndk.h"

static void *onCreate(void *args)
{
  (void)args;
  FATAL("onCreate called on proxy object");
}

static void onDestroy(void *userdata)
{
  (void)userdata;
  FATAL("onDestroy called on proxy object");
}

static binder_status_t onTransact(AIBinder *binder,
                                  transaction_code_t code,
                                  const AParcel *parcel_in,
                                  AParcel *parcel_out)
{
  (void)binder;
  (void)code;
  (void)parcel_in;
  (void)parcel_out;
  FATAL("onTransact called on a proxy object");
}

static int32_t string_size(const char *string)
{
  if (!string)
    return -1;
  return strlen(string);
}

static int32_t string_array_size(char *const *array)
{
  if (!array)
    return -1;
  int32_t size;
  for (size = 0; array[size] != NULL; size++)
    continue;
  return size;
}

static const char *string_array_getter(const void *array_data, size_t index, int32_t *outlength)
{
  const char **array = (const char **)array_data;
  if (!array[index])
    *outlength = -1;
  else
    *outlength = strlen(array[index]);
  return array[index];
}

static binder_status_t meaningful_status(const AStatus *status_out)
{
  binder_status_t status = STATUS_OK;

 if (!AStatus_isOk(status_out))
 {
   binder_exception_t exc_code = AStatus_getExceptionCode(status_out);
   if (exc_code == EX_TRANSACTION_FAILED)
   {
     status = AStatus_getStatus(status_out);
     ERR("transaction failed: %d", status);
   }
   else
   {
     if (exc_code == EX_SERVICE_SPECIFIC)
     {
       int32_t exc_code_service = AStatus_getServiceSpecificError(status_out);
       ERR("service specific exception code: %d", exc_code_service);
     }
     else
       ERR("exception code: %d", exc_code);
     const char *message = AStatus_getMessage(status_out);
     if (message)
       ERR("exception message: %s", message);
     status = STATUS_FAILED_TRANSACTION;
   }
 }

  return status;
}

__attribute__((__warn_unused_result__))
void *dnsResolver_getHandle(void)
{
  if (!binder_is_available())
    FATAL("binder isn't available");

  AIBinder *binder = AServiceManager_getService("dnsresolver");
  if (!binder)
    goto error_early;
  AIBinder_Class *clazz = AIBinder_Class_define("android.net.IDnsResolver",
                                                &onCreate,
                                                &onDestroy,
                                                &onTransact);
  if (!clazz)
    goto error;

  if (!AIBinder_associateClass(binder, clazz))
    goto error;

  return binder;
error:
  AIBinder_decStrong(binder);
error_early:
  return NULL;
}

void dnsResolver_decRef(void *handle)
{
  if (!binder_is_available())
    FATAL("binder isn't available");

  AIBinder *const binder = handle;
  AIBinder_decStrong(binder);
}

int32_t dnsResolver_isAlive(void *handle, bool *aidl_return)
{
  if (!binder_is_available())
    FATAL("binder isn't available");

  AIBinder *const binder = handle;
  binder_status_t status;
  AParcel *parcel_in = NULL;

  status = AIBinder_prepareTransaction(binder, &parcel_in);
  if (status != STATUS_OK)
    return status;

  AParcel *parcel_out = NULL;
  status = AIBinder_transact(binder, FIRST_CALL_TRANSACTION + 0 /* isAlive */,
                             &parcel_in, &parcel_out, 0);
  if (status != STATUS_OK)
    goto end_parcel_in;

  AStatus *status_out = NULL;
  status = AParcel_readStatusHeader(parcel_out, &status_out);
  if (status != STATUS_OK)
    goto end_parcel_out;

  if (!AStatus_isOk(status_out))
  {
    status = meaningful_status(status_out);
    goto end_status;
  }

  status = AParcel_readBool(parcel_out, aidl_return);

end_status:
  AStatus_delete(status_out);
end_parcel_out:
  AParcel_delete(parcel_out);
  return status; // parcel_in is already deleted at this point
end_parcel_in:
  AParcel_delete(parcel_in);
  return status;
}

int32_t dnsResolver_createNetworkCache(void *handle, int32_t netId)
{
  if (!binder_is_available())
    FATAL("binder isn't available");

  AIBinder *const binder = handle;
  binder_status_t status;
  AParcel *parcel_in = NULL;

  status = AIBinder_prepareTransaction(binder, &parcel_in);
  if (status != STATUS_OK)
    return status;

  status = AParcel_writeInt32(parcel_in, netId);
  if (status != STATUS_OK)
    goto end_parcel_in;

  AParcel *parcel_out = NULL;
  status = AIBinder_transact(binder, FIRST_CALL_TRANSACTION + 7 /* createNetworkCache */,
                             &parcel_in, &parcel_out, 0);
  if (status != STATUS_OK)
    goto end_parcel_in;

  AStatus *status_out = NULL;
  status = AParcel_readStatusHeader(parcel_out, &status_out);
  if (status != STATUS_OK)
    goto end_parcel_out;

  if (!AStatus_isOk(status_out))
  {
    status = meaningful_status(status_out);
    goto end_status;
  }

  status = STATUS_OK;

end_status:
  AStatus_delete(status_out);
end_parcel_out:
  AParcel_delete(parcel_out);
  return status; // parcel_in is already deleted at this point
end_parcel_in:
  AParcel_delete(parcel_in);
  return status;
}

int32_t dnsResolver_setResolverConfiguration(void *handle, const struct ResolverParamsParcel *params)
{
  if (!binder_is_available())
    FATAL("binder isn't available");

  AIBinder *const binder = handle;
  binder_status_t status;
  AParcel *parcel_in = NULL;

  status = AIBinder_prepareTransaction(binder, &parcel_in);
  if (status != STATUS_OK)
    return status;

  status = AParcel_writeInt32(parcel_in, 1);
  if (status != STATUS_OK)
    goto end_parcel_in;

  int32_t start_position = AParcel_getDataPosition(parcel_in);
  status = AParcel_writeInt32(parcel_in, 0);
  if (status != STATUS_OK)
    goto end_parcel_in;

  status = AParcel_writeInt32(parcel_in, params->netId);
  if (status != STATUS_OK)
    goto end_parcel_in;
  status = AParcel_writeInt32(parcel_in, params->sampleValiditySeconds);
  if (status != STATUS_OK)
    goto end_parcel_in;
  status = AParcel_writeInt32(parcel_in, params->successThreshold);
  if (status != STATUS_OK)
    goto end_parcel_in;
  status = AParcel_writeInt32(parcel_in, params->minSamples);
  if (status != STATUS_OK)
    goto end_parcel_in;
  status = AParcel_writeInt32(parcel_in, params->maxSamples);
  if (status != STATUS_OK)
    goto end_parcel_in;
  status = AParcel_writeInt32(parcel_in, params->baseTimeoutMsec);
  if (status != STATUS_OK)
    goto end_parcel_in;
  status = AParcel_writeInt32(parcel_in, params->retryCount);
  if (status != STATUS_OK)
    goto end_parcel_in;
  status = AParcel_writeStringArray(parcel_in, params->servers, string_array_size(params->servers),
                           &string_array_getter);
  if (status != STATUS_OK)
    goto end_parcel_in;
  status = AParcel_writeStringArray(parcel_in, params->domains, string_array_size(params->domains),
                                    &string_array_getter);
  if (status != STATUS_OK)
    goto end_parcel_in;
  status = AParcel_writeString(parcel_in, params->tlsName, string_size(params->tlsName));
  if (status != STATUS_OK)
    goto end_parcel_in;
  status = AParcel_writeStringArray(parcel_in, params->tlsServers, string_array_size(params->tlsServers),
                                    &string_array_getter);
  if (status != STATUS_OK)
    goto end_parcel_in;
  status = AParcel_writeStringArray(parcel_in, params->tlsFingerprints, string_array_size(params->tlsFingerprints),
                                    &string_array_getter);
  if (status != STATUS_OK)
    goto end_parcel_in;

  int32_t end_position = AParcel_getDataPosition(parcel_in);
  status = AParcel_setDataPosition(parcel_in, start_position);
  if (status != STATUS_OK)
    goto end_parcel_in;
  status = AParcel_writeInt32(parcel_in, end_position - start_position);
  if (status != STATUS_OK)
    goto end_parcel_in;
  status = AParcel_setDataPosition(parcel_in, end_position);
  if (status != STATUS_OK)
    goto end_parcel_in;

  AParcel *parcel_out = NULL;
  status = AIBinder_transact(binder, FIRST_CALL_TRANSACTION + 2 /* setResolverConfiguration */,
                             &parcel_in, &parcel_out, 0);
  if (status != STATUS_OK)
    goto end_parcel_in;

  AStatus *status_out = NULL;
  status = AParcel_readStatusHeader(parcel_out, &status_out);
  if (status != STATUS_OK)
    goto end_parcel_out;

  status = meaningful_status(status_out);

  AStatus_delete(status_out);
end_parcel_out:
  AParcel_delete(parcel_out);
  return status; // parcel_in is already deleted at this point
end_parcel_in:
  AParcel_delete(parcel_in);
  return status;
}

void dnsResolver_dump(void *handle, int fd)
{
  if (!binder_is_available())
    FATAL("binder isn't available");

  AIBinder *const binder = handle;
  AIBinder_dump(binder, fd, NULL, 0);
}

int32_t dnsResolver_ping(void *handle)
{
  if (!binder_is_available())
    FATAL("binder isn't available");

  AIBinder *const binder = handle;
  return AIBinder_ping(binder);
}
