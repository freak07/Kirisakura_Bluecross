#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include <dlfcn.h>

#include "log_helpers.h"
#include "binder_ndk.h"

static bool binder_available = false;

bool binder_is_available(void)
{
  return binder_available;
}

AIBinder_Class *(*AIBinder_Class_define)(const char* interfaceDescriptor, AIBinder_Class_onCreate onCreate, AIBinder_Class_onDestroy onDestroy, AIBinder_Class_onTransact onTransact) __attribute__((warn_unused_result)) = NULL;
bool (*AIBinder_associateClass)(AIBinder* binder, const AIBinder_Class* clazz) = NULL;
void (*AIBinder_decStrong)(AIBinder* binder) = NULL;
binder_status_t (*AIBinder_prepareTransaction)(AIBinder* binder, AParcel** in) = NULL;
binder_status_t (*AIBinder_transact)(AIBinder* binder, transaction_code_t code, AParcel** in, AParcel** out, binder_flags_t flags) = NULL;
binder_status_t (*AIBinder_ping)(AIBinder* binder) = NULL;
binder_status_t (*AIBinder_dump)(AIBinder* binder, int fd, const char** args, uint32_t numArgs) = NULL;
binder_status_t (*AParcel_readStatusHeader)(const AParcel* parcel, AStatus** status) = NULL;
binder_status_t (*AParcel_readBool)(const AParcel* parcel, bool* value) = NULL;
void (*AParcel_delete)(AParcel* parcel) = NULL;
binder_status_t (*AParcel_setDataPosition)(const AParcel* parcel, int32_t position) = NULL;
int32_t (*AParcel_getDataPosition)(const AParcel* parcel) = NULL;
binder_status_t (*AParcel_writeInt32)(AParcel* parcel, int32_t value) = NULL;
binder_status_t (*AParcel_writeStringArray)(AParcel* parcel, const void* arrayData, int32_t length, AParcel_stringArrayElementGetter getter) = NULL;
binder_status_t (*AParcel_writeString)(AParcel* parcel, const char* string, int32_t length) = NULL;
bool (*AStatus_isOk)(const AStatus* status) = NULL;
void (*AStatus_delete)(AStatus* status) = NULL;
binder_exception_t (*AStatus_getExceptionCode)(const AStatus* status) = NULL;
int32_t (*AStatus_getServiceSpecificError)(const AStatus* status) = NULL;
const char* (*AStatus_getMessage)(const AStatus* status) = NULL;
binder_status_t (*AStatus_getStatus)(const AStatus* status) = NULL;
AIBinder *(*AServiceManager_getService)(const char* instance) __attribute__((__warn_unused_result__)) = NULL;

static void *load_libbinder_ndk(void)
{
  static bool already_loaded = false;
  static void *handle = NULL;
  if (!already_loaded)
  {
    if (!(handle = dlopen("libbinder_ndk.so", RTLD_LAZY)))
    {
      WARN("libbinder_ndk.so isn't available");
      binder_available = false;
    }
    else
      binder_available = true;
    already_loaded = true;
  }
  return handle;
}

#ifdef X
# undef X
#endif
#define X(symb) ({                                                      \
      if (!((symb) = (__typeof__(symb))dlsym(handle, #symb)))           \
        FATAL("can't import " #symb " from libbinder_ndk.so");          \
    })

static  __attribute__((__constructor__(65535))) void load_symbols(void)
{
  void *handle = load_libbinder_ndk();
  if (!handle)
    return; // no libbinder_ndk.so available
  X(AIBinder_Class_define);
  X(AIBinder_associateClass);
  X(AIBinder_decStrong);
  X(AIBinder_prepareTransaction);
  X(AIBinder_transact);
  X(AIBinder_ping);
  X(AIBinder_dump);
  X(AParcel_readStatusHeader);
  X(AParcel_readBool);
  X(AParcel_delete);
  X(AParcel_setDataPosition);
  X(AParcel_getDataPosition);
  X(AParcel_writeInt32);
  X(AParcel_writeStringArray);
  X(AParcel_readStatusHeader);
  X(AParcel_writeString);
  X(AStatus_isOk);
  X(AStatus_delete);
  X(AStatus_getExceptionCode);
  X(AStatus_getServiceSpecificError);
  X(AStatus_getMessage);
  X(AStatus_getStatus);
  X(AServiceManager_getService);
}

#undef X
