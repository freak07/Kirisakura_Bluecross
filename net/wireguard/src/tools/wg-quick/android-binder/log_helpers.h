#pragma once

#include <stdio.h>
#include <stdlib.h>

#include <android/log.h>

#define LOG(fmt, ...) ({                                                      \
      fprintf(stderr, "[INFO] " fmt "\n" ,## __VA_ARGS__);                    \
      __android_log_print(ANDROID_LOG_INFO, "wg-quick", fmt ,## __VA_ARGS__); \
    })

#define WARN(fmt, ...) ({                                                     \
      fprintf(stderr, "[WARN] " fmt "\n" ,## __VA_ARGS__);                    \
      __android_log_print(ANDROID_LOG_WARN, "wg-quick", fmt ,## __VA_ARGS__); \
    })

#define ERR(fmt, ...) ({                                                       \
      fprintf(stderr, "[ERR] " fmt "\n" ,## __VA_ARGS__);                      \
      __android_log_print(ANDROID_LOG_ERROR, "wg-quick", fmt ,## __VA_ARGS__); \
    })

#define FATAL(fmt, ...) ({                                                     \
      fprintf(stderr, "[FATAL] " fmt "\n",## __VA_ARGS__);                     \
      __android_log_print(ANDROID_LOG_FATAL, "wg-quick", fmt ,## __VA_ARGS__); \
      abort();                                                                 \
    })
