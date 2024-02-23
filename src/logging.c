#include <stdarg.h>
#include <stddef.h>

extern void rust_log_dbg(const char *log, size_t len);
extern void rust_log_info(const char *log, size_t len);
extern void rust_log_err(const char *log, size_t len);

#define min(a, b) (((a) < (b)) ? (a) : (b))

extern int vsnprintf(char *, unsigned int, const char *, va_list);

int c_log_info(const char *fmt, va_list argptr) {
    char formatted_string[1024] = {0};

    int result = vsnprintf(formatted_string, 1024, fmt, argptr);

    if (result < 0) {
        return -1;
    }

    rust_log_info(formatted_string, min((size_t)result, 1024));

    return 0;
}

int c_log_err(const char *fmt, va_list argptr) {
    char formatted_string[1024] = {0};

    int result = vsnprintf(formatted_string, 1024, fmt, argptr);

    if (result < 0) {
        return -1;
    }

    rust_log_err(formatted_string, min((size_t)result, 1024));

    return 0;
}

int c_log_dbg(const char *fmt, va_list argptr) {
    char formatted_string[1024] = {0};

    int result = vsnprintf(formatted_string, 1024, fmt, argptr);

    if (result < 0) {
        return -1;
    }

    rust_log_dbg(formatted_string, min((size_t)result, 1024));

    return 0;
}

