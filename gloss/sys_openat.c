#include <errno.h>

int _openat(int dirfd, const char *name, int flags, int mode) {
#ifdef __SEGGER_LIBC__
  errno = EINVAL;
#else
  errno = ENOSYS;
#endif
  return -1;
}
