"""Linux atomic directory commit without replacing even an empty existing target."""
import ctypes
import errno
import os


def commit_directory(source, target):
    libc = ctypes.CDLL(None, use_errno=True)
    rename = getattr(libc, 'renameat2', None)
    if rename is None:
        raise OSError(errno.ENOSYS, 'Atomic no-replace rename requires Linux renameat2')
    rename.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
    rename.restype = ctypes.c_int
    if rename(-100, os.fsencode(source), -100, os.fsencode(target), 1) != 0:
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error), str(target))
