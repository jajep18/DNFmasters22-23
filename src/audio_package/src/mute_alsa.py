import ctypes

#For handling spammy stoud
import contextlib
import os
import sys

# This is a workaround for the ALSA error message
# Solution provided by 
# https://stackoverflow.com/questions/36956083/how-can-the-terminal-output-of-executables-run-by-python-functions-be-silenced-i
# See also https://stackoverflow.com/questions/7088672/pyaudio-working-but-spits-out-error-messages-each-time


ERROR_HANDLER_FUNC = ctypes.CFUNCTYPE(None, ctypes.c_char_p, ctypes.c_int,
                                      ctypes.c_char_p, ctypes.c_int,
                                      ctypes.c_char_p)


def py_error_handler(filename, line, function, err, fmt):
    pass

c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)

try:
    asound = ctypes.cdll.LoadLibrary('libasound.so.2')
    asound.snd_lib_error_set_handler(c_error_handler)
except OSError:
    pass

# End of workaround

# Workaround for suppressing all the stderr spam from jack as well: https://stackoverflow.com/questions/36956083/how-can-the-terminal-output-of-executables-run-by-python-functions-be-silenced-i
@contextlib.contextmanager
def suppress_stderr():
    devnull = os.open(os.devnull, os.O_WRONLY)
    old_stdout = os.dup(2)
    sys.stdout.flush()
    os.dup2(devnull, 2)
    os.close(devnull)
    try:
        yield
    finally:
        os.dup2(old_stdout, 2)
        os.close(old_stdout)