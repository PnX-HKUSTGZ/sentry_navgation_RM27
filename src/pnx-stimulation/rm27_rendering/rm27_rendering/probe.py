"""Create a real, short-lived GL context; isolated by the parent's timeout."""
import ctypes as C
import ctypes.util
import json
import sys


def _library(name):
    return C.CDLL(ctypes.util.find_library(name) or f"lib{name}.so.1")


def _function(lib, name, result, arguments):
    function = getattr(lib, name)
    function.restype = result
    function.argtypes = arguments
    return function


def _strings(gl):
    get_string = _function(gl, "glGetString", C.c_char_p, [C.c_uint])
    values = {}
    for name, enum in (("renderer", 0x1F01), ("vendor", 0x1F00), ("version", 0x1F02)):
        value = get_string(enum)
        if not value:
            raise RuntimeError(f"OpenGL returned no {name}")
        values[name] = value.decode("utf-8", "replace")
    return values


def _glx():
    x11, gl = _library("X11"), _library("GL")
    pointer = C.c_void_p

    class VisualInfo(C.Structure):
        _fields_ = [("visual", pointer), ("visualid", C.c_ulong),
                    ("screen", C.c_int), ("depth", C.c_int), ("visual_class", C.c_int),
                    ("red_mask", C.c_ulong), ("green_mask", C.c_ulong),
                    ("blue_mask", C.c_ulong), ("colormap_size", C.c_int),
                    ("bits_per_rgb", C.c_int)]

    class WindowAttributes(C.Structure):
        _fields_ = [("background_pixmap", C.c_ulong), ("background_pixel", C.c_ulong),
                    ("border_pixmap", C.c_ulong), ("border_pixel", C.c_ulong),
                    ("bit_gravity", C.c_int), ("win_gravity", C.c_int),
                    ("backing_store", C.c_int), ("backing_planes", C.c_ulong),
                    ("backing_pixel", C.c_ulong), ("save_under", C.c_int),
                    ("event_mask", C.c_long), ("do_not_propagate_mask", C.c_long),
                    ("override_redirect", C.c_int), ("colormap", C.c_ulong),
                    ("cursor", C.c_ulong)]

    display = _function(x11, "XOpenDisplay", pointer, [C.c_char_p])(None)
    if not display:
        raise RuntimeError("Cannot open DISPLAY for an OpenGL context")
    visual = window = colormap = context = None
    try:
        screen = _function(x11, "XDefaultScreen", C.c_int, [pointer])(display)
        root = _function(x11, "XRootWindow", C.c_ulong, [pointer, C.c_int])(display, screen)
        attributes = (C.c_int * 5)(4, 12, 24, 5, 0)  # RGBA, depth=24, double buffer
        visual = _function(gl, "glXChooseVisual", C.POINTER(VisualInfo),
                           [pointer, C.c_int, C.POINTER(C.c_int)])(display, screen, attributes)
        if not visual:
            raise RuntimeError("No usable GLX visual on DISPLAY")
        colormap = _function(x11, "XCreateColormap", C.c_ulong,
                             [pointer, C.c_ulong, pointer, C.c_int])(
                                 display, root, visual.contents.visual, 0)
        settings = WindowAttributes()
        settings.colormap = colormap
        window = _function(x11, "XCreateWindow", C.c_ulong,
                           [pointer, C.c_ulong, C.c_int, C.c_int, C.c_uint, C.c_uint,
                            C.c_uint, C.c_int, C.c_uint, pointer, C.c_ulong,
                            C.POINTER(WindowAttributes)])(
                                display, root, 0, 0, 16, 16, 0, visual.contents.depth,
                                1, visual.contents.visual, 1 << 13, C.byref(settings))
        context = _function(gl, "glXCreateContext", pointer,
                            [pointer, C.POINTER(VisualInfo), pointer, C.c_int])(
                                display, visual, None, 1)
        if not context or not _function(gl, "glXMakeCurrent", C.c_int,
                                        [pointer, C.c_ulong, pointer])(display, window, context):
            raise RuntimeError("Could not make GLX context current")
        return _strings(gl)
    finally:
        if context:
            gl.glXMakeCurrent(display, 0, None)
            _function(gl, "glXDestroyContext", None, [pointer, pointer])(display, context)
        if window:
            _function(x11, "XDestroyWindow", C.c_int, [pointer, C.c_ulong])(display, window)
        if colormap:
            _function(x11, "XFreeColormap", C.c_int, [pointer, C.c_ulong])(display, colormap)
        if visual:
            _function(x11, "XFree", C.c_int, [pointer])(visual)
        _function(x11, "XCloseDisplay", C.c_int, [pointer])(display)


def _egl():
    egl, gl = _library("EGL"), _library("GL")
    pointer = C.c_void_p
    integer = C.c_int
    get_proc = _function(egl, "eglGetProcAddress", pointer, [C.c_char_p])
    address = get_proc(b"eglGetPlatformDisplayEXT")
    display = None
    if address:
        get_display = C.CFUNCTYPE(pointer, C.c_uint, pointer, C.POINTER(integer))(address)
        query_address = get_proc(b"eglQueryDevicesEXT")
        if query_address:
            query_devices = C.CFUNCTYPE(C.c_uint, integer, C.POINTER(pointer),
                                        C.POINTER(integer))(query_address)
            devices, count = (pointer * 16)(), integer()
            if query_devices(16, devices, C.byref(count)) and count.value:
                # Ogre's headless pbuffer renderer also enumerates EGL devices
                # and uses its default device, rather than a desktop EGL display.
                display = get_display(0x313F, devices[0], None)  # EGL_PLATFORM_DEVICE_EXT
        if not display:
            display = get_display(0x31DD, None, None)  # EGL_PLATFORM_SURFACELESS_MESA
    else:
        display = _function(egl, "eglGetDisplay", pointer, [pointer])(None)
    major, minor = integer(), integer()
    if not display or not _function(egl, "eglInitialize", C.c_uint,
                                   [pointer, C.POINTER(integer), C.POINTER(integer)])(
                                       display, C.byref(major), C.byref(minor)):
        raise RuntimeError("Could not initialize EGL display")
    context = surface = None
    try:
        if not _function(egl, "eglBindAPI", C.c_uint, [C.c_uint])(0x30A2):  # OpenGL
            raise RuntimeError("EGL does not support OpenGL")
        attributes = (integer * 9)(0x3033, 1, 0x3040, 8, 0x3024, 8, 0x3025, 8, 0x3038)
        config, count = pointer(), integer()
        if not _function(egl, "eglChooseConfig", C.c_uint,
                         [pointer, C.POINTER(integer), C.POINTER(pointer), integer,
                          C.POINTER(integer)])(display, attributes, C.byref(config), 1,
                                               C.byref(count)) or not count.value:
            raise RuntimeError("No usable EGL OpenGL pbuffer config")
        pbuffer = (integer * 5)(0x3057, 16, 0x3056, 16, 0x3038)
        surface = _function(egl, "eglCreatePbufferSurface", pointer,
                            [pointer, pointer, C.POINTER(integer)])(display, config, pbuffer)
        context_attributes = (integer * 1)(0x3038)
        context = _function(egl, "eglCreateContext", pointer,
                            [pointer, pointer, pointer, C.POINTER(integer)])(
                                display, config, None, context_attributes)
        if not surface or not context or not _function(egl, "eglMakeCurrent", C.c_uint,
                                                       [pointer, pointer, pointer, pointer])(
                                                           display, surface, surface, context):
            raise RuntimeError("Could not make EGL OpenGL context current")
        return _strings(gl)
    finally:
        _function(egl, "eglMakeCurrent", C.c_uint, [pointer, pointer, pointer, pointer])(
            display, None, None, None)
        if context:
            _function(egl, "eglDestroyContext", C.c_uint, [pointer, pointer])(display, context)
        if surface:
            _function(egl, "eglDestroySurface", C.c_uint, [pointer, pointer])(display, surface)
        _function(egl, "eglTerminate", C.c_uint, [pointer])(display)


if __name__ == "__main__":
    try:
        print(json.dumps(_egl() if sys.argv[1] == "egl" else _glx()))
    except Exception as error:
        print(json.dumps({"error": str(error)}))
        sys.exit(1)
