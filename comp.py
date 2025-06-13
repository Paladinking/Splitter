from comp_backend import *
import pathlib
import subprocess


CLFLAGS="/DEBUG /MD /W1"
LINKFLAGS = ""

BUILD_DIR = "build"
BIN_DIR = "bin"

CLFLAGS_DBG = f"-g -O0 -municode"
LINKFLAGS_DBG = f"-g -O0"

BUILD_DIR_DBG = "build-gcc"
BIN_DIR_DBG = "bin-gcc"


WORKDIR = pathlib.Path(__file__).parent.resolve()

add_backend("msvc", "Msvc", BUILD_DIR, BIN_DIR, WORKDIR, CLFLAGS, LINKFLAGS)
add_backend("mingw", "Mingw", BUILD_DIR_DBG, BIN_DIR_DBG, WORKDIR, CLFLAGS_DBG, LINKFLAGS_DBG)

get_parser().add_argument("-r", "--run", action="store_true", help="Run exe")

set_backend("msvc")

if not backend().msvc:
    DLLFLAGS = None

def main():
    sdl3 = find_package("SDL3")
    sdl3_image = find_package("SDL3_image")
    sdl3_ttf = find_package("SDL3_ttf")
    box2d = find_package("box2d")

    Executable("main.exe", "src/main.c", "src/polygon.c",
               packages=[sdl3, sdl3_image, sdl3_ttf, box2d])

    CopyToBin(*sdl3.dlls, *sdl3_image.dlls, *sdl3_ttf.dlls)

    build(__file__)

    if get_args().run:
        try:
            subprocess.run([pathlib.Path(bin_dir()) / "main.exe"])
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
