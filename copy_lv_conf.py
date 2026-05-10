Import("env")
import shutil, os

src  = os.path.join(env["PROJECT_DIR"], "lv_conf.h")
dst  = os.path.join(env["PROJECT_LIBDEPS_DIR"],
                    env["PIOENV"], "lv_conf.h")

if os.path.isfile(src):
    shutil.copy(src, dst)
    print(f"lv_conf.h -> {dst}")
else:
    print("WARNUNG: lv_conf.h nicht im Projekt-Root gefunden!")
