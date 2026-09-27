"""Full compile and link check of the CubeIDE project sources with arm-none-eabi-gcc.
Usage: python link_check.py <repo_root>
Reports compile errors and unresolved symbols at link time (build_check.ps1 is syntax only)."""
import concurrent.futures as cf
import glob
import os
import subprocess
import sys

root = os.path.abspath(sys.argv[1])
gcc = r"C:\ST\STM32CubeIDE_1.19.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740\tools\bin\arm-none-eabi-gcc.exe"
out = os.path.join(root, ".linkcheck")
os.makedirs(out, exist_ok=True)

incs = ["Core/Inc", "Core/Src/Kalman", "Core/Src/Kalman/Libs", "FATFS/Target", "FATFS/App",
        "Drivers/STM32H7xx_HAL_Driver/Inc", "Drivers/STM32H7xx_HAL_Driver/Inc/Legacy",
        "Middlewares/Third_Party/FreeRTOS/Source/include",
        "Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2",
        "Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F",
        "Drivers/CMSIS/RTOS2/Include", "Middlewares/Third_Party/FatFs/src",
        "Drivers/CMSIS/Device/ST/STM32H7xx/Include", "Drivers/CMSIS/Include"]
flags = ["-mcpu=cortex-m7", "-mfpu=fpv5-d16", "-mfloat-abi=hard", "-mthumb", "-std=gnu11", "-Os",
         "-ffunction-sections", "-fdata-sections", "-DUSE_PWR_LDO_SUPPLY", "-DUSE_HAL_DRIVER", "-DSTM32H723xx"]
flags += ["-I" + os.path.join(root, i) for i in incs]

srcs = []
for pat in ["Core/Src/**/*.c", "FATFS/App/*.c", "FATFS/Target/*.c",
            "Drivers/STM32H7xx_HAL_Driver/Src/*.c",
            "Middlewares/Third_Party/FreeRTOS/Source/*.c",
            "Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/*.c",
            "Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/*.c",
            "Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c",
            "Middlewares/Third_Party/FatFs/src/*.c", "Middlewares/Third_Party/FatFs/src/option/*.c"]:
    srcs += glob.glob(os.path.join(root, pat), recursive=True)
srcs = [s for s in srcs if not s.endswith("_template.c")]
asm = [os.path.join(root, "startup_stm32h723xx.s")]
if not os.path.exists(asm[0]):
    asm = glob.glob(os.path.join(root, "**", "startup_stm32h723xx.s"), recursive=True)[:1]


def compile_one(src):
    obj = os.path.join(out, os.path.relpath(src, root).replace(os.sep, "_") + ".o")
    cmd = [gcc] + flags + ["-c", src, "-o", obj]
    if src.endswith(".s"):
        cmd = [gcc, "-mcpu=cortex-m7", "-mfpu=fpv5-d16", "-mfloat-abi=hard", "-mthumb", "-x", "assembler-with-cpp", "-c", src, "-o", obj]
    r = subprocess.run(cmd, capture_output=True, text=True)
    return src, obj, r.returncode, r.stderr


objs, failed = [], []
with cf.ThreadPoolExecutor(max_workers=8) as ex:
    for src, obj, rc, err in ex.map(compile_one, srcs + asm):
        if rc != 0:
            failed.append((src, err))
        else:
            objs.append(obj)
print(f"compiled {len(objs)} objects, {len(failed)} failed")
for src, err in failed[:5]:
    print("COMPILE FAIL", os.path.relpath(src, root)); print(err[:800])

ld = os.path.join(root, "STM32H723VGTX_FLASH.ld")
link = [gcc, "-mcpu=cortex-m7", "-mfpu=fpv5-d16", "-mfloat-abi=hard", "-mthumb", "--specs=nano.specs", "--specs=nosys.specs",
        "-T" + ld, "-Wl,--gc-sections", "-Wl,-Map=" + os.path.join(out, "fc.map"), "-Wl,--print-memory-usage",
        "-o", os.path.join(out, "fc.elf")] + objs + ["-lm", "-lc"]
r = subprocess.run(link, capture_output=True, text=True)
text = (r.stdout + r.stderr)
und = sorted({l.strip() for l in text.splitlines() if "undefined reference" in l or "multiple definition" in l})
print("link rc", r.returncode, "issues", len(und))
for l in und[:30]:
    print(l[:220])
for l in text.splitlines():
    if "Memory region" in l or "RAM" in l or "FLASH" in l or "overflow" in l.lower():
        print(l[:160])
sys.exit(0 if not failed and r.returncode == 0 else 1)
