## This cmakelist.txt file was generated from OnAfterCodegen.m build hook from
## the Pixhawk Pilot Support Package

add_definitions(
-DMODEL=px4_swashplateless_2016a -DNUMST=2 -DNCSTATES=0 -DHAVESTDIO -DUNIX -DON_TARGET_WAIT_FOR_START=0 -DONESTEPFCN=1 -DEXT_MODE=1 -DTERMFCN=1 -DMAT_FILE=0 -DMULTI_INSTANCE_CODE=0 -DINTEGER_CODE=0 -DMT=0 -DCLASSIC_INTERFACE=0 -DALLOCATIONFCN=0 -DTID01EQ=0 -D__MW_TARGET_USE_HARDWARE_RESOURCES_H__ -DUSE_STDPERIPH_DRIVER -DPX4 -DCONFIG_ARCH_BOARD_PX4FMU_V2 -DGIT_VERSION=d3398270e101359de9687c2d077d991c9892f573 -D__DF_NUTTX -D__PX4_NUTTX -DARM_MATH_CM4=1 -D__FPU_PRESENT=1 -D__FPU_USED=1 -DHSE_VALUE=8000000 -DNULL=0 -DEXIT_FAILURE=1 -DEXTMODE_DISABLETESTING -DEXTMODE_DISABLEPRINTF -DRT -DSTACK_SIZE=64 -DEXT_MODE_BKGND -DBAUD_RATE=\"115200\" -DEXT_MODE_DESCRIPTOR=\"ttyACM0\" )

include_directories(
"/usr/local/MATLAB/R2016a/toolbox/psp/pixhawk/+codertarget/+pixhawk/+blocks"
"/usr/local/MATLAB/R2016a/simulink/include/sf_runtime"
"/media/humma/Windows/Users/Humma/Desktop/EMECH/MATLAB/px4_swashplateless_2016a_ert_rtw"
"/media/humma/Windows/Users/Humma/Desktop/EMECH/MATLAB"
"/home/humma/Desktop"
"/usr/local/MATLAB/R2016a/extern/include"
"/usr/local/MATLAB/R2016a/simulink/include"
"/usr/local/MATLAB/R2016a/rtw/c/src"
"/usr/local/MATLAB/R2016a/rtw/c/src/ext_mode/common"
"/usr/local/MATLAB/R2016a/rtw/c/ert"
"/usr/local/MATLAB/R2016a/toolbox/coder/rtiostream/src/utils"
"/usr/local/MATLAB/R2016a/rtw/c/src/ext_mode/serial"
"/usr/local/MATLAB/R2016a/toolbox/psp/pixhawk/include"
)

px4_add_module(
MODULE modules__px4_simulink_app
MAIN px4_simulink_app
STACK_MAIN 2000
SRCS
   PX4PSP_ADC.c
   ert_main.c
   px4_swashplateless_2016a.c
   rtGetInf.c
   rtGetNaN.c
   rt_nonfinite.c
   ext_svr.c
   updown.c
   ext_work.c
   rtiostream_utils.c
   nuttxinitialize.c
   PX4_ext_serial_pkt.c
   PX4_rtiostream_serial_interface.c
   PX4_ext_svr_serial_transport.c
   PX4_rtiostream_serial.c
DEPENDS
    platforms__common
)

# Over-ride compile flags here: 
set(SL_CUSTOM_FLAGS  -MD -c -std=gnu99 $(ARCHCWARNINGS) $(ARCHCPUFLAGS) $(ARCHINCLUDES) $(INSTRUMENTATIONDEFINES) $(ARCHDEFINES) -fno-common -nodefaultlibs -nostdlib -Wframe-larger-than=2000 -fvisibility=hidden -include visibility.h -O0 -fno-builtin-printf)



# Macro Flag Expansion
string (REPLACE "$(ARCHCWARNINGS)" "-Wall -Wextra -Wdouble-promotion -Wshadow -Wfloat-equal -Wframe-larger-than=1024 -Wpointer-arith -Wlogical-op -Wmissing-declarations -Wpacked -Wno-unused-parameter -Werror=format-security -Werror=array-bounds -Wfatal-errors -Wformat=1 -Wbad-function-cast -Wstrict-prototypes -Wold-style-declaration -Wmissing-parameter-type -Wmissing-prototypes -Wnested-externs" SL_CUSTOM_FLAGS "${SL_CUSTOM_FLAGS}")
string (REPLACE "$(ARCHCPUFLAGS)" "-mcpu=cortex-m4 -mthumb -march=armv7e-m -mfpu=fpv4-sp-d16 -ffixed-r10 -mfloat-abi=hard" SL_CUSTOM_FLAGS "${SL_CUSTOM_FLAGS}")
string (REPLACE "$(ARCHINCLUDES)" " " SL_CUSTOM_FLAGS "${SL_CUSTOM_FLAGS}")
string (REPLACE "$(INSTRUMENTATIONDEFINES)" " " SL_CUSTOM_FLAGS "${SL_CUSTOM_FLAGS}")
string (REPLACE "$(ARCHDEFINES)" "-DCONFIG_ARCH_BOARD_PX4FMU_V2" SL_CUSTOM_FLAGS "${SL_CUSTOM_FLAGS}")
string (REPLACE ";" " " CMAKE_C_FLAGS "${SL_CUSTOM_FLAGS}")

