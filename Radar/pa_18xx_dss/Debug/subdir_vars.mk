################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
CFG_SRCS += \
../dss_pa.cfg 

CMD_SRCS += \
../c674x_linker.cmd \
../dss_pa_linker.cmd 

C_SRCS += \
../dss_config_edma_util.c \
../dss_data_path.c \
../dss_main.c \
../radarProcess.c 

GEN_CMDS += \
./configPkg/linker.cmd 

GEN_FILES += \
./configPkg/linker.cmd \
./configPkg/compiler.opt 

GEN_MISC_DIRS += \
./configPkg/ 

C_DEPS += \
./dss_config_edma_util.d \
./dss_data_path.d \
./dss_main.d \
./radarProcess.d 

GEN_OPTS += \
./configPkg/compiler.opt 

OBJS += \
./dss_config_edma_util.oe674 \
./dss_data_path.oe674 \
./dss_main.oe674 \
./radarProcess.oe674 

GEN_MISC_DIRS__QUOTED += \
"configPkg\" 

OBJS__QUOTED += \
"dss_config_edma_util.oe674" \
"dss_data_path.oe674" \
"dss_main.oe674" \
"radarProcess.oe674" 

C_DEPS__QUOTED += \
"dss_config_edma_util.d" \
"dss_data_path.d" \
"dss_main.d" \
"radarProcess.d" 

GEN_FILES__QUOTED += \
"configPkg\linker.cmd" \
"configPkg\compiler.opt" 

C_SRCS__QUOTED += \
"../dss_config_edma_util.c" \
"../dss_data_path.c" \
"../dss_main.c" \
"../radarProcess.c" 


