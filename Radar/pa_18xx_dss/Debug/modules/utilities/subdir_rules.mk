################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
modules/utilities/%.oe674: ../modules/utilities/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C6000 Compiler'
	"C:/ti/ti-cgt-c6000_8.3.3/bin/cl6x" -mv6740 --abi=eabi -O3 --include_path="C:/ti/Thesis_radar/pa_18xx_dss" --include_path="C:/ti/Thesis_radar/pa_18xx_dss/modules/utilities" --include_path="C:/ti/mmwave_sdk_03_02_00_04/packages" --include_path="C:/ti/mathlib_c674x_3_1_2_1/packages" --include_path="C:/ti/dsplib_c674x_3_4_0_0/packages/ti/dsplib/src/DSPF_sp_fftSPxSP/c674" --include_path="C:/ti/dsplib_c64Px_3_4_0_0/packages/ti/dsplib/src/DSP_fft16x16/c64P" --include_path="C:/ti/dsplib_c64Px_3_4_0_0/packages/ti/dsplib/src/DSP_fft16x16_imre/c64P" --include_path="C:/ti/dsplib_c64Px_3_4_0_0/packages/ti/dsplib/src/DSP_fft32x32/c64P" --include_path="C:/ti/ti-cgt-c6000_8.3.3/include" --define=SOC_XWR18XX --define=SUBSYS_DSS --define=ENABLE_ADVANCED_FRAME --define=MMWAVE_L3RAM_SIZE=0x100000 --define=MMWAVE_L3RAM_NUM_BANK=8 --define=MMWAVE_SHMEM_BANK_SIZE=0x20000 --define=DOWNLOAD_FROM_CCS --define=DebugP_ASSERT_ENABLED -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --disable_push_pop --obj_extension=.oe674 --preproc_with_compile --preproc_dependency="modules/utilities/$(basename $(<F)).d_raw" --obj_directory="modules/utilities" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


