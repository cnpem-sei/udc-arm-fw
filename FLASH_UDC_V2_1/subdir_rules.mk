################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
main.obj: ../main.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/home/allefpablo/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.4.LTS/bin/armcl" -mv7M3 --code_state=16 --abi=eabi -me -O2 --include_path="/home/allefpablo/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.4.LTS/include" --include_path="/home/allefpablo/lnls-elp/ARM/app/board_drivers" --include_path="/home/allefpablo/lnls-elp/controlsuite/arm/MWare" --include_path="/home/allefpablo/lnls-elp/ARM/app" --include_path="/home/allefpablo/lnls-elp/ARM/app/communication_drivers/ethernet" --include_path="/home/allefpablo/lnls-elp/ARM/app/communication_drivers/ethernet/server_net/includes" --include_path="/home/allefpablo/lnls-elp/ARM/app/communication_drivers/ethernet/server_net/uip-1.0" --include_path="/home/allefpablo/lnls-elp/ARM/app/communication_drivers/ethernet/server_net/uip-1.0/uip" --include_path="/home/allefpablo/lnls-elp/ARM/app/communication_drivers/ethernet/server_net/uip-1.0/apps" -g --gcc --define=ccs --define="_STANDALONE" --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --ual --preproc_with_compile --preproc_dependency="main.d_raw" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

startup_ccs.obj: ../startup_ccs.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/home/allefpablo/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.4.LTS/bin/armcl" -mv7M3 --code_state=16 --abi=eabi -me -O2 --include_path="/home/allefpablo/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.4.LTS/include" --include_path="/home/allefpablo/lnls-elp/ARM/app/board_drivers" --include_path="/home/allefpablo/lnls-elp/controlsuite/arm/MWare" --include_path="/home/allefpablo/lnls-elp/ARM/app" --include_path="/home/allefpablo/lnls-elp/ARM/app/communication_drivers/ethernet" --include_path="/home/allefpablo/lnls-elp/ARM/app/communication_drivers/ethernet/server_net/includes" --include_path="/home/allefpablo/lnls-elp/ARM/app/communication_drivers/ethernet/server_net/uip-1.0" --include_path="/home/allefpablo/lnls-elp/ARM/app/communication_drivers/ethernet/server_net/uip-1.0/uip" --include_path="/home/allefpablo/lnls-elp/ARM/app/communication_drivers/ethernet/server_net/uip-1.0/apps" -g --gcc --define=ccs --define="_STANDALONE" --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --ual --preproc_with_compile --preproc_dependency="startup_ccs.d_raw" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

uartstdio.obj: ../uartstdio.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/home/allefpablo/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.4.LTS/bin/armcl" -mv7M3 --code_state=16 --abi=eabi -me -O2 --include_path="/home/allefpablo/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.4.LTS/include" --include_path="/home/allefpablo/lnls-elp/ARM/app/board_drivers" --include_path="/home/allefpablo/lnls-elp/controlsuite/arm/MWare" --include_path="/home/allefpablo/lnls-elp/ARM/app" --include_path="/home/allefpablo/lnls-elp/ARM/app/communication_drivers/ethernet" --include_path="/home/allefpablo/lnls-elp/ARM/app/communication_drivers/ethernet/server_net/includes" --include_path="/home/allefpablo/lnls-elp/ARM/app/communication_drivers/ethernet/server_net/uip-1.0" --include_path="/home/allefpablo/lnls-elp/ARM/app/communication_drivers/ethernet/server_net/uip-1.0/uip" --include_path="/home/allefpablo/lnls-elp/ARM/app/communication_drivers/ethernet/server_net/uip-1.0/apps" -g --gcc --define=ccs --define="_STANDALONE" --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --ual --preproc_with_compile --preproc_dependency="uartstdio.d_raw" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '


