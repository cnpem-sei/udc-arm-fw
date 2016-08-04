################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
app/communication_drivers/bsmp/bsmp/src/bsmp.obj: ../app/communication_drivers/bsmp/bsmp/src/bsmp.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.7/bin/armcl" -mv7M3 --code_state=16 --abi=eabi -me -Ooff --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.7/include" --include_path="C:/Users/joao.rosa/git/ARM/app/board_drivers" --include_path="C:/ti/controlSUITE/device_support/f28m36x/v206/MWare" --include_path="C:/Users/joao.rosa/git/ARM/app/communication_drivers/ethernet" --include_path="C:/Users/joao.rosa/git/ARM/app/communication_drivers/ethernet/server_net/includes" --include_path="C:/Users/joao.rosa/git/ARM/app/communication_drivers/ethernet/server_net/uip-1.0" --include_path="C:/Users/joao.rosa/git/ARM/app/communication_drivers/ethernet/server_net/uip-1.0/uip" --include_path="C:/Users/joao.rosa/git/ARM/app/communication_drivers/ethernet/server_net/uip-1.0/apps" -g --gcc --define=ccs --define="_STANDALONE" --diag_wrap=off --display_error_number --diag_warning=225 --gen_func_subsections=on --ual --preproc_with_compile --preproc_dependency="app/communication_drivers/bsmp/bsmp/src/bsmp.pp" --obj_directory="app/communication_drivers/bsmp/bsmp/src" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

app/communication_drivers/bsmp/bsmp/src/client.obj: ../app/communication_drivers/bsmp/bsmp/src/client.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.7/bin/armcl" -mv7M3 --code_state=16 --abi=eabi -me -Ooff --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.7/include" --include_path="C:/Users/joao.rosa/git/ARM/app/board_drivers" --include_path="C:/ti/controlSUITE/device_support/f28m36x/v206/MWare" --include_path="C:/Users/joao.rosa/git/ARM/app/communication_drivers/ethernet" --include_path="C:/Users/joao.rosa/git/ARM/app/communication_drivers/ethernet/server_net/includes" --include_path="C:/Users/joao.rosa/git/ARM/app/communication_drivers/ethernet/server_net/uip-1.0" --include_path="C:/Users/joao.rosa/git/ARM/app/communication_drivers/ethernet/server_net/uip-1.0/uip" --include_path="C:/Users/joao.rosa/git/ARM/app/communication_drivers/ethernet/server_net/uip-1.0/apps" -g --gcc --define=ccs --define="_STANDALONE" --diag_wrap=off --display_error_number --diag_warning=225 --gen_func_subsections=on --ual --preproc_with_compile --preproc_dependency="app/communication_drivers/bsmp/bsmp/src/client.pp" --obj_directory="app/communication_drivers/bsmp/bsmp/src" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

app/communication_drivers/bsmp/bsmp/src/server.obj: ../app/communication_drivers/bsmp/bsmp/src/server.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.7/bin/armcl" -mv7M3 --code_state=16 --abi=eabi -me -Ooff --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.7/include" --include_path="C:/Users/joao.rosa/git/ARM/app/board_drivers" --include_path="C:/ti/controlSUITE/device_support/f28m36x/v206/MWare" --include_path="C:/Users/joao.rosa/git/ARM/app/communication_drivers/ethernet" --include_path="C:/Users/joao.rosa/git/ARM/app/communication_drivers/ethernet/server_net/includes" --include_path="C:/Users/joao.rosa/git/ARM/app/communication_drivers/ethernet/server_net/uip-1.0" --include_path="C:/Users/joao.rosa/git/ARM/app/communication_drivers/ethernet/server_net/uip-1.0/uip" --include_path="C:/Users/joao.rosa/git/ARM/app/communication_drivers/ethernet/server_net/uip-1.0/apps" -g --gcc --define=ccs --define="_STANDALONE" --diag_wrap=off --display_error_number --diag_warning=225 --gen_func_subsections=on --ual --preproc_with_compile --preproc_dependency="app/communication_drivers/bsmp/bsmp/src/server.pp" --obj_directory="app/communication_drivers/bsmp/bsmp/src" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

app/communication_drivers/bsmp/bsmp/src/server_priv.obj: ../app/communication_drivers/bsmp/bsmp/src/server_priv.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.7/bin/armcl" -mv7M3 --code_state=16 --abi=eabi -me -Ooff --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.7/include" --include_path="C:/Users/joao.rosa/git/ARM/app/board_drivers" --include_path="C:/ti/controlSUITE/device_support/f28m36x/v206/MWare" --include_path="C:/Users/joao.rosa/git/ARM/app/communication_drivers/ethernet" --include_path="C:/Users/joao.rosa/git/ARM/app/communication_drivers/ethernet/server_net/includes" --include_path="C:/Users/joao.rosa/git/ARM/app/communication_drivers/ethernet/server_net/uip-1.0" --include_path="C:/Users/joao.rosa/git/ARM/app/communication_drivers/ethernet/server_net/uip-1.0/uip" --include_path="C:/Users/joao.rosa/git/ARM/app/communication_drivers/ethernet/server_net/uip-1.0/apps" -g --gcc --define=ccs --define="_STANDALONE" --diag_wrap=off --display_error_number --diag_warning=225 --gen_func_subsections=on --ual --preproc_with_compile --preproc_dependency="app/communication_drivers/bsmp/bsmp/src/server_priv.pp" --obj_directory="app/communication_drivers/bsmp/bsmp/src" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


