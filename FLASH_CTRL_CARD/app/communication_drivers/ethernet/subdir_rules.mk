################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
app/communication_drivers/ethernet/ethernet_uip.obj: ../app/communication_drivers/ethernet/ethernet_uip.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.4/bin/armcl" -mv7M3 --code_state=16 --abi=eabi -me -Ooff --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.4/include" --include_path="R:/LNLS/Grupos/ELP/Doctec/Sirius/Projetos/Controlador Digital/Prototipos/V2.0/UDC/Software/Firmware Base/ARM/app/board_drivers" --include_path="C:/ti/controlSUITE/device_support/f28m36x/v206/MWare" --include_path="R:/LNLS/Grupos/ELP/Doctec/Sirius/Projetos/Controlador Digital/Prototipos/V2.0/UDC/Software/Firmware Base/ARM/app/communication_drivers/ethernet" --include_path="R:/LNLS/Grupos/ELP/Doctec/Sirius/Projetos/Controlador Digital/Prototipos/V2.0/UDC/Software/Firmware Base/ARM/app/communication_drivers/ethernet/server_net/includes" --include_path="R:/LNLS/Grupos/ELP/Doctec/Sirius/Projetos/Controlador Digital/Prototipos/V2.0/UDC/Software/Firmware Base/ARM/app/communication_drivers/ethernet/server_net/uip-1.0" --include_path="R:/LNLS/Grupos/ELP/Doctec/Sirius/Projetos/Controlador Digital/Prototipos/V2.0/UDC/Software/Firmware Base/ARM/app/communication_drivers/ethernet/server_net/uip-1.0/uip" --include_path="R:/LNLS/Grupos/ELP/Doctec/Sirius/Projetos/Controlador Digital/Prototipos/V2.0/UDC/Software/Firmware Base/ARM/app/communication_drivers/ethernet/server_net/uip-1.0/apps" -g --gcc --define=ccs --diag_wrap=off --display_error_number --diag_warning=225 --gen_func_subsections=on --ual --preproc_with_compile --preproc_dependency="app/communication_drivers/ethernet/ethernet_uip.pp" --obj_directory="app/communication_drivers/ethernet" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


