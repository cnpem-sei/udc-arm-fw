################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
app/communication_drivers/usb_device/superv_cmd.obj: ../app/communication_drivers/usb_device/superv_cmd.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.7/bin/armcl" --code_state=32 --abi=eabi -me -O2 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.7/include" --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="app/communication_drivers/usb_device/superv_cmd.pp" --obj_directory="app/communication_drivers/usb_device" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

app/communication_drivers/usb_device/usb_dev_serial.obj: ../app/communication_drivers/usb_device/usb_dev_serial.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.7/bin/armcl" --code_state=32 --abi=eabi -me -O2 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.7/include" --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="app/communication_drivers/usb_device/usb_dev_serial.pp" --obj_directory="app/communication_drivers/usb_device" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

app/communication_drivers/usb_device/usb_serial_structs.obj: ../app/communication_drivers/usb_device/usb_serial_structs.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.7/bin/armcl" --code_state=32 --abi=eabi -me -O2 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.7/include" --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="app/communication_drivers/usb_device/usb_serial_structs.pp" --obj_directory="app/communication_drivers/usb_device" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


