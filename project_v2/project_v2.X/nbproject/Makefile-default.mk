#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/project_v2.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/project_v2.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=FreeRTOS/portable/MPLAB/PIC24_dsPIC/port.c FreeRTOS/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.S FreeRTOS/portable/MemMang/heap_1.c FreeRTOS/croutine.c FreeRTOS/event_groups.c FreeRTOS/list.c FreeRTOS/queue.c FreeRTOS/stream_buffer.c FreeRTOS/tasks.c FreeRTOS/timers.c uart.c init.c adc.c led_module.c main_current.c button_module.c countdown_module.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/FreeRTOS/portable/MPLAB/PIC24_dsPIC/port.o ${OBJECTDIR}/FreeRTOS/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.o ${OBJECTDIR}/FreeRTOS/portable/MemMang/heap_1.o ${OBJECTDIR}/FreeRTOS/croutine.o ${OBJECTDIR}/FreeRTOS/event_groups.o ${OBJECTDIR}/FreeRTOS/list.o ${OBJECTDIR}/FreeRTOS/queue.o ${OBJECTDIR}/FreeRTOS/stream_buffer.o ${OBJECTDIR}/FreeRTOS/tasks.o ${OBJECTDIR}/FreeRTOS/timers.o ${OBJECTDIR}/uart.o ${OBJECTDIR}/init.o ${OBJECTDIR}/adc.o ${OBJECTDIR}/led_module.o ${OBJECTDIR}/main_current.o ${OBJECTDIR}/button_module.o ${OBJECTDIR}/countdown_module.o
POSSIBLE_DEPFILES=${OBJECTDIR}/FreeRTOS/portable/MPLAB/PIC24_dsPIC/port.o.d ${OBJECTDIR}/FreeRTOS/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.o.d ${OBJECTDIR}/FreeRTOS/portable/MemMang/heap_1.o.d ${OBJECTDIR}/FreeRTOS/croutine.o.d ${OBJECTDIR}/FreeRTOS/event_groups.o.d ${OBJECTDIR}/FreeRTOS/list.o.d ${OBJECTDIR}/FreeRTOS/queue.o.d ${OBJECTDIR}/FreeRTOS/stream_buffer.o.d ${OBJECTDIR}/FreeRTOS/tasks.o.d ${OBJECTDIR}/FreeRTOS/timers.o.d ${OBJECTDIR}/uart.o.d ${OBJECTDIR}/init.o.d ${OBJECTDIR}/adc.o.d ${OBJECTDIR}/led_module.o.d ${OBJECTDIR}/main_current.o.d ${OBJECTDIR}/button_module.o.d ${OBJECTDIR}/countdown_module.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/FreeRTOS/portable/MPLAB/PIC24_dsPIC/port.o ${OBJECTDIR}/FreeRTOS/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.o ${OBJECTDIR}/FreeRTOS/portable/MemMang/heap_1.o ${OBJECTDIR}/FreeRTOS/croutine.o ${OBJECTDIR}/FreeRTOS/event_groups.o ${OBJECTDIR}/FreeRTOS/list.o ${OBJECTDIR}/FreeRTOS/queue.o ${OBJECTDIR}/FreeRTOS/stream_buffer.o ${OBJECTDIR}/FreeRTOS/tasks.o ${OBJECTDIR}/FreeRTOS/timers.o ${OBJECTDIR}/uart.o ${OBJECTDIR}/init.o ${OBJECTDIR}/adc.o ${OBJECTDIR}/led_module.o ${OBJECTDIR}/main_current.o ${OBJECTDIR}/button_module.o ${OBJECTDIR}/countdown_module.o

# Source Files
SOURCEFILES=FreeRTOS/portable/MPLAB/PIC24_dsPIC/port.c FreeRTOS/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.S FreeRTOS/portable/MemMang/heap_1.c FreeRTOS/croutine.c FreeRTOS/event_groups.c FreeRTOS/list.c FreeRTOS/queue.c FreeRTOS/stream_buffer.c FreeRTOS/tasks.c FreeRTOS/timers.c uart.c init.c adc.c led_module.c main_current.c button_module.c countdown_module.c



CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk ${DISTDIR}/project_v2.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=24FJ256GA702
MP_LINKER_FILE_OPTION=,--script=p24FJ256GA702.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/FreeRTOS/portable/MPLAB/PIC24_dsPIC/port.o: FreeRTOS/portable/MPLAB/PIC24_dsPIC/port.c  .generated_files/flags/default/8546ca8f5a3b7e536b1bc19028b27034797acca5 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FreeRTOS/portable/MPLAB/PIC24_dsPIC" 
	@${RM} ${OBJECTDIR}/FreeRTOS/portable/MPLAB/PIC24_dsPIC/port.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/portable/MPLAB/PIC24_dsPIC/port.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FreeRTOS/portable/MPLAB/PIC24_dsPIC/port.c  -o ${OBJECTDIR}/FreeRTOS/portable/MPLAB/PIC24_dsPIC/port.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/FreeRTOS/portable/MPLAB/PIC24_dsPIC/port.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/FreeRTOS/portable/MemMang/heap_1.o: FreeRTOS/portable/MemMang/heap_1.c  .generated_files/flags/default/9e490a7be75073562f3021495501c33c96fa9abe .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FreeRTOS/portable/MemMang" 
	@${RM} ${OBJECTDIR}/FreeRTOS/portable/MemMang/heap_1.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/portable/MemMang/heap_1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FreeRTOS/portable/MemMang/heap_1.c  -o ${OBJECTDIR}/FreeRTOS/portable/MemMang/heap_1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/FreeRTOS/portable/MemMang/heap_1.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/FreeRTOS/croutine.o: FreeRTOS/croutine.c  .generated_files/flags/default/64a383f14b5559b764bcf8f1cdd892a05fd934b6 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FreeRTOS" 
	@${RM} ${OBJECTDIR}/FreeRTOS/croutine.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/croutine.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FreeRTOS/croutine.c  -o ${OBJECTDIR}/FreeRTOS/croutine.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/FreeRTOS/croutine.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/FreeRTOS/event_groups.o: FreeRTOS/event_groups.c  .generated_files/flags/default/58044fa9a08abf2ff2b95c69cb477675b8a3277d .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FreeRTOS" 
	@${RM} ${OBJECTDIR}/FreeRTOS/event_groups.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/event_groups.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FreeRTOS/event_groups.c  -o ${OBJECTDIR}/FreeRTOS/event_groups.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/FreeRTOS/event_groups.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/FreeRTOS/list.o: FreeRTOS/list.c  .generated_files/flags/default/f33efd7f63f4370c1f4aad5b2a309db51bc36605 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FreeRTOS" 
	@${RM} ${OBJECTDIR}/FreeRTOS/list.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/list.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FreeRTOS/list.c  -o ${OBJECTDIR}/FreeRTOS/list.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/FreeRTOS/list.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/FreeRTOS/queue.o: FreeRTOS/queue.c  .generated_files/flags/default/a326b122028996fbed961cd4267e79e6fd7898fd .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FreeRTOS" 
	@${RM} ${OBJECTDIR}/FreeRTOS/queue.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/queue.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FreeRTOS/queue.c  -o ${OBJECTDIR}/FreeRTOS/queue.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/FreeRTOS/queue.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/FreeRTOS/stream_buffer.o: FreeRTOS/stream_buffer.c  .generated_files/flags/default/2db9f763863a55f75eecbd73d05e3a8f776783f5 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FreeRTOS" 
	@${RM} ${OBJECTDIR}/FreeRTOS/stream_buffer.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/stream_buffer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FreeRTOS/stream_buffer.c  -o ${OBJECTDIR}/FreeRTOS/stream_buffer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/FreeRTOS/stream_buffer.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/FreeRTOS/tasks.o: FreeRTOS/tasks.c  .generated_files/flags/default/38fbf748386076b2e2909202192498dcdd3e38c6 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FreeRTOS" 
	@${RM} ${OBJECTDIR}/FreeRTOS/tasks.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/tasks.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FreeRTOS/tasks.c  -o ${OBJECTDIR}/FreeRTOS/tasks.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/FreeRTOS/tasks.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/FreeRTOS/timers.o: FreeRTOS/timers.c  .generated_files/flags/default/d39ed53ce0c9505d2bb3ebdd37b182c63f2e15c1 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FreeRTOS" 
	@${RM} ${OBJECTDIR}/FreeRTOS/timers.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/timers.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FreeRTOS/timers.c  -o ${OBJECTDIR}/FreeRTOS/timers.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/FreeRTOS/timers.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/uart.o: uart.c  .generated_files/flags/default/201c3fb7d2f6974f102f228517cadf20c4f47fc9 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/uart.o.d 
	@${RM} ${OBJECTDIR}/uart.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  uart.c  -o ${OBJECTDIR}/uart.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/uart.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/init.o: init.c  .generated_files/flags/default/e25cd82c602ef70d0bb9063a3e3f79306ea90484 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/init.o.d 
	@${RM} ${OBJECTDIR}/init.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  init.c  -o ${OBJECTDIR}/init.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/init.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/adc.o: adc.c  .generated_files/flags/default/56f6bc0b6d910c1b8fd384b064becc88ed9a59e9 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/adc.o.d 
	@${RM} ${OBJECTDIR}/adc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  adc.c  -o ${OBJECTDIR}/adc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/adc.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/led_module.o: led_module.c  .generated_files/flags/default/fe3cd7a19cc79946738feaecda6b457cf6e89873 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/led_module.o.d 
	@${RM} ${OBJECTDIR}/led_module.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  led_module.c  -o ${OBJECTDIR}/led_module.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/led_module.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/main_current.o: main_current.c  .generated_files/flags/default/8c70a824258eb3ba74a6dd90b1528717d3e18bf3 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main_current.o.d 
	@${RM} ${OBJECTDIR}/main_current.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  main_current.c  -o ${OBJECTDIR}/main_current.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/main_current.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/button_module.o: button_module.c  .generated_files/flags/default/6d42cbb497e02abf0b53277ff2e791cde2dbe429 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/button_module.o.d 
	@${RM} ${OBJECTDIR}/button_module.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  button_module.c  -o ${OBJECTDIR}/button_module.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/button_module.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/countdown_module.o: countdown_module.c  .generated_files/flags/default/67fad67223fd9d26bc1e3858e9904aa51f4fc1d7 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/countdown_module.o.d 
	@${RM} ${OBJECTDIR}/countdown_module.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  countdown_module.c  -o ${OBJECTDIR}/countdown_module.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/countdown_module.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
else
${OBJECTDIR}/FreeRTOS/portable/MPLAB/PIC24_dsPIC/port.o: FreeRTOS/portable/MPLAB/PIC24_dsPIC/port.c  .generated_files/flags/default/7f25edd2f9c7aa836e03e9d81845c6009e2c3b2e .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FreeRTOS/portable/MPLAB/PIC24_dsPIC" 
	@${RM} ${OBJECTDIR}/FreeRTOS/portable/MPLAB/PIC24_dsPIC/port.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/portable/MPLAB/PIC24_dsPIC/port.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FreeRTOS/portable/MPLAB/PIC24_dsPIC/port.c  -o ${OBJECTDIR}/FreeRTOS/portable/MPLAB/PIC24_dsPIC/port.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/FreeRTOS/portable/MPLAB/PIC24_dsPIC/port.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/FreeRTOS/portable/MemMang/heap_1.o: FreeRTOS/portable/MemMang/heap_1.c  .generated_files/flags/default/4e1b8bdf96549a324cfd4e98c72b524ecf946bf1 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FreeRTOS/portable/MemMang" 
	@${RM} ${OBJECTDIR}/FreeRTOS/portable/MemMang/heap_1.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/portable/MemMang/heap_1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FreeRTOS/portable/MemMang/heap_1.c  -o ${OBJECTDIR}/FreeRTOS/portable/MemMang/heap_1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/FreeRTOS/portable/MemMang/heap_1.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/FreeRTOS/croutine.o: FreeRTOS/croutine.c  .generated_files/flags/default/7923d77edeee2f7611830dc43d58295726348814 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FreeRTOS" 
	@${RM} ${OBJECTDIR}/FreeRTOS/croutine.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/croutine.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FreeRTOS/croutine.c  -o ${OBJECTDIR}/FreeRTOS/croutine.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/FreeRTOS/croutine.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/FreeRTOS/event_groups.o: FreeRTOS/event_groups.c  .generated_files/flags/default/fc1214f52200c9cfb3687a78fb2056ecfca42152 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FreeRTOS" 
	@${RM} ${OBJECTDIR}/FreeRTOS/event_groups.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/event_groups.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FreeRTOS/event_groups.c  -o ${OBJECTDIR}/FreeRTOS/event_groups.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/FreeRTOS/event_groups.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/FreeRTOS/list.o: FreeRTOS/list.c  .generated_files/flags/default/2e2cd1ca04b847c8fd906a795e99faaf929d61d6 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FreeRTOS" 
	@${RM} ${OBJECTDIR}/FreeRTOS/list.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/list.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FreeRTOS/list.c  -o ${OBJECTDIR}/FreeRTOS/list.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/FreeRTOS/list.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/FreeRTOS/queue.o: FreeRTOS/queue.c  .generated_files/flags/default/4ba35aff37cc5897b4c7d46bac5c1d1f1701d28c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FreeRTOS" 
	@${RM} ${OBJECTDIR}/FreeRTOS/queue.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/queue.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FreeRTOS/queue.c  -o ${OBJECTDIR}/FreeRTOS/queue.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/FreeRTOS/queue.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/FreeRTOS/stream_buffer.o: FreeRTOS/stream_buffer.c  .generated_files/flags/default/87917b148a21e5c6fdada51597a2c85b2a7f9ea9 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FreeRTOS" 
	@${RM} ${OBJECTDIR}/FreeRTOS/stream_buffer.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/stream_buffer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FreeRTOS/stream_buffer.c  -o ${OBJECTDIR}/FreeRTOS/stream_buffer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/FreeRTOS/stream_buffer.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/FreeRTOS/tasks.o: FreeRTOS/tasks.c  .generated_files/flags/default/4f2ea4e0fd3edf7ad99179ef6f7207a321ad45ab .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FreeRTOS" 
	@${RM} ${OBJECTDIR}/FreeRTOS/tasks.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/tasks.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FreeRTOS/tasks.c  -o ${OBJECTDIR}/FreeRTOS/tasks.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/FreeRTOS/tasks.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/FreeRTOS/timers.o: FreeRTOS/timers.c  .generated_files/flags/default/58df58b720b0850ca2f39457b817cd096582e087 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FreeRTOS" 
	@${RM} ${OBJECTDIR}/FreeRTOS/timers.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/timers.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FreeRTOS/timers.c  -o ${OBJECTDIR}/FreeRTOS/timers.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/FreeRTOS/timers.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/uart.o: uart.c  .generated_files/flags/default/446cd80b02097b47315101488b593064b81a97f4 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/uart.o.d 
	@${RM} ${OBJECTDIR}/uart.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  uart.c  -o ${OBJECTDIR}/uart.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/uart.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/init.o: init.c  .generated_files/flags/default/296a5f38922b2ec8b78622a151f0e09ac9f48653 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/init.o.d 
	@${RM} ${OBJECTDIR}/init.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  init.c  -o ${OBJECTDIR}/init.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/init.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/adc.o: adc.c  .generated_files/flags/default/5f3c597173b6f6dc86535eb3d4e6dac0089d9bf4 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/adc.o.d 
	@${RM} ${OBJECTDIR}/adc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  adc.c  -o ${OBJECTDIR}/adc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/adc.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/led_module.o: led_module.c  .generated_files/flags/default/3a8ba215e34cf398a446e1a1daa63f18d910cc0 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/led_module.o.d 
	@${RM} ${OBJECTDIR}/led_module.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  led_module.c  -o ${OBJECTDIR}/led_module.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/led_module.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/main_current.o: main_current.c  .generated_files/flags/default/15eda52b0a521c8b0724da48dc16c2ac7bec489e .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main_current.o.d 
	@${RM} ${OBJECTDIR}/main_current.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  main_current.c  -o ${OBJECTDIR}/main_current.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/main_current.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/button_module.o: button_module.c  .generated_files/flags/default/7dcda464df727b22645afd4af6d1cf6b6f8a1072 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/button_module.o.d 
	@${RM} ${OBJECTDIR}/button_module.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  button_module.c  -o ${OBJECTDIR}/button_module.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/button_module.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/countdown_module.o: countdown_module.c  .generated_files/flags/default/fbff450b69fc0ccd52821b11a64ac8d9d7990d47 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/countdown_module.o.d 
	@${RM} ${OBJECTDIR}/countdown_module.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  countdown_module.c  -o ${OBJECTDIR}/countdown_module.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/countdown_module.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/FreeRTOS/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.o: FreeRTOS/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.S  .generated_files/flags/default/3416d13d585b346f54279d6897fc2fd7703dcaee .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FreeRTOS/portable/MPLAB/PIC24_dsPIC" 
	@${RM} ${OBJECTDIR}/FreeRTOS/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  FreeRTOS/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.S  -o ${OBJECTDIR}/FreeRTOS/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/FreeRTOS/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.o.d"  -D__DEBUG   -omf=elf -DXPRJ_default=$(CND_CONF)    -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC" -Wa,-MD,"${OBJECTDIR}/FreeRTOS/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.o.asm.d",--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,,-g,--no-relax$(MP_EXTRA_AS_POST)  -mdfp="${DFP_DIR}/xc16"
	
else
${OBJECTDIR}/FreeRTOS/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.o: FreeRTOS/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.S  .generated_files/flags/default/f7adcbd741c0db20a4c71065aa24151debbf45c9 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FreeRTOS/portable/MPLAB/PIC24_dsPIC" 
	@${RM} ${OBJECTDIR}/FreeRTOS/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  FreeRTOS/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.S  -o ${OBJECTDIR}/FreeRTOS/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/FreeRTOS/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.o.d"  -omf=elf -DXPRJ_default=$(CND_CONF)    -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC" -Wa,-MD,"${OBJECTDIR}/FreeRTOS/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.o.asm.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax$(MP_EXTRA_AS_POST)  -mdfp="${DFP_DIR}/xc16"
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${DISTDIR}/project_v2.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o ${DISTDIR}/project_v2.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG=__DEBUG   -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC"  -mreserve=data@0x800:0x81B -mreserve=data@0x81C:0x81D -mreserve=data@0x81E:0x81F -mreserve=data@0x820:0x821 -mreserve=data@0x822:0x823 -mreserve=data@0x824:0x827 -mreserve=data@0x82A:0x84F   -Wl,--local-stack,,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D__DEBUG=__DEBUG,,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,${DISTDIR}/memoryfile.xml$(MP_EXTRA_LD_POST)  -mdfp="${DFP_DIR}/xc16" 
	
else
${DISTDIR}/project_v2.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o ${DISTDIR}/project_v2.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"./FreeRTOS/include" -I"." -I"./FreeRTOS/portable/MPLAB/PIC24_dsPIC" -Wl,--local-stack,,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,${DISTDIR}/memoryfile.xml$(MP_EXTRA_LD_POST)  -mdfp="${DFP_DIR}/xc16" 
	${MP_CC_DIR}\\xc16-bin2hex ${DISTDIR}/project_v2.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf   -mdfp="${DFP_DIR}/xc16" 
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${OBJECTDIR}
	${RM} -r ${DISTDIR}

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(wildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
