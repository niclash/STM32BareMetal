set(TGT first)
add_executable(${TGT} main.c)
target_link_options(${TGT} PRIVATE ${COMMON_LINK_OPTIONS})
add_custom_command(TARGET ${TGT} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${TGT}> ${TGT}.bin
        COMMAND ${CMAKE_OBJDUMP} -h -S $<TARGET_FILE:${TGT}> > ${TGT}.list
)