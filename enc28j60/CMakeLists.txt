set(NAME enc28j60_driver)

include(../cmake/FreeRTOS_Kernel_import.cmake)

add_library(${NAME} ${CMAKE_CURRENT_SOURCE_DIR}/src/enc28j60.cpp)

target_include_directories(${NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/inc
                                            ${FREERTOS_KERNEL_PATH}/include
                                            ../inc
                                            )


target_link_libraries(${NAME} PRIVATE hal
                        FreeRTOS-Kernel-Heap4
                        pico_stdlib
                        pico_lwip pico_lwip_freertos
                        hardware_spi
                    )

add_compile_definitions(
                        #ENC_DEBUG_ON
                       )
