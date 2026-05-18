add_library( u2u
    u2u.h
    u2u.c
    u2u_client_profile.h
    u2u_client_profile.c
    u2u_HAL_pico.h
    u2u_HAL_pico.c
    )

target_include_directories(u2u PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(u2u pico_stdlib hardware_uart)
