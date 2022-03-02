add_library(bmp085
	bmp085.c
)

target_include_directories(bmp085 PUBLIC
	.
)

target_link_libraries(bmp085 pico_stdlib hardware_i2c)