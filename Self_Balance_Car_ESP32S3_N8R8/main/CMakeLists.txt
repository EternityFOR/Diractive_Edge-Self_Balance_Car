idf_component_register(SRCS "smart_car.c" 
                       INCLUDE_DIRS "include"
                       EMBED_TXTFILES ${project_dir}/sound_test/car_2.raw)
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")