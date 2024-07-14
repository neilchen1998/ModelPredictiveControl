add_executable(app app.cpp)
target_compile_features(app PRIVATE cxx_std_17)

target_link_libraries(app PRIVATE model_library controller_library io_library)
