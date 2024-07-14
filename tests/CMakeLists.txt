FetchContent_Declare(
  catch
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG v2.13.6)
FetchContent_MakeAvailable(catch)

add_executable(models_testlib models_testlib.cc)

target_compile_features(models_testlib PRIVATE cxx_std_17)

target_link_libraries(models_testlib PRIVATE Catch2::Catch2 Eigen3::Eigen model_library)

add_test(NAME models_testlib COMMAND models_testlib)
