file(GLOB MODELS_HEADER_LIST CONFIGURE_DEPENDS "${ModelPredictiveControl_SOURCE_DIR}/include/controllers/*.hpp")

add_library(model_library model_predictive_control.cc linear_quadratic_regulator.cc ${MODELS_HEADER_LIST})

target_include_directories(model_library PUBLIC ../include)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  target_link_libraries(model_library PRIVATE Eigen3::Eigen fmt::fmt)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "Test")
  target_link_libraries(model_library PRIVATE Eigen3::Eigen)
endif()

target_compile_features(model_library PUBLIC cxx_std_17)

source_group(
  TREE "${PROJECT_SOURCE_DIR}/include"
  PREFIX "Header Files"
  FILES ${MODELS_HEADER_LIST})
