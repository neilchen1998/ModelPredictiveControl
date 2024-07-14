file(GLOB CONTROLLERS_HEADER_LIST CONFIGURE_DEPENDS "${ModelPredictiveControl_SOURCE_DIR}/include/controllers/*.hpp")
file(GLOB MODELS_HEADER_LIST CONFIGURE_DEPENDS "${ModelPredictiveControl_SOURCE_DIR}/include/models/*.hpp")

add_library(controller_library model_predictive_control.cc ${CONTROLLERS_HEADER_LIST})
add_library(model_library mass_spring_damper.cc ${MODELS_HEADER_LIST})

target_include_directories(controller_library PUBLIC ../include)
target_include_directories(model_library PUBLIC ../include)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  target_link_libraries(controller_library PRIVATE Eigen3::Eigen fmt::fmt)
  target_link_libraries(model_library PRIVATE Eigen3::Eigen)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "Test")
  target_link_libraries(controller_library PRIVATE Eigen3::Eigen)
  target_link_libraries(model_library PRIVATE Eigen3::Eigen)
endif()

target_compile_features(controller_library PUBLIC cxx_std_17)
target_compile_features(model_library PUBLIC cxx_std_17)

source_group(
  TREE "${PROJECT_SOURCE_DIR}/include"
  PREFIX "Header Files"
  FILES ${CONTROLLERS_HEADER_LIST} ${MODELS_HEADER_LIST})
