
add_executable(construction_time construction_time.cpp)
target_link_libraries(construction_time KDTree)

add_executable(error_test error_test.cpp)
target_link_libraries(error_test KDTree)

add_executable(toy_test toy_test.cpp)
target_link_libraries(toy_test KDTree)

add_executable(read_mesh read_mesh.cpp)
target_link_libraries(read_mesh KDTree)

add_executable(generate_input generate_input.cpp)
target_link_libraries(generate_input KDTree)