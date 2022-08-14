project(imageBasics)

add_executable(imageBasics imageBasics.cpp)

target_link_libraries(imageBasics ${OpenCV_LIBS})

add_executable(undistortImg undistortImg.cpp)

target_link_libraries(undistortImg ${OpenCV_LIBS})