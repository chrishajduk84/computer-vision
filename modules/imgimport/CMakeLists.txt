set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include_directories(include ../core/include ${DeckLinkSDK_INCLUDE_PATH})
ADD_DEFINITIONS("-DBOOST_LOG_DYN_LINK")
add_library(ImageImport ${BOOST_LIBRARIES} ${DeckLinkSDK_LIBS}  src/imgimport.cpp src/DeckLinkCapture.cpp src/DeckLinkOpenCv.cpp src/DeckLinkInputCallback.cpp)
target_compile_features(ImageImport PRIVATE)
add_subdirectory("test")

