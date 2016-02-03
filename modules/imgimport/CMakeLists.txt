set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include_directories(include ../core/include ${DeckLinkSDK_INCLUDE_PATH})
add_library(ImageImport ${DeckLinkSDK_LIBS}  src/imgimport.cpp src/DeckLinkCapture.cpp src/DeckLinkOpenCv.cpp src/DeckLinkInputCallback.cpp)
add_subdirectory("test")

