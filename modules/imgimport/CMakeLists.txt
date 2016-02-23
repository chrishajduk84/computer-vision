set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include ../core/include ${DeckLinkSDK_INCLUDE_PATH})
ADD_DEFINITIONS("-DBOOST_LOG_DYN_LINK")

if (DeckLinkSDK_FOUND)
   include_directories(${DeckLinkSDK_INCLUDE_PATH})
endif (DeckLinkSDK_FOUND)

add_library(ImageImport ${BOOST_LIBRARIES} ${DeckLinkSDK_LIBS} ${DeckLinkSDK_SRC_FILE} src/imgimport.cpp src/DeckLinkCapture.cpp src/DeckLinkOpenCv.cpp src/DeckLinkInputCallback.cpp)
target_compile_features(ImageImport PRIVATE)
add_subdirectory("test")

