set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include_directories(include ../core/include ${DeckLinkSDK_INCLUDE_PATH})
ADD_DEFINITIONS("-DBOOST_LOG_DYN_LINK")

if (DeckLinkSDK_FOUND)
    ADD_DEFINITIONS("-DHAS_DECKLINK")
    include_directories(${DeckLinkSDK_INCLUDE_PATH})
    set(DeckLinkFiles ${DeckLinkSDK_LIBS} ${DeckLinkSDK_SRC_FILE} src/DeckLinkCapture.cpp src/DeckLinkOpenCv.cpp src/DeckLinkInputCallback.cpp)
endif (DeckLinkSDK_FOUND)

add_library(ImageImport ${BOOST_LIBRARIES} src/imgimport.cpp src/vidimport.cpp ${DeckLinkFiles})

target_compile_features(ImageImport PRIVATE)
add_subdirectory("test")

