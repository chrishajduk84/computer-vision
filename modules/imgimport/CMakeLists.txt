include_directories(include ../core/include ${DeckLinkSDK_INCLUDE_PATH})
add_library(ImageImport ${DeckLinkSDK_LIBS}  src/imgimport.cpp)
