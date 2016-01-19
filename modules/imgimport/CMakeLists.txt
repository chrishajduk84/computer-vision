include_directories(include ../core/include)
target_link_libraries(ImageImport ${DeckLinkSDK_LIBS} Core)
add_library(ImageImport src/imgimport.cpp ${DeckLinkSDK_SRC_FILE})
