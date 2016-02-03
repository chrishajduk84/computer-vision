enable_testing()

if(Boost_FOUND AND OpenCV_FOUND)
    message(status "PRINTING THIS - DECKLINK ${DeckLinkSDK_INCLUDE_PATH}")
    include_directories(../include ../../core/include)
    ADD_DEFINITIONS("-DBOOST_LOG_DYN_LINK")
    add_executable(target_video_test main.cpp ${DeckLinkSDK_INCLUDE_PATH}/DeckLinkAPIDispatch.cpp)
    target_link_libraries(target_video_test ${OpenCV_LIBS} ${Boost_LIBRARIES} ${DeckLinkSDK_LIBS} ${Boost_FILESYSTEM_LIBRARY} Core ImageImport)
    install(TARGETS target_video_test DESTINATION bin)
    # Tests
    add_test("video_load" target_video_test "Simple Video Load")
endif()
