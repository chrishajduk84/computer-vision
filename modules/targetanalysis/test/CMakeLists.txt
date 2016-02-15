enable_testing()

if(Boost_FOUND AND OpenCV_FOUND)
    ADD_DEFINITIONS("-DBOOST_LOG_DYN_LINK")
    add_executable(target_loader_test test.cpp)
    target_link_libraries(target_loader_test ${OpenCV_LIBS} ${Boost_LIBRARIES} Core TargetAnalysis)

    #install(TARGETS target_loader_test DESTINATION bin)

    # Tests
    add_test(NAME "simple_load"
             COMMAND target_loader_test "${CMAKE_CURRENT_LIST_DIR}/json/sample.json" "Simple Load")
endif()
