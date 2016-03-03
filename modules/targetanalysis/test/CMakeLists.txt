enable_testing()

if(Boost_FOUND AND OpenCV_FOUND)
    ADD_DEFINITIONS("-DBOOST_LOG_DYN_LINK")
    add_executable(target_loader_test test.cpp)
    target_link_libraries(target_loader_test ${OpenCV_LIBS} ${Boost_LIBRARIES} Core TargetAnalysis)

    # Tests
    add_test(NAME "SimpleLoad"
             COMMAND target_loader_test "${TESTDATA_DIR}/sample.json" "Simple Load")
endif()
