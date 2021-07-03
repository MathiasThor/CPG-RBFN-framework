cmake_minimum_required(VERSION 2.8.3)
project(morf_controller)

set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED YES)

SET (GOROBOTS "../../../..")
SET (CONTROLLER "${GOROBOTS}/neural_controllers/morf")
SET (UTILS "${GOROBOTS}/utils")
SET (RAPIDJSON "${CONTROLLER}/rapidjson")

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS std_msgs geometry_msgs roscpp sensor_msgs)
find_package(GSL REQUIRED)

include_directories("${catkin_INCLUDE_DIRS}"
        "${GOROBOTS}"
        "${UTILS}"
		"${UTILS}/vrepRemoteAPI"
		"${UTILS}/ESN"
		"${RAPIDJSON}"
		"${GSL_INCLUDE_DIRS}")

SET(LIBS ${LIBS} ${GSL_LIBRARIES})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_definitions(-DDO_NOT_USE_SHARED_MEMORY)
add_definitions(-DMAX_EXT_API_CONNECTIONS=255)
add_definitions(-DNON_MATLAB_PARSING)

add_executable(
        morf_controller
        ${CONTROLLER}/sim/main_hexapod_controller
        ${CONTROLLER}/sim/simRosClass
        ${CONTROLLER}/sim/modularController
        ${CONTROLLER}/sim/neutronMotorDefinition.h
        ${CONTROLLER}/sim/neutronController
        ${CONTROLLER}/sim/postProcessing
        ${CONTROLLER}/sim/cpg_rbfn
        ${CONTROLLER}/sim/rbfn
        ${UTILS}/ann-framework/ann
        ${UTILS}/ann-framework/neuron
        ${UTILS}/ann-framework/synapse
        ${UTILS}/ann-library/psn
        ${UTILS}/ann-library/vrn
        ${UTILS}/ann-library/pmn
        ${UTILS}/ann-library/so2cpg
        ${UTILS}/ann-library/adaptiveso2cpgsynplas
        ${UTILS}/ann-library/extendedso2cpg
        ${UTILS}/ann-library/pcpg
        ${UTILS}/delayline
        ${UTILS}/interpolator2d
	${UTILS}/vrepRemoteAPI/extApiPlatform
	${UTILS}/vrepRemoteAPI/extApiInternal
	${UTILS}/vrepRemoteAPI/extApi
	${UTILS}/vrepRemoteAPI/simConst
	${UTILS}/environment)

target_link_libraries(morf_controller ${catkin_LIBRARIES} ${LIBS})
add_dependencies(morf_controller ${catkin_EXPORTED_TARGETS})
