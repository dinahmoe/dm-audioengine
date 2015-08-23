CMAKE_MINIMUM_REQUIRED (VERSION 2.8)

PROJECT("AudioEngine")
MESSAGE(STATUS "Configuring AudioEngine library")

OPTION(USE_EXTERNAL_AUDIO "Exposes the calls to be driven by an external audio callback" OFF)

SET(AudioEngine_VERSION_MAJOR 1)
SET(AudioEngine_VERSION_MINOR 0)
SET(AudioEngine_VERSION_BUILD 2)

SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../dm-cmake)
INCLUDE(SetTargetOptions)
INCLUDE(SetPlatformDefines)
SET_PLATFORM_DEFINES()

SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../dm-cmake)
INCLUDE(SetTargetOptions)
INCLUDE(SetPlatformDefines)
SET_PLATFORM_DEFINES()

IF (USE_EXTERNAL_AUDIO) 
  SET(DMF_USES_EXTERNAL_AUDIO 1)
ELSE (USE_EXTERNAL_AUDIO)
  SET(DMF_USES_EXTERNAL_AUDIO 0)
ENDIF (USE_EXTERNAL_AUDIO)

IF (DMF_OFFLINE_RENDER)
  SET(DMF_USES_OFFLINE_RENDERING 1)
ELSE (DMF_OFFLINE_RENDER)
  SET(DMF_USES_OFFLINE_RENDERING 0)
ENDIF (DMF_OFFLINE_RENDER)


CONFIGURE_FILE (
  "${CMAKE_CURRENT_SOURCE_DIR}/include/AudioEngineConfig.hpp.in"
  "${CMAKE_CURRENT_BINARY_DIR}/AudioEngineConfig.hpp"
  )
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../dm-utils/include)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../dm-parameters/include)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../dm-log/include)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../dm-audiomidi/include)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../dm-dsp/include)


# note that I include also src because it contains all the internal header files
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/src)

# INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/dspBasics)

SET(AudioEngine_src
    "${CMAKE_CURRENT_SOURCE_DIR}/src/AudioFileReader.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/AudioNodeInput.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/InMemoryBuffer.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/AudioBufferSourceNode.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/AudioFileReader.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/AudioNodeInputType.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/BiquadFilterNode.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/DelayNode.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/AudioNodeOutput.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/AudioContext.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/AudioGainNode.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/AudioNodeOutput.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/AudioInputNode.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/AudioNode.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ControlTimeline.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/AudioNodeInput.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ControlTimeline.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ChannelSplitterNode.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ChannelMergerNode.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/DynamicsCompressorNode.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/OscillatorNode.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/WaveShaperNode.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/SummingNode.cpp")


SET(AudioEngine_include
    "${CMAKE_CURRENT_SOURCE_DIR}/include/AudioEngine.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/AudioContext.hpp")


ADD_LIBRARY(dm-audioengine ${AudioEngine_src} ${AudioEngine_include} ${AudioEngine_platform})
FSET_TARGET_OPTIONS(dm-audioengine "${DM_AUDIOENGINE_COMPILE_FLAGS}")

ADD_SUBDIRECTORY(test EXCLUDE_FROM_ALL)

