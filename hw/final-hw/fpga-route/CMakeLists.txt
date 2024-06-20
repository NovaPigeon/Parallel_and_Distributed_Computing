cmake_minimum_required(VERSION 3.0)
project(aceroute VERSION 0.1.0)

string(REPLACE "-DNDEBUG" "" CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")

find_package(ZLIB REQUIRED)
find_package(CapnProto REQUIRED)

find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED)

file(GLOB AceRoute_SOURCES
  "src/ace-route.cc"
  "src/ace-route-reader.cc"
  "src/ace-route-writer.cc"
  "src/config.cc"
  "src/connection.cc"
  "src/device.cc"
  "src/intentcode.cc"
  "src/main.cc"
  "src/net.cc"
  "src/node.cc"
  "src/partition.cc"
  "src/route-fixer.cc"
  "src/routing-graph.cc"
  "src/utils.cc"
  "src/partition.cc"
)

file(GLOB FPGAIF_CAPNP_FILES
  "extern/interchange/*.capnp"
)
capnp_generate_cpp(FPGAIF_CPP_FILES FPGAIF_H_FILES ${FPGAIF_CAPNP_FILES})

add_compile_options(-Wall ${OpenMP_CXX_FLAGS})

set(EXE ${PROJECT_NAME}.exe)
add_executable(${EXE} ${AceRoute_SOURCES} ${FPGAIF_H_FILES})
target_compile_features(${EXE} PRIVATE cxx_std_20)
target_include_directories(${EXE} PRIVATE 
                           ${CMAKE_SOURCE_DIR}/extern/taskflow 
                           ${CMAKE_BINARY_DIR}/extern/interchange)
target_link_libraries(${EXE} PRIVATE ZLIB::ZLIB CapnProto::capnp -llog4cplus OpenMP::OpenMP_CXX)
