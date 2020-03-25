if (NOT CUDA_TARGET_COMPUTE_CAPABILITY)
    message(STATUS
        "No CUDA compute capability was set. Executing script to find out...")
    if("$ENV{CUDA_SM}" STREQUAL "")
        set(ENV{CUDA_INCLUDE_DIRS} "${CUDA_INCLUDE_DIRS}")
        set(ENV{CUDA_CUDART_LIBRARY} "${CUDA_CUDART_LIBRARY}")
        set(ENV{CMAKE_CXX_COMPILER} "${CMAKE_CXX_COMPILER}")
        execute_process(COMMAND 
            bash -c "${CMAKE_CURRENT_SOURCE_DIR}/scripts/get_cuda_sm.sh" 
            OUTPUT_VARIABLE CUDA_TARGET_COMPUTE_CAPABILITY_)
    else()
        set(CUDA_TARGET_COMPUTE_CAPABILITY_ $ENV{CUDA_SM})
    endif()
    set(CUDA_TARGET_COMPUTE_CAPABILITY "${CUDA_TARGET_COMPUTE_CAPABILITY_}" 
        CACHE STRING "CUDA compute capability of the (first) CUDA device on \
        the system, in XY format (like the X.Y format but no dot); see table \
        of features and capabilities by capability X.Y value at \
        https://en.wikipedia.org/wiki/CUDA#Version_features_and_specifications")
    execute_process(COMMAND 
        bash -c "echo -n $(echo ${CUDA_TARGET_COMPUTE_CAPABILITY})" 
        OUTPUT_VARIABLE CUDA_TARGET_COMPUTE_CAPABILITY)
    execute_process(COMMAND 
        bash -c "echo ${CUDA_TARGET_COMPUTE_CAPABILITY} | sed 's/^\\([0-9]\\)\\([0-9]\\)/\\1.\\2/;' | xargs echo -n" 
        OUTPUT_VARIABLE FORMATTED_COMPUTE_CAPABILITY)
    message(STATUS 
        "CUDA device-side code will assume compute capability \
        ${FORMATTED_COMPUTE_CAPABILITY}")
endif()

set(CUDA_GENCODE "arch=compute_${CUDA_TARGET_COMPUTE_CAPABILITY},code=sm_${CUDA_TARGET_COMPUTE_CAPABILITY}")
