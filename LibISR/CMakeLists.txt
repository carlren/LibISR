add_subdirectory(UI)
add_subdirectory(Objects)
add_subdirectory(Engines)

set(LibISR_Common
	LibISRDefine.h
	${LibISR_Objects}
	${LibISR_Engines_Base}
	${LibISR_Engines_shared}
	${LibISR_Utils}
	)

cuda_add_library(LibISR_GPU
	${LibISR_Common}
	${LibISR_Engines_GPU}
	)
target_link_libraries(LibISR_GPU UI Objects Engines ${CUDA_LIBRARY})

add_library(LibISR_CPU
	${LibISR_Common}
	${LibISR_Engines_CPU}
	)
target_link_libraries(LibISR_CPU UI Objects Engines)