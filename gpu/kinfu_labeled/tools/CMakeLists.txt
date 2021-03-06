if(NOT WITH_OPENNI)
  return()
endif()

if(NOT VTK_FOUND)
  set(DEFAULT FALSE)
  set(REASON "VTK was not found.")
else()
  set(DEFAULT TRUE)
  set(REASON)
  set(VTK_USE_FILE "${VTK_USE_FILE}" CACHE INTERNAL "VTK_USE_FILE")
  include("${VTK_USE_FILE}")
  include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
endif()

file(GLOB hdrs "*.h*")
include_directories(SYSTEM ${VTK_INCLUDE_DIRS})

### STANDALONE TEXTURE MAPPING
#set(the_target pcl_kinfu_largeScale_texture_output pcl_kinfu_labeled_app)
#set(srcs standalone_texture_mapping.cpp)
#
#source_group("Source Files" FILES ${srcs})
#
#PCL_ADD_EXECUTABLE(${the_target} COMPONENT ${SUBSYS_NAME} SOURCES ${srcs} ${hdrs} BUNDLE)
#target_link_libraries("${the_target}" pcl_common pcl_io ${OPENNI_LIBRARIES} pcl_visualization pcl_gpu_kinfu_large_scale pcl_kdtree pcl_features pcl_surface)

## KINECT FUSION
set(the_target pcl_kinfu_labeled_app)
set(srcs kinfu_labeled_app.cpp capture.cpp evaluation.cpp)

source_group("Source Files" FILES ${srcs})

PCL_ADD_EXECUTABLE(${the_target} COMPONENT ${SUBSYS_NAME} SOURCES ${srcs} ${hdrs} BUNDLE)
target_link_libraries("${the_target}" pcl_common pcl_io ${OPENNI_LIBRARIES} pcl_visualization pcl_gpu_kinfu_labeled pcl_octree)

## STANDALONE MARCHING CUBES
#set(the_target pcl_kinfu_largeScale_mesh_output)
#set(srcs process_kinfu_large_scale_output.cpp)

#PCL_ADD_EXECUTABLE(${the_target} COMPONENT ${SUBSYS_NAME} SOURCES ${srcs} ${hdrs} BUNDLE)
#target_link_libraries("${the_target}" pcl_common pcl_io ${OPENNI_LIBRARIES} pcl_visualization pcl_gpu_kinfu_large_scale pcl_filters)

## RECORD MAPS_RGB
#set(the_target pcl_record_kinect_maps_rgb)
#set(srcs record_maps_rgb.cpp)

#PCL_ADD_EXECUTABLE(${the_target} COMPONENT ${SUBSYS_NAME} SOURCES ${srcs} ${hdrs} BUNDLE)
#target_link_libraries("${the_target}" pcl_common pcl_io ${OPENNI_LIBRARIES} pcl_visualization pcl_gpu_kinfu_large_scale pcl_filters)




#if(WITH_OPENNI)
#	if(NOT VTK_FOUND)
#		set(DEFAULT FALSE)
#		set(REASON "VTK was not found.")
#	else(NOT VTK_FOUND)
#		set(DEFAULT TRUE)
#		set(REASON)
#		set(VTK_USE_FILE "${VTK_USE_FILE}" CACHE INTERNAL "VTK_USE_FILE")
#		include("${VTK_USE_FILE}")
#		include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
#	endif(NOT VTK_FOUND)
#
#	FILE(GLOB hdrs "*.h*" include/pcl/gpu/kinfu_labeled/tools/*.h*)
#	include_directories(${VTK_INCLUDE_DIRS} ${OPENNI_INCLUDE_DIRS})
#
#	## KINECT FUSION
#	set(the_target pcl_kinfu_labeled_app)
#	set(srcs kinfu_labeled_app.cpp capture.cpp evaluation.cpp)
#
#	source_group("Source Files" FILES ${srcs} )
#
#	PCL_ADD_EXECUTABLE_OPT_BUNDLE("${the_target}" "${SUBSYS_NAME}" ${srcs} ${hdrs})
#	target_link_libraries("${the_target}" pcl_common pcl_io ${OPENNI_LIBRARIES} pcl_visualization pcl_gpu_kinfu_labeled)
#
#	if(OpenCV_FOUND)
#		target_link_libraries("${the_target}" ${OpenCV_LIBS})
#	endif()
#
#	### RECORD TSDFVOLUME
#	#set(the_target pcl_record_tsdfvolume)
#	#set(srcs record_tsdfvolume.cpp capture.cpp)
#
#	#PCL_ADD_EXECUTABLE("${the_target}" "${SUBSYS_NAME}" ${srcs} ${hdrs})
#	#target_link_libraries("${the_target}" pcl_common pcl_io ${OPENNI_LIBRARIES} pcl_visualization pcl_gpu_kinfu_labeled)
#
#	## KINECT FUSION SIMULATION - DISABLED FOR NOW
#	#find_package(GLEW)
#	#if(GLEW_FOUND)
#	#	find_package(GLUT)
#	#	if(GLUT_FOUND)
#	#		include_directories("${GLUT_INCLUDE_DIR}" "${GLEW_INCLUDE_DIR}")
#	#		## KINECT FUSION SIMULATION
#	#		set(the_target kinfu_labeled_app_sim)
#	#		set(srcs kinfu_labeled_app_sim.cpp capture.cpp evaluation.cpp)
#	#
#	#		source_group("Source Files" FILES ${srcs} )
#   	#			find_package( OpenCV REQUIRED )
#	#		PCL_ADD_EXECUTABLE("${the_target}" "${SUBSYS_NAME}" ${srcs} ${hdrs})
#	#		target_link_libraries("${the_target}" pcl_common pcl_io
#  	#			pcl_visualization pcl_kdtree  pcl_simulation  ${OpenCV_LIBS}
#	#	 		${OPENNI_LIBRARIES} pcl_gpu_kinfu_labeled
#	#	  		${GLEW_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES})
# 	#
#	#		if(OpenCV_FOUND)
#	#			target_link_libraries("${the_target}" ${OpenCV_LIBS})
#	#		endif()
#	#	endif(GLUT_FOUND)
#	#endif (GLEW_FOUND)
#
#endif()
