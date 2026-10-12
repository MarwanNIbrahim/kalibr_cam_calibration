add_subdirectory(kalibr)
add_subdirectory(CLI11 EXCLUDE_FROM_ALL)

# Collect every shared library built under kalibr/ so they can be installed
# side by side (into the python package or the system lib dir).
function(collect_shared_libs dir out)
  # Functions inherit the caller's variables, so reset before recursing
  set(libs "")
  get_property(targets DIRECTORY ${dir} PROPERTY BUILDSYSTEM_TARGETS)
  foreach(target IN LISTS targets)
    get_target_property(type ${target} TYPE)
    if(type STREQUAL "SHARED_LIBRARY")
      list(APPEND libs ${target})
    endif()
  endforeach()
  get_property(subdirs DIRECTORY ${dir} PROPERTY SUBDIRECTORIES)
  foreach(subdir IN LISTS subdirs)
    collect_shared_libs(${subdir} sub_libs)
    list(APPEND libs ${sub_libs})
  endforeach()
  set(${out} ${libs} PARENT_SCOPE)
endfunction()

collect_shared_libs(${CMAKE_CURRENT_SOURCE_DIR}/kalibr KALIBR_SHARED_LIBS)
list(REMOVE_DUPLICATES KALIBR_SHARED_LIBS)

# $ORIGIN lets the libraries find each other once installed in one directory.
set_target_properties(${KALIBR_SHARED_LIBS} PROPERTIES INSTALL_RPATH "$ORIGIN")
install(TARGETS ${KALIBR_SHARED_LIBS} LIBRARY DESTINATION ${KALIBR_LIB_DESTINATION})
