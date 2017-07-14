add_subdirectory(rviz)
#add_subdirectory(image_view)  # TODO: add back image_view
if (BUILD_TESTING)
  add_subdirectory(test)
endif()

# Python bindings are only support with Qt5, PyQt5, and PySide2 for Kinetic+.
if (UseQt5)
  #add_subdirectory(python_bindings)
  # TODO: get python bindings working
endif()
