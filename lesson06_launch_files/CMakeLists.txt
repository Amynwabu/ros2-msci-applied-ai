# ... other install commands ...

# INSTALL THE LAUNCH DIRECTORY
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
