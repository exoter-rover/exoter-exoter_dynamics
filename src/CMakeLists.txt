rock_library(dynamics
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(dynamics_bin Main.cpp
    DEPS dynamics)

