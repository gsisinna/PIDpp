from conan import ConanFile
from conan.tools.cmake import CMake, CMakeToolchain, cmake_layout


class PidppConan(ConanFile):
    name = "pidpp"
    version = "0.1.0"
    license = "MIT"
    author = "PIDPP contributors"
    url = "https://github.com/pidpp/pidpp"
    description = "PIDPP Control Engine"
    topics = ("pid", "robotics", "control", "embedded")
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeDeps"
    exports_sources = "CMakeLists.txt", "cmake/*", "include/*", "src/*", "LICENSE"
    package_type = "library"

    def layout(self):
        cmake_layout(self)

    def generate(self):
        tc = CMakeToolchain(self)
        tc.variables["PIDPP_BUILD_TESTS"] = False
        tc.variables["PIDPP_BUILD_EXAMPLES"] = False
        tc.variables["PIDPP_BUILD_BENCHMARKS"] = False
        tc.variables["PIDPP_INSTALL"] = True
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.set_property("cmake_file_name", "PIDPP")
        self.cpp_info.set_property("cmake_target_name", "pidpp::pidpp")
        self.cpp_info.libs = ["pidpp"]
