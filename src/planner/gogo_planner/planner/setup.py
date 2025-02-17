import os
import re
import subprocess
import sys
from pathlib import Path

from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext

# 将 Windows 平台名转换为 CMake 所需的格式
PLAT_TO_CMAKE = {
    "win32": "Win32",
    "win-amd64": "x64",
    "win-arm32": "ARM",
    "win-arm64": "ARM64",
}

# 定义一个 CMakeExtension 类，不需要指定源码列表
class CMakeExtension(Extension):
    def __init__(self, name: str, sourcedir: str = "") -> None:
        super().__init__(name, sources=[])
        # 将 sourcedir 转换为绝对路径
        self.sourcedir = os.fspath(Path(sourcedir).resolve())

# 自定义 CMake 构建类，继承自 build_ext
class CMakeBuild(build_ext):
    def build_extension(self, ext: CMakeExtension) -> None:
        # 计算扩展模块最终生成的完整路径，并取得所在目录
        ext_fullpath = Path.cwd() / self.get_ext_fullpath(ext.name)
        extdir = ext_fullpath.parent.resolve()

        # 根据 DEBUG 环境变量或 self.debug 决定编译配置
        debug = int(os.environ.get("DEBUG", 0)) if self.debug is None else self.debug
        cfg = "Debug" if debug else "Release"

        # 取得环境变量中可能设置的 CMake 生成器
        cmake_generator = os.environ.get("CMAKE_GENERATOR", "")

        # 构造传递给 CMake 的参数，其中包括输出目录、Python解释器和构建类型
        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}{os.sep}",
            f"-DPYTHON_EXECUTABLE={sys.executable}",
            f"-DCMAKE_BUILD_TYPE={cfg}",
        ]
        build_args = []

        # 如果环境变量中定义了 CMAKE_ARGS，则添加进去
        if "CMAKE_ARGS" in os.environ:
            cmake_args += [item for item in os.environ["CMAKE_ARGS"].split(" ") if item]

        # 将版本号传入 C++ 代码（如果 CMakeLists.txt 里有使用 EXAMPLE_VERSION_INFO）
        cmake_args += [f"-DEXAMPLE_VERSION_INFO={self.distribution.get_version()}"]

        # 对于非 MSVC 编译器，如果未指定生成器或使用 Ninja，则尝试使用 Ninja
        if self.compiler.compiler_type != "msvc":
            if not cmake_generator or cmake_generator == "Ninja":
                try:
                    import ninja
                    ninja_executable_path = Path(ninja.BIN_DIR) / "ninja"
                    cmake_args += [
                        "-GNinja",
                        f"-DCMAKE_MAKE_PROGRAM:FILEPATH={ninja_executable_path}",
                    ]
                except ImportError:
                    pass
        else:
            # MSVC 下的特殊处理（可参考官方示例，此处不做过多修改）
            single_config = any(x in cmake_generator for x in {"NMake", "Ninja"})
            contains_arch = any(x in cmake_generator for x in {"ARM", "Win64"})
            if not single_config and not contains_arch:
                cmake_args += ["-A", PLAT_TO_CMAKE[self.plat_name]]
            if not single_config:
                cmake_args += [f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_{cfg.upper()}={extdir}"]
                build_args += ["--config", cfg]

        # macOS 下处理 ARCHFLAGS
        if sys.platform.startswith("darwin"):
            archs = re.findall(r"-arch (\S+)", os.environ.get("ARCHFLAGS", ""))
            if archs:
                cmake_args += ["-DCMAKE_OSX_ARCHITECTURES={}".format(";".join(archs))]

        # 设置并行构建参数（如果没有在环境变量中指定）
        if "CMAKE_BUILD_PARALLEL_LEVEL" not in os.environ:
            if hasattr(self, "parallel") and self.parallel:
                build_args += [f"-j{self.parallel}"]

        # 使用 setuptools 指定的临时构建目录（build_temp），并在此目录下创建一个子目录（以扩展名命名）
        build_temp = Path(self.build_temp).resolve() / ext.name
        if not build_temp.exists():
            build_temp.mkdir(parents=True, exist_ok=True)

        print("Configuring CMake in:", build_temp)
        # 第一步：在 build_temp 目录下执行 cmake 配置命令（生成 CMakeCache.txt 等文件）
        subprocess.run(
            ["cmake", ext.sourcedir, *cmake_args],
            cwd=str(build_temp),
            check=True,
        )
        print("Building extensions with CMake in:", build_temp)
        # 第二步：在同一目录下执行 cmake --build 命令
        subprocess.run(
            ["cmake", "--build", ".", *build_args],
            cwd=str(build_temp),
            check=True,
        )

# 调用 setup() 配置包信息，此处 ext_modules 指定了一个 CMakeExtension（名称与 CMakeLists.txt 中生成的模块名称一致）
setup(
    name="planner",
    version="0.0.1",
    author="Dean Moldovan",
    author_email="dean0x7d@gmail.com",
    description="A test project using pybind11 and CMake",
    long_description="",
    ext_modules=[CMakeExtension("planner")],
    cmdclass={"build_ext": CMakeBuild},
    zip_safe=False,
    extras_require={"test": ["pytest>=6.0"]},
    python_requires=">=3.7",
)
