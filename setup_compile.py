from setuptools import find_packages
from distutils.core import setup, Extension
import sys, os

dep = """
C++ dependencies for this project are:

bullet
tinyxml
boost_python
assimp
Qt5
caffee
jsoncpp


If you see compilation error FIRST THING TO CHECK if pkg-config call was successful.
Install dependencies that pkg-config cannot find.
"""

from setuptools.command.install import install as DistutilsInstall
from setuptools.command.egg_info import egg_info as EggInfo

setup_py_dir = os.path.dirname(os.path.realpath(__file__))

print("library build location: ", setup_py_dir)
# blib = setup_py_dir + '/terrainRLAdapter'
blib = setup_py_dir
if not os.path.exists(blib):
    print("Please follow instructions in README to build local (not global for your system) Bullet installation.")
    sys.exit(1)

def recompile():
    USE_PYTHON3 = ""
    if sys.version_info[0]==2:
        USE_PYTHON3 = "USE_PYTHON3=0"
    cmd = """
        #!/bin/bash
        cd simAdapter
        ./gen_swig.sh
        cd ../
        premake4 gmake &&
        cd gmake &&
        make config=release64 -j 4 &&
        cd ../
        """#  % (setup_py_dir, USE_PYTHON3)
    print(cmd)
    """
    res = os.system(cmd)
    if res:
        print(dep)
        sys.exit(1)
    """

class MyInstall(DistutilsInstall):
    def run(self):
        recompile()
        DistutilsInstall.run(self)

class MyEgg(EggInfo):
    def run(self):
        recompile()
        EggInfo.run(self)

need_files_args = []
file_types = 'txt png jpg urdf obj mtl dae off stl STL xml glsl so 87 dylib'.split()
hh = setup_py_dir + "/args"
print ("hh: ", hh)
print ("os.walk(hh): ", list(os.walk(hh)))

for root, dirs, files in os.walk(hh):
    for fn in files:
        ext = os.path.splitext(fn)[1][1:]
        if ext and ext in file_types:
            fn = root + "/" + fn
            need_files_args.append("./args/" + fn[1+len(hh):])

need_files_data = []            
hh = setup_py_dir + "/data"

for root, dirs, files in os.walk(hh):
    for fn in files:
        ext = os.path.splitext(fn)[1][1:]
        if ext and ext in file_types:
            fn = root + "/" + fn
            print (fn)
            need_files_data.append("./data/" + fn[1+len(hh):])

print("found resource files: %i" % len(need_files_args))
print( "need_files_args: ", need_files_args)
#for n in need_files: print("-- %s" % n)
print("found data resource files: %i" % len(need_files_data))
print( "need_files_data: ", need_files_data)
#for n in need_files: print("-- %s" % n)


extension_mod = Extension("_terrainRLAdapter", 
                          # ['terrainRLAdapter.swig', "SimAdapter.cpp"],
                          # swig_opts=['-modern'],
                          ["terrainRLAdapter.cpp", "SimAdapter.cpp"],
                          extra_compile_args = ['-std=c++0x', '-ggdb', '-fPIC', 
                                                "-Wl,-rpath," + setup_py_dir +"/lib"],
                          include_dirs = ['./', './external/Bullet/src', './external', 
                                          './external/caffe/include', './external/caffe/build/src',
                                          './external/3rdparty/include/hdf5',
                                          './external/3rdparty/include/',
                                          './external/3rdparty/include/openblas',
                                          './external/3rdparty/include/lmdb',
                                          './external/OpenCV/include',
                                          './external/caffe/src/',
                                          "/usr/local/cuda/include/",
                                          "./",
                                            "anim",
                                            "learning",
                                            "sim",
                                            "render",
                                            "scenarios",
                                            "util",
                                            "../"
                                          ])

setup(
      #ext_modules=[Extension('terrainRLAdapter',
      #          sources=[],
      #          libraries=['_terrainRLAdapter'],
      #          runtime_library_dirs=['./lib/']
              # libraries=['X11', 'Xt']
      #        )],
    name = 'terrainRLAdapter',
    version = '0.2',
    description = 'Character simulation in Bullet Physics Engine',
    maintainer = 'Glen Berseth',
    maintainer_email = 'glen@fracturedplane.com',
    url = 'https://github.com/UBCMOCCA/TerrainRL',
    # packages=['simAdapter.terrainRLAdapter'],
    # package_dir={'terrainRLAdapter': 'terrainRLAdapter'},
    # package_data = { 'terrainRLAdapter': ['./lib/*.so'] },
    py_modules=['simAdapter.terrainRLAdapter'],
    ext_modules=[extension_mod],
    data_files = [ ('args', need_files_args),
                  ('data' , need_files_data) ],
    cmdclass = {
        'install': MyInstall,
        'egg_info': MyEgg
        },
    # package_data = { 'args': need_files_args }
    )

