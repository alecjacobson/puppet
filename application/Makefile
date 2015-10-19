# Make file for GLUT skinning app

.PHONY: all
all: ctags puppet


gg=g++
#gg=g++-mp-4.7
#gg=g++-mp-4.3
# Boost with macports is built with gcc-apple and doesn't work with any proper g++
#gg=g++-apple-4.2

# -fno-objc-arc  seems to at least hide if not fix some memory bug
CFLAGS+=-Wno-deprecated-declarations -fno-objc-arc
RELEASE_CFLAGS=-std=c++11 -g -O3 -Wall -Wextra -Wno-reorder -Wno-unknown-pragmas -D_RELEASE -msse4.2 -DIGL_STATIC_LIBRARY
DEBUG_CFLAGS=-std=c++11 -g -ggdb -O0 -Wall -Wextra -Wno-reorder -Wno-unknown-pragmas -D_DEBUG -msse4.2 -fstack-protector-all -DIGL_STATIC_LIBRARY
CFLAGS+=${RELEASE_CFLAGS}
build=release
#CFLAGS+=${DEBUG_CFLAGS}
#build=debug

# LIBIGL Library
LIBIGL=/usr/local/igl/libigl/
LIBIGL_LIB=-L$(LIBIGL)/lib -liglpng -ligltetgen -liglmosek -ligl -liglembree -liglbbw -liglviewer -liglanttweakbar -liglopengl -liglopengl2
LIBIGL_INC=-I$(LIBIGL)/include

EMBREE=$(LIBIGL)/external/embree
EMBREE_INC=-I$(EMBREE)/ -I$(EMBREE)/include
EMBREE_LIB=-L$(EMBREE)/build -lembree -lsys

MATLAB_CFLAGS+=-DWITH_MATLAB
MATLAB_INC=-I$(MATLAB)/extern/include/
MATLAB_LIB=-L$(MATLAB)/bin/maci64 -lmx -lmat -lmex -lstdc++

ANTTWEAKBAR=/usr/local/igl/libigl/external/AntTweakBar
ANTTWEAKBAR_LIB=-L$(ANTTWEAKBAR)/lib -lAntTweakBar
ANTTWEAKBAR_INC=-I$(ANTTWEAKBAR)/include -I$(ANTTWEAKBAR)/src

#OPENAL=/usr/local/
#OPENAL_INC=-I$(OPENAL)/include
#OPENAL_LIB=-L$(OPENAL)/lib -lopenal
OPENAL_INC=-framework OpenAL
OPENAL_LIB=-framework OpenAL

ALUT=/usr/local/
ALUT_INC=-I$(ALUT)/include
ALUT_LIB=-L$(ALUT)/lib -lalut

# Tetgen Library
TETGEN=$(LIBIGL)/external/tetgen/
TETGEN_LIB=-L$(TETGEN) -ltet
TETGEN_INC=-I$(TETGEN)

# YIMAGE Library
YIMG=$(LIBIGL)/external/yimg/
YIMG_LIB=-L$(YIMG) -lyimg -lz -L/usr/X11/lib -lpng -bind_at_load
YIMG_INC=-I/usr/X11/include -I$(YIMG) 

# TinyXML2 Library
TINYXML2=$(LIBIGL)/external/tinyxml2/
TINYXML2_LIB=-L$(TINYXML2) -ltinyxml2
TINYXML2_INC=-I$(TINYXML2) 

# GLFW
GLFW=$(LIBIGL)/external/glfw
GLFW_INC=-I$(GLFW)/include
GLFW_LIB=-L$(GLFW)/lib/ -lglfw3 \
				 -framework AppKit -framework CoreVideo -framework IOKit -framework AGL

UNAME := $(shell uname)
ifeq ($(UNAME), Linux)
# OpenGL and GLUT
	OPENGL_LIB=-lGL -lGLU
	GLUT_LIB=-lglut
# Eigen
	EIGEN3_INC=-I/usr/local/include/eigen3 -I/usr/local/include/eigen3/unsupported
# BOOST
	BOOST_INC= -I/usr/local/include
	BOOST_LIB= -L/usr/local/lib -lboost_system-mt -lboost_program_options-mt
# DRIVER API
	DRIVER=../driver/
	FIRMWARE_PWD=../firmware/..
	DRIVER_LIB=-L$(DRIVER)/generic/lib -lpuppet_api -L$(DRIVER)/unix/lib -lpuppet_serial
	DRIVER_INC=-I$(DRIVER)/unix/include -I$(DRIVER)/generic/include -I$(DRIVER) -I$(DRIVER)/ftdi/include -I${FIRMWARE_PWD}
# Mosek
	MOSEKPLATFORM=linux64x86
	MOSEK=/usr/local/mosek
	MOSEK_INC=-I$(MOSEK)/7/tools/platform/$(MOSEKPLATFORM)/h
	MOSEK_LIB=-L$(MOSEK)/7/tools/platform/$(MOSEKPLATFORM)/bin -lmosek64 -liomp5 -lpthread
else
# OpenGL and GLUT
	OPENGL_LIB=-framework OpenGL
	#GLUT_LIB=-L/opt/local/lib -lglut -framework Cocoa
	GLUT_LIB=-framework GLUT
	ANTTWEAKBAR_LIB+= -framework AppKit
# Eigen
	EIGEN3_INC=-I/usr/local/include/eigen3 -I/usr/local/include/eigen3/unsupported
# BOOST
	BOOST_INC= -I/usr/local/include
	BOOST_LIB= -L/usr/local/lib -lboost_system-mt -lboost_program_options -lboost_program_options-mt
# Driver API
	DRIVER=../driver/
	FIRMWARE_PWD=../firmware/../
	DRIVER_LIB=-L$(DRIVER)/generic/lib -lpuppet_api -L$(DRIVER)/ftdi/lib -lpuppet_d2xx_serial
	DRIVER_INC=-I$(DRIVER)/unix/include -I$(DRIVER)/generic/include -I$(DRIVER) -I$(DRIVER)/ftdi/include -I${FIRMWARE_PWD}
# FTDI D2XX
	FTDI=/usr/local/
	FTDI_LIB=-L$(FTDI)/lib -lftd2xx -lftdi -framework IOKit
	FTDI_INC=-I$(FTDI)/include
# Mosek
	MOSEKPLATFORM=osx64x86
	MOSEK=/usr/local/mosek
	MOSEK_INC=-I$(MOSEK)/7/tools/platform/$(MOSEKPLATFORM)/h
	MOSEK_LIB=-L$(MOSEK)/7/tools/platform/$(MOSEKPLATFORM)/bin -lmosek64 
	#AFLAGS += -arch x86_64 -m64 -march=corei7-avx
	CARBON_LIB=-framework Carbon
endif



# Gather libraries and includes 
INC=-I. ${FTDI_INC} ${EIGEN3_INC} ${LIBIGL_INC} ${YIMG_INC} ${ANTTWEAKBAR_INC} ${BOOST_INC} ${DRIVER_INC} ${MOSEK_INC} ${TETGEN_INC} ${TINYXML2_INC} ${EMBREE_INC} ${ALUT_INC} ${OPENAL_INC} ${MATLAB_INC}  
# For LINUX it's important that these are in reverse dependent order
#LIB=${OPENGL_LIB} ${GLUT_LIB} ${LIBIGL_LIB} ${YIMG_LIB} ${ANTTWEAKBAR_LIB} ${BOOST_LIB} ${DRIVER_LIB} ${MOSEK_LIB} ${TETGEN_LIB} ${TINYXML2_LIB}
LIB=${FTDI_LIB} ${DRIVER_LIB} ${BOOST_LIB} ${YIMG_LIB} ${LIBIGL_LIB} ${MOSEK_LIB} ${TETGEN_LIB} ${TINYXML2_LIB} ${ANTTWEAKBAR_LIB} ${GLUT_LIB} ${OPENGL_LIB} ${CARBON_LIB} ${EMBREE_LIB} ${ALUT_LIB} ${OPENAL_LIB} ${MATLAB_LIB} 
CFLAGS+=${MATLAB_CFLAGS}

CPP_FILES=$(wildcard ./*.cpp)
OBJ_FILES=$(addprefix $(build)/obj/,$(notdir $(CPP_FILES:.cpp=.o))) 

.PHONY: ctags
ctags:
	ctags *.cpp *.h

# This requires Alec's modifed dylibbundler

puppet.app: puppet data/logo/puppet.icns
	rm -rf $@
	build_app $^
	cp -r data/parts/ ./$@/Contents/Resources/parts/
	cp -r data/cinekid/ ./$@/Contents/Resources/cinekid/
	cp -r examples/cinekid/dino/ ./$@/Contents/Resources/cinekid/dino/
	cp -r examples/cinekid/ikea-lamp/ ./$@/Contents/Resources/cinekid/ikea-lamp/
	dylibbundler -of -od -b -x ./$@/Contents/MacOS/$< -d ./$@/Contents/libs/
	install_name_tool -change @rpath/libboost_program_options.dylib  @executable_path/../libs/libboost_program_options.dylib  ./$@/Contents/MacOS/$<
	install_name_tool -change @rpath/libmx.dylib  @executable_path/../libs/libmx.dylib  ./$@/Contents/MacOS/$<
	install_name_tool -change @rpath/libmex.dylib  @executable_path/../libs/libmex.dylib  ./$@/Contents/MacOS/$<
	install_name_tool -change @rpath/libmat.dylib  @executable_path/../libs/libmat.dylib  ./$@/Contents/MacOS/$<
	install_name_tool -change @loader_path/libmosek64.7.0.dylib  @executable_path/../libs/libmosek64.7.0.dylib  ./$@/Contents/MacOS/$<

wackeldackel.app: wackeldackel data/logo/wackeldackel.icns
	build_app $^
	cp -r data/wackeldackel/ ./$@/Contents/Resources/wackeldackel/
	cp -r data/parts/ ./$@/Contents/Resources/parts/
	dylibbundler -od -b -x ./$@/Contents/MacOS/$< -d ./$@/Contents/libs/
	install_name_tool -change @rpath/libboost_program_options.dylib  @executable_path/../libs/libboost_program_options.dylib  ./$@/Contents/MacOS/$<
	install_name_tool -change @rpath/libmx.dylib  @executable_path/../libs/libmx.dylib  ./$@/Contents/MacOS/$<
	install_name_tool -change @rpath/libmex.dylib  @executable_path/../libs/libmex.dylib  ./$@/Contents/MacOS/$<
	install_name_tool -change @rpath/libmat.dylib  @executable_path/../libs/libmat.dylib  ./$@/Contents/MacOS/$<
	install_name_tool -change @loader_path/libmosek64.7.0.dylib  @executable_path/../libs/libmosek64.7.0.dylib  ./$@/Contents/MacOS/$<


puppet: ${build}/obj $(OBJ_FILES) ${build}/obj/main.o
	$(gg) -o puppet $(AFLAGS) $(CFLAGS) $(OBJ_FILES) ${build}/obj/main.o $(LIB)

.PHONY: libpuppet
libpuppet: lib/libpuppet.a 

lib/libpuppet.a: ${OBJ_FILES}
	mkdir -p lib
	rm -f $@
	ar cqs $@ $(OBJ_FILES)

${build}/obj:
	mkdir -p ${build}/obj

debug/obj/Uniped.o: Uniped.cpp Uniped.h
	$(gg) $(AFLAGS) $(RELEASE_CFLAGS) -o $@ -c $< $(INC)

debug/obj/dqs.o: dqs.cpp dqs.h
	$(gg) $(AFLAGS) $(RELEASE_CFLAGS) -o $@ -c $< $(INC)

${build}/obj/%.o: %.cpp %.h
#echo ${OBJ_FILES}
#echo $@
	$(gg) $(AFLAGS) $(CFLAGS) -o $@ -c $< $(INC)

# Special rule for main
${build}/obj/main.o: main.cxx
	$(gg) $(AFLAGS) $(CFLAGS) -o $@ -c $< $(INC)

${build}/obj/tree_fit_test.o: tree_fit_test.cxx
	$(gg) $(AFLAGS) $(CFLAGS) -o $@ -c $< $(INC)

tree_fit_test: ${build}/obj/tree_fit.o ${build}/obj/tree_fit_test.o ${build}/obj/fit_rotation.o
	$(gg) -o $@ $(AFLAGS) $(CFLAGS) $^ $(LIB)

${build}/obj/%.o: %.cxx
	$(gg) $(AFLAGS) $(CFLAGS) -o $@ -c $< $(INC)

transformation_test: ${build}/obj/transformation_test.o $(OBJ_FILES)
	$(gg) -o $@ $(AFLAGS) $(CFLAGS) $^ $(LIB)

bezier_test: ${build}/obj/bezier_test.o $(OBJ_FILES)
	$(gg) -o $@ $(AFLAGS) $(CFLAGS) $^ $(LIB)

bind_test: ${build}/obj/bind_test.o $(OBJ_FILES)
	$(gg) -o $@ $(AFLAGS) $(CFLAGS) $^ $(LIB)

pupskin_test: ${build}/obj/pupskin_test.o $(OBJ_FILES)
	$(gg) -o $@ $(AFLAGS) $(CFLAGS) $^ $(LIB)

wackeldackel: ${build}/obj/wackeldackel.o $(OBJ_FILES)
	$(gg) -o $@ $(AFLAGS) $(CFLAGS) $^ $(LIB) $(GLFW_LIB)

leg: ${build}/obj/leg.o $(OBJ_FILES)
	$(gg) -o $@ $(AFLAGS) $(CFLAGS) $^ $(LIB)

tank: ${build}/obj/tank.o $(OBJ_FILES)
	$(gg) -o $@ $(AFLAGS) $(CFLAGS) $^ $(LIB)


.PHONY: valgrind_pupskin
valgrind_pupskin: ${build}/obj/PupSkin.o ${build}/obj/pupskin_test.o
#$(gg) -o pupskin_test $(AFLAGS) $(CFLAGS) ${build}/obj/pupskin_test.o  $(LIB)
	g++-mp-4.7 -o pupskin_test  -std=c++11 -g -ggdb -O0 -Wall -Wextra -Wno-reorder -Wno-unknown-pragmas -D_DEBUG -msse4.2 debug/obj/pupskin_test.o  -L/usr/local//lib -lftd2xx -lftdi -L../../../the-puppet/driver/generic/lib -lpuppet_api -L../../../the-puppet/driver/ftdi/lib -lpuppet_d2xx_serial -L/usr/local/lib -L/usr/local/igl/libigl//external/yimg/ -lyimg -lz -L/usr/X11/lib -lpng -bind_at_load -L/usr/local/igl/libigl//lib -liglpng -ligltetgen -liglmosek -ligl -liglembree -liglbbw -L/usr/local/mosek/7/tools/platform/osx64x86/bin -lmosek64  -L/usr/local/igl/libigl//external/tetgen/ -ltet -L/usr/local/igl/libigl//external/tinyxml2/ -ltinyxml2 -L/usr/local/igl/libigl/external/AntTweakBar/lib -lAntTweakBar -framework AppKit -framework GLUT -framework OpenGL -framework Carbon -L/usr/local/igl/libigl//external/embree/bin -lembree -lsys -L/usr/local//lib -lalut -framework OpenAL -lstdc++ 
	valgrind --leak-check=yes ./pupskin_test

.PHONY: clean
clean:
	rm -f $(OBJ_FILES)
	rm -f puppet
	rm -rf puppet.app
	rm -rf wackeldackel.app
