BIN=Main
OBJ_DIR = objs
MKDIR = mkdir -p

EIGEN_DIR = /usr/local/include/eigen3
BULLET_INC_DIR = /usr/local/include/bullet
ALEMBIC_INC_DIR = ../deps/alembic/lib
OPENEXR_INC_DIR = /usr/local/include/OpenEXR

PYTHON_INC = /usr/local/opt/python/Frameworks/Python.framework/Versions/3.7/include/python3.7m
PYTHON_LIB = /usr/local/opt/python/Frameworks/Python.framework/Versions/3.7/lib -lpython3.7m

INC = -I./ \
	-I$(EIGEN_DIR) \
	-I$(BULLET_INC_DIR) \
	-I$(ALEMBIC_INC_DIR) \
	-I$(ALEMBIC_INC_DIR)/../build/lib \
	-I$(OPENEXR_INC_DIR)


LIBS = -framework OpenGL -framework GLUT -lGLEW -lBulletDynamics -lBulletCollision -lLinearMath -lm -lstdc++ ../deps/alembic/build/lib/Alembic/libAlembic.a -lImath -lHalf -lIex -lIexMath -lIlmThread
CC = clang++
CFLAGS = -std=c++11 -O3 -Wall -fPIC 

SRC_DIRS = util/ \
	util/json/ \
	anim/ \
	sim/ \
	render/ \
	render/lodepng/ \
	scenes/ \
	scenes/arm/ \
	scenes/particle/ \
	scenes/pendulum/

SRC_FILES = DeepMimicCore.cpp $(foreach dir, $(SRC_DIRS), $(wildcard $(dir)*.cpp))
OUT_DIRS = $(addprefix $(OBJ_DIR)/, $(SRC_DIRS))
OBJ = $(addprefix $(OBJ_DIR)/, $(patsubst %.cpp, %.o, $(SRC_FILES)))


default: build

build: $(BIN)

print:
	$(OUT_DIRS)

python: $(OUT_DIRS) $(OBJ)
	swig -c++ -python DeepMimicCore.i
	$(CC) -c -g $(CFLAGS) $(INC) $(LIBS) DeepMimicCore.cpp DeepMimicCore_wrap.cxx -I$(PYTHON_INC)
	$(CC) -shared $(CFLAGS) $(OBJ) $(LIBS) DeepMimicCore_wrap.o -o _DeepMimicCore.so -L$(PYTHON_LIB)

$(OBJ_DIR)/%.o: %.cpp
	$(CC) -c -g $(CFLAGS) $(INC) -o $@ $<

$(BIN): $(OUT_DIRS) $(OBJ)
	$(CC) -g $(CFLAGS) $(OBJ) $(LIBS) -o $@

$(OUT_DIRS):
	$(MKDIR) $(OUT_DIRS)
	
clean:
	- rm -f -r $(OUT_DIRS) $(BIN) $(OBJ) $(OBJ_DIR) DeepMimicCore_wrap.cxx _DeepMimicCore.so
