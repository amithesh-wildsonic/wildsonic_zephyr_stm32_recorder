CC=gcc
CXX=g++

COMMON_FLAGS=-g
COMMON_FLAGS+=-pedantic -Wall -Wextra -Wcast-align -Wcast-qual -Wdisabled-optimization
COMMON_FLAGS+=-Wformat=2 -Winit-self -Wlogical-op -Wmissing-declarations
COMMON_FLAGS+=-Wmissing-include-dirs -Wredundant-decls -Wshadow -Wsign-conversion
COMMON_FLAGS+=-Wstrict-overflow=5 -Wswitch-default -Wundef -Werror -Wno-unused

CCFLAGS=-std=c99
CCFLAGS+=$(COMMON_FLAGS)

CXXFLAGS=-std=c++11 -Wctor-dtor-privacy -Wnoexcept -Wold-style-cast
CXXFLAGS+=-Woverloaded-virtual -Wsign-promo -Wstrict-null-sentinel
CXXFLAGS+=$(COMMON_FLAGS)

INCLUDES=-I../include/

LIBS=-lm -lboost_system -lboost_filesystem

PROJ_OUT_DIR=out
PROJ_OBJ_DIR=obj

_OBJ = main.o bson_encode.o
OBJ = $(patsubst %,$(PROJ_OUT_DIR)/$(PROJ_OBJ_DIR)/%,$(_OBJ))

.DEFAULT_GOAL := $(PROJ_OUT_DIR)/bson_encode

$(PROJ_OUT_DIR)/$(PROJ_OBJ_DIR)/bson_encode.o: ../bson_encode.cpp
	$(shell [ -d $(dir $@) ] || mkdir -p $(dir $@))
	$(CXX) -c $< -o $@ $(CXXFLAGS) $(INCLUDES)

$(PROJ_OUT_DIR)/$(PROJ_OBJ_DIR)/%.o: %.cpp
	$(shell [ -d $(dir $@) ] || mkdir -p $(dir $@))
	$(CXX) -c $< -o $@ $(CXXFLAGS) $(INCLUDES)

$(PROJ_OUT_DIR)/$(PROJ_OBJ_DIR)/%.o: %.c
	$(shell [ -d $(dir $@) ] || mkdir -p $(dir $@))
	$(CC) -c $< -o $@ $(CCFLAGS) $(INCLUDES)

$(PROJ_OUT_DIR)/test_encode: $(OBJ) $(PROJ_OUT_DIR)/$(PROJ_OBJ_DIR)/bson_encode.o
	$(shell [ -d $(dir $@) ] || mkdir -p $(dir $@))
	$(CXX) -o $@ $^ $(CXXFLAGS) $(LIBS)

test: test_clean $(PROJ_OUT_DIR)/test_encode
	./$(PROJ_OUT_DIR)/test_encode ./temp.bson
	python ./bson_decode.py ./temp.bson

.PHONY: test_clean clean

test_clean:
	rm -rf ./temp.bson*

clean: test_clean
	rm -rf $(PROJ_OUT_DIR)/
