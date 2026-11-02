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

INCLUDES=-I../../lib/trace/include/ -I../../lib/hal/include -I../../lib/config/include -I../../lib/wavfile/include

DEFINES=-D__CORE__=0
TEST_DEFINES=-DTRACE_TEST

LIBS=-lm -lboost_system -lboost_filesystem

PROJ_OUT_DIR=out
PROJ_OBJ_DIR=obj

_OBJ = main.o trace.o
OBJ = $(patsubst %,$(PROJ_OUT_DIR)/$(PROJ_OBJ_DIR)/%,$(_OBJ))

_TEST_OBJ = test_trace_parse.o trace.o
TEST_OBJ = $(patsubst %,$(PROJ_OUT_DIR)/$(PROJ_OBJ_DIR)/%,$(_TEST_OBJ))

.DEFAULT_GOAL := $(PROJ_OUT_DIR)/trace_parse

$(PROJ_OUT_DIR)/$(PROJ_OBJ_DIR)/trace.o: ../../lib/trace/trace.cpp
	$(shell [ -d $(dir $@) ] || mkdir -p $(dir $@))
	$(CXX) -c $< -o $@ $(CXXFLAGS) $(INCLUDES) $(DEFINES) $(TEST_DEFINES)

$(PROJ_OUT_DIR)/$(PROJ_OBJ_DIR)/%.o: %.cpp
	$(shell [ -d $(dir $@) ] || mkdir -p $(dir $@))
	$(CXX) -c $< -o $@ $(CXXFLAGS) $(INCLUDES) $(DEFINES)

$(PROJ_OUT_DIR)/$(PROJ_OBJ_DIR)/%.o: %.c
	$(shell [ -d $(dir $@) ] || mkdir -p $(dir $@))
	$(CC) -c $< -o $@ $(CCFLAGS) $(INCLUDES) $(DEFINES)

$(PROJ_OUT_DIR)/trace_parse: $(OBJ)
	$(shell [ -d $(dir $@) ] || mkdir -p $(dir $@))
	$(CXX) -o $@ $^ $(CXXFLAGS) $(LIBS)

$(PROJ_OUT_DIR)/test_trace_parse: $(TEST_OBJ)
	$(shell [ -d $(dir $@) ] || mkdir -p $(dir $@))
	$(CXX) -o $@ $^ $(CXXFLAGS) $(LIBS)

test: test_clean $(PROJ_OUT_DIR)/test_trace_parse $(PROJ_OUT_DIR)/trace_parse
	./$(PROJ_OUT_DIR)/test_trace_parse ./temp.bin
	./$(PROJ_OUT_DIR)/trace_parse ./temp.bin

.PHONY: test_clean clean

test_clean:
	rm -rf ./temp.bin*

clean: test_clean
	rm -rf $(PROJ_OUT_DIR)/
