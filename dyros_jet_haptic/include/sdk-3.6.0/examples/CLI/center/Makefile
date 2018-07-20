#  (C) 2001-2015 Force Dimension
#  All Rights Reserved.
#
#  Version 3.6.0


# dependency
LDLIBS += -ldrd

# project layout
TOP_DIR = ../../..
include $(TOP_DIR)/Makefile.common

# local configuration
SRC_DIR   = .
OBJ_DIR   = ./obj/$(CFG)/$(OS)-$(ARCH)-$(COMPILER)

# add Eigen dependency
CXXFLAGS += -I$(TOP_DIR)/external/Eigen

# definitions
SOURCES   = $(wildcard $(SRC_DIR)/*.cpp)
INCLUDES  = $(wildcard $(SRC_DIR)/*.h)
OBJECTS   = $(patsubst %.cpp, $(OBJ_DIR)/%.o, $(notdir $(SOURCES)))
NAME      = $(shell pwd | awk 'BEGIN { FS="/" } { print $$NF }') 
TARGET    = $(BIN_DIR)/$(NAME)

# rules
all: $(TARGET)

$(OBJECTS): $(SOURCES) $(INCLUDES)

$(TARGET): $(OBJ_DIR) $(DRD_LIB_STATIC) $(OBJECTS)
	$(CXX) $(CXXFLAGS) $(OBJECTS) $(LDFLAGS) $(LDLIBS) -o $(TARGET)

$(OBJ_DIR):
	mkdir -p $@

$(OBJ_DIR)/%.o : $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f $(TARGET) $(OBJECTS) *~
	-rm -rf $(OBJ_DIR)
	
