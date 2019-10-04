# C compiler/linker and compiler flags #
CC := gcc
AR := ar
CFLAGS := -std=c11 -Wall -Wconversion -Wextra -Wfatal-errors -Wpedantic -fPIC -g
ARFLAGS := -crs
LDFLAGS := -shared

# Final binary/executable/library #
BIN := ssd1803a

# Source, build, and lib directories #
SRC_DIR := src
BUILD_DIR := build
LIB_DIR := lib

# Header file directories #
INC := $(shell find $(SRC_DIR) -type d)

# Source files #
SRC := $(wildcard $(INC:%=%/*.c))

# Object files #
OBJ := $(SRC:%.c=$(BUILD_DIR)/%.o)

# Dependency files #
DEP := $(OBJ:%.o=%.d)

# Libraries #
LIB := -pthread -lpigpio -lrt -lz

# Default target named after the binary #
$(BIN): $(LIB_DIR)/lib$(BIN).a $(LIB_DIR)/lib$(BIN).so

# Link the object files to create the static library #
$(LIB_DIR)/lib$(BIN).a: $(OBJ)
	@echo Creating $(@D)...
	@mkdir -p $(@D)
	@echo Linking $^ to create $@...
	$(AR) $(ARFLAGS) $@ $^
	@echo

# Link the object files to create the shared library #
$(LIB_DIR)/lib$(BIN).so: $(OBJ)
	@echo Creating $(@D)...
	@mkdir -p $(@D)
	@echo Linking $^ to create $@...
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@ $(LIB)
	@echo

# Include all of the dependencies #
-include $(DEP)

# Compile source files into object files #
# Any dependencies on header files is covered by calling -include $(DEPS) #
# $(INC:%=-I %) is used to specify directories containing header files #
# -MMD is used to generate a dependency output file during compilation #
$(BUILD_DIR)/%.o: %.c
	@echo Creating $(@D)...
	@mkdir -p $(@D)
	@echo Compiling $<...
	$(CC) $(CFLAGS) $(INC:%=-I %) -MMD -c $< -o $@
	@echo

.PHONY: all clean distclean liba libso install uninstall

# Default build target #
all: $(LIB_DIR)/lib$(BIN).a $(LIB_DIR)/lib$(BIN).so

# Remove the object and dependency files #
clean:
	@echo Deleting $(BUILD_DIR)/.../*.o and $(BUILD_DIR)/.../*.d...
	@$(RM) $(OBJ) $(DEP)

# Remove the libraries, object, and dependency files #
distclean: clean
	@echo Deleting $(BUILD_DIR)/*...
	@rm -rf $(BUILD_DIR)/*
	@echo Deleting $(LIB_DIR)/*...
	@$(RM) $(LIB_DIR)/*

# Install the libraries and include files into the correct location #
install:
	@echo Installing $(BIN).h in /usr/local/include...
	@cp $(SRC_DIR)/$(BIN).h /usr/local/include/$(BIN).h
	@echo Installing lib$(BIN).a in /usr/local/lib...
	@mv $(LIB_DIR)/lib$(BIN).a /usr/local/lib/lib$(BIN).a
	@echo Installing lib$(BIN).so in /usr/local/lib...
	@mv $(LIB_DIR)/lib$(BIN).so /usr/local/lib/lib$(BIN).so

# Create the static library #
liba: $(LIB_DIR)/lib$(BIN).a

# Create the shared library #
libso: $(LIB_DIR)/lib$(BIN).so

# Uninstall the libraries from /usr/local/lib #
uninstall:
	@echo Uninstalling $(BIN).h from /usr/local/include...
	@rm -rf /usr/local/include/$(BIN).h
	@echo Uninstalling lib$(BIN).a from /usr/local/lib...
	@rm -rf /usr/local/lib/lib$(BIN).a
	@echo Uninstalling lib$(BIN).so from /usr/local/lib...
	@rm -rf /usr/local/lib/lib$(BIN).so