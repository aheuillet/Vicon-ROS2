#####################
### RULES OPTIONS ###
#####################

CC=g++                              # compiler
FLAGS=-Wall -Wextra -std=c++14      # compiler flags
LDFLAGS=-lViconDataStreamSDK_CPP      			# linker flags
SHARED=src/							# shared sources directory
OUTPUT_OPTION = -o $@			    # output option

# Indicates to the Makefile where files can be found
vpath %.cpp src			       # .cpp files location
vpath %.hpp src			       # .hpp files location
vpath %.o build				           # .o files location

BIN = client

#TEST = 

COMPIL = $(addprefix bin/, $(BIN))

.PHONY: default all clean

#######################
### COMPILING RULES ###
#######################

default: $(COMPIL)

all: default

clean:
	rm -f bin/client build/*.o

###########################
### FILE SPECIFIC RULES ###
###########################

### OBJECT FILES ###

build/main.o: utils.hpp
build/utils.o: utils.hpp

### BINARIES ###

bin/client: build/main.o build/utils.o
	@echo "> Compiling $@"
	@$(CC) $(FLAGS) -I$(SHARED) $^ -o $@ $(LDFLAGS) $(OUTPUT_OPTION)

#####################
### GENERIC RULES ###
#####################

build/%.o: %.cpp
	@echo "> Compiling $@"
	@$(CC) $(FLAGS) -I$(SHARED) -c $< $(LDFLAGS) $(OUTPUT_OPTION)

bin/%: build/%.o
	@echo "> Compiling $@"
	@$(CC) $(FLAGS) -I$(SHARED) $^ -o $@ $(LDFLAGS) $(OUTPUT_OPTION)