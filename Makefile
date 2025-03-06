# Compiler settings
CC=gcc
CFLAGS=-Wall -Wextra -pthread
LDFLAGS=-lm -pthread

# Project files
TARGET=apollo11
SRC=AGC.c
OBJ=$(SRC:.c=.o)

# Default target
all: $(TARGET)

# Linking
$(TARGET): $(OBJ)
	$(CC) $(OBJ) -o $(TARGET) $(LDFLAGS)

# Compilation
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Clean build files
clean:
	@echo "Realizando a limpeza dos arquivos de compilação"
	rm -f $(OBJ) $(TARGET)

# Run the simulator
run: $(TARGET)
	./$(TARGET)

help:
	@echo "Usage: make [all|clean|run]"
	@echo "all: Build the simulator"
	@echo "clean: Remove build files"
	@echo "run: Run the simulator"

.PHONY: all clean run