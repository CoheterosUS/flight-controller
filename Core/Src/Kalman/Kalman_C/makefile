# Convenciones estándar para C
CC = gcc
CFLAGS = -Wall
SRCS = $(wildcard *.c)
TARGET = prueba.exe

# El objetivo tiene el mismo nombre que el archivo final
$(TARGET): 
	$(CC) $(CFLAGS) $(SRCS) -o $(TARGET)

.PHONY: run

run: $(TARGET)
	./$(TARGET)