# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Iinc -Ilib/nlohmann

# Directories
SRCDIR = src
BINDIR = bin
OBJDIR = obj
LIBDIR = lib

# Source files
SRCS = $(wildcard $(SRCDIR)/*.cpp) main.cpp

# Object files
OBJS = $(patsubst %.cpp, $(OBJDIR)/%.o, $(notdir $(SRCS)))

# Target executable
TARGET = $(BINDIR)/3d_software.exe

# Linker flags
LDFLAGS = -lm -mconsole

# Default target
all: $(TARGET)

# Rule to create the binary
$(TARGET): $(OBJS)
	@if not exist $(BINDIR) mkdir $(BINDIR)
	$(CXX) $(OBJS) -o $@ $(LDFLAGS)

# Rule to create object files for source files in SRCDIR
$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	@if not exist $(OBJDIR) mkdir $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Rule to compile main.cpp (since it's not in SRCDIR)
$(OBJDIR)/%.o: %.cpp
	@if not exist $(OBJDIR) mkdir $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean up
clean:
	del /f $(OBJDIR)\*.o
	if exist $(BINDIR)\3d_software.exe del /f $(BINDIR)\3d_software.exe

# Phony targets
.PHONY: all clean
