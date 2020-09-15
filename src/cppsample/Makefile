#
# Sample C program for Sensapex micromanipulator SDK
# (c) Sensapex 2013
#
# Makefile
#

CC = g++

CPP_FLAGS += -W -Wall -fPIC -I../lib

ifeq ($(debug),1)
	PREFIX=/local
	CPP_FLAGS += -g  -D_DEBUG 
	LDFLAGS   += -L/usr/local/lib
else
	PREFIX=
	CPP_FLAGS += -O2 -DNDEBUG
endif

LIBS = -L../lib -lumanipulatorctl

SRCS = cppsample.cpp
HDRS = customumctl.h
OBJS = $(SRCS:.cpp=.o)

.cpp.o: $(HDRS)
	@$(CC) -c $(CPP_FLAGS) $< -o $@

TARGET = cppsample

# Targets
all: $(TARGET)

$(OBJS): $(SRC) $(HDRS)

install: $(TARGET)
	install -D -v -s $(TARGET) $(DESTDIR)/usr$(PREFIX)/bin/$(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) $(LIBS) -o $(TARGET)

clean:
	rm -f $(TARGET) $(OBJS)

