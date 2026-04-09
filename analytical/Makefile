CXX      := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -I include
TARGET   := analytical
SRCS     := src/main.cpp src/floorplanner.cpp src/parser.cpp src/legalize.cpp
OBJS     := $(SRCS:.cpp=.o)

.PHONY: all clean run

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

src/%.o: src/%.cpp include/floorplanner.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# 使用預設測試案例執行
run: $(TARGET)
	./$(TARGET) \
		../input_pa2/n100.block \
		../input_pa2/n100.nets \
		./output/n100_output.txt \
		../input_pa2/n100.constraint

clean:
	rm -f $(OBJS) $(TARGET) ./output/n100_output.txt
