CXX      := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -I include -I third_party
TARGET   := analytical
SRCS     := src/main.cpp src/floorplanner.cpp src/tsv_placement.cpp src/routing_congestion.cpp src/parser.cpp src/legalize.cpp src/legalize_heu.cpp src/util.cpp
OBJS     := $(SRCS:.cpp=.o)

.PHONY: all clean run

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

src/floorplanner.o: src/floorplanner.cpp include/floorplanner.h third_party/stb_image_write.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

src/util.o: src/util.cpp include/util.h include/floorplanner.h third_party/stb_image_write.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

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
