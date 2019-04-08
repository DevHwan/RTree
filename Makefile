all: build

build: Test.cpp TestBadData.cpp MemoryTest.cpp
	mkdir -p bin
	g++ -o bin/Test -std=c++11 -I. Test.cpp
	g++ -o bin/TestBadData -std=c++11 -I. TestBadData.cpp
	g++ -o bin/MemoryTest -std=c++11 -I. MemoryTest.cpp

test: build
	./bin/Test
	./bin/TestBadData baddata.txt
	./bin/MemoryTest

clean:
	rm -rf bin
