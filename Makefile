params := $(wordlist 2,100,$(MAKECMDGOALS))

build:
	g++ main.cpp map.cpp agent.cpp ctNode.cpp ctSolution.cpp search.cpp -o multirobot -std=c++0x

run: build
	./multirobot $(params)

simple-run: build
	python3 call-multirobot.py $(params)
