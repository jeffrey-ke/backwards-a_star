#/usr/bin/env bash
if [[ $1 == remake ]]; then
	cmake .
fi
cd build
make
if [[ $? -eq 0 ]]; then
	lldb ./run_test
fi
