#/usr/bin/env bash
if [[ $1 == remake ]]; then
	cmake .
fi
cd build
make
lldb ./run_test
