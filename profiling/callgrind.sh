cd build || exit 1
if make -j hai823i_nomrigide_opt; then
    cd .. || exit 1
    valgrind \
        --tool=callgrind \
        --callgrind-out-file=valgrind_callgrind.out \
        --collect-jumps=yes \
        --simulate-cache=yes \
        ./build/hai823i_nomrigide_opt
    kcachegrind valgrind_callgrind.out
else
    cd .. || exit 1
    exit 1
fi