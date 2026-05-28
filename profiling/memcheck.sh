cd build || exit 1
if make -j hai823i_nomrigide_debug; then
    cd .. || exit 1
    valgrind \
        --tool=memcheck \
        --leak-check=full \
        --show-leak-kinds=all \
        --track-origins=yes \
        --error-exitcode=1 \
        --log-file=valgrind_memcheck.log \
        ./build/hai823i_nomrigide_debug
else
    cd .. || exit 1
    exit 1
fi