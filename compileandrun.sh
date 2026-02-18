cd build
if make -j; then
    cd ..
    ./build/hai823i_nomrigide
else
    cd ..
fi