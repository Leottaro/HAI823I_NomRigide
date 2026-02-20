#!/usr/bin/env bash

mode=${1:-debug}

case "$mode" in
    debug)
        target="hai823i_nomrigide_debug"
        ;;
    opt)
        target="hai823i_nomrigide_opt"
        ;;
    *)
        echo "Usage: $0 [debug|opt]"
        exit 1
        ;;
esac

echo "Compiling $mode executable..."

cd build || exit 1
if make -j "$target"; then
    cd .. || exit 1
    ./build/"$target"
else
    cd .. || exit 1
    exit 1
fi