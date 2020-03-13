#!/bin/bash
export USE_CCACHE=1
export CCACHE_DIR=/home/cvolo4yzhka/source/.ccache
ccache -M 50G
export ANDROID_JACK_VM_ARGS="-Dfile.encoding=UTF-8 -XX:+TieredCompilation -Xmx4G"
source build/envsetup.sh
lunch full_zte_blade_a476_n1_x32-user
make bootimage -j16 2>&1 | tee out/build32.log
