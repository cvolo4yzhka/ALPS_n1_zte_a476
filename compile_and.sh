#!/bin/bash
export USE_CCACHE=1
ccache -M 50G
export ANDROID_JACK_VM_ARGS="-Dfile.encoding=UTF-8 -XX:+TieredCompilation -Xmx4G"
source build/envsetup.sh
lunch full_zte_blade_a476_n1-user
make -j16 2>&1 | tee out/build.log
