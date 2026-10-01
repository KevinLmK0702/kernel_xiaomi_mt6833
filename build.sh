#!/bin/bash
# Adapted from README.md by DeepSeek

# Color definition
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

FLGS="--target=aarch64-linux-gnu \
      -Wno-implicit-function-declaration \
      -Wno-unused-but-set-variable \
      -Wno-unused-variable \
      -Wno-unused-function \
      -Wno-unused-label \
      -Wno-incompatible-pointer-types"

print_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
print_warning() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
print_error() { echo -e "${RED}[ERROR]${NC} $1"; }

print_info "设置编译环境..."

TC_DIR="$HOME/toolchains"
CLANG_DIR="$TC_DIR/Clang-16.0/bin/"
CLANG_LIB="$TC_DIR/Clang-16.0/lib/"

DATE_STR=$(date +'%y%m%d')
KER_STR="Evergo and Everpal Kernel, patched ReSukiSU v4.2.0 and SUSFS v2.3.0, compiled on $DATE_STR"

export PATH="$CLANG_DIR:$PATH"
export LD_LIBRARY_PATH="$CLANG_LIB":$LD_LIBRARY_PATH
export ARCH=arm64
export SUBARCH=arm
export CLANG_TRIPLE=aarch64-linux-gnu-
export LD=ld.lld
export O=out


rm -rf out/ && mkdir out
rm -rf build_kernel.log

print_info "更新 ReSukiSU..."

curl -LSs "https://raw.githubusercontent.com/ReSukiSU/ReSukiSU/main/kernel/setup.sh" | bash 

print_info "配置内核..."

make -j$(nproc --all) O=out \
          CC="clang" \
          LD=ld.lld \
          CLANG_TRIPLE=aarch64-linux-gnu- \
          CROSS_COMPILE=aarch64-linux-gnu- \
          CROSS_COMPILE_COMPAT=aarch64-linux-gnueabi- \
          CROSS_COMPILE_ARM32=arm-linux-gnueabi- \
          LLVM_IAS=1 \
          KCFLAGS="$FLGS" \
          evergo_defconfig

print_info "开始编译内核..."

make -j$(nproc --all) O=out \
          CC="clang" \
          LD=ld.lld \
          CLANG_TRIPLE=aarch64-linux-gnu- \
          CROSS_COMPILE=aarch64-linux-gnu- \
          CROSS_COMPILE_COMPAT=aarch64-linux-gnueabi- \
          CROSS_COMPILE_ARM32=arm-linux-gnueabi- \
          LLVM_IAS=1 \
          KCFLAGS="$FLGS" \
          Image.gz dtbs 2>&1 | tee -a build_kernel.log

if [ -f "out/arch/arm64/boot/Image.gz" ]; then
    print_info "内核编译成功!"
    ls -lh "out/arch/arm64/boot/Image.gz"
    file "out/arch/arm64/boot/Image.gz"

    ZIP_KP="Mix-Kernel-ReSukiSU-v4.2.0-SUSFS-v2.3.0-KPN-EXP-$DATE_STR.zip"
    ZIP_VANILLA="Mix-Kernel-ReSukiSU-v4.2.0-SUSFS-v2.3.0-Vanilla-$DATE_STR.zip"

    rm -f "$ZIP_KP" "$ZIP_VANILLA"

    print_info "修补 KPatch-Next-EXP..."

    wget -O kpimg https://github.com/741afb7/KPatch-Next-EXP/releases/download/0.13.14/kpimg-linux 
    wget -O kptools https://github.com/741afb7/KPatch-Next-EXP/releases/download/0.13.14/kptools-linux
    chmod +x kpimg kptools

    cp out/arch/arm64/boot/Image Image.vanilla
    ./kptools -p -i out/arch/arm64/boot/Image -k kpimg -o Image.patched 

    print_info "为修补了 KPatch-Next-EXP 内核打包 AnyKernel3 刷机包..."
    rm -rf AnyKernel3
    git clone --depth=1 https://github.com/osm0sis/AnyKernel3.git AnyKernel3 
    cp Image.patched AnyKernel3/Image 
    cmp -s AnyKernel3/Image Image.patched

    cd AnyKernel3

    sed -i 's/device\.name1=.*/device.name1=evergo/' anykernel.sh
    sed -i 's/device\.name2=.*/device.name2=everpal/' anykernel.sh
    sed -i '/device\.name[3-5]=.*/d' anykernel.sh

    sed -i 's/BLOCK=.*/BLOCK=boot/' anykernel.sh
    sed -i 's/IS_SLOT_DEVICE=.*/IS_SLOT_DEVICE=auto/' anykernel.sh

    sed -i "s/kernel\.string=.*/kernel.string=$KER_STR/" anykernel.sh

    zip -r "../$ZIP_KP" * 

    cd ..  

    print_info "为没有修补的内核打包 AnyKernel3 刷机包..."
    rm -rf AnyKernel3
    git clone --depth=1 https://github.com/osm0sis/AnyKernel3.git AnyKernel3 
    cp Image.vanilla AnyKernel3/Image 

    cd AnyKernel3

    sed -i 's/device\.name1=.*/device.name1=evergo/' anykernel.sh
    sed -i 's/device\.name2=.*/device.name2=everpal/' anykernel.sh
    sed -i '/device\.name[3-5]=.*/d' anykernel.sh

    sed -i 's/BLOCK=.*/BLOCK=boot/' anykernel.sh
    sed -i 's/IS_SLOT_DEVICE=.*/IS_SLOT_DEVICE=auto/' anykernel.sh

    sed -i "s/kernel\.string=.*/kernel.string=$KER_STR/" anykernel.sh

    zip -r "../$ZIP_VANILLA" * 

    cd ..  
    
    ls -la Mix-Kernel-ReSukiSU-v4.2.0-SUSFS-v2.3.0-*.zip
else
    print_error "内核编译失败!"
    exit 1
fi