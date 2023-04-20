#!/bin/bash

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

thread_num=$(expr $(free -m | grep "Mem" | awk '{print $7}') / 4096)
thread_num=$(($thread_num > 0 ? $thread_num : 1))
echo "Build with $thread_num threads"

cpp_ws_dir=$script_dir/
cpp_ws_dir=$(realpath $cpp_ws_dir)
build_dir=$cpp_ws_dir/build

# 1. 如果build目录不存在，则创建
if [ ! -d "$build_dir" ]; then
    mkdir -p $build_dir
fi
cd $build_dir

# 2. 清理旧的可执行文件，保证运行的是最新的可执行文件
echo "pwd in script file: $(pwd)"
executable_file=$build_dir/main
rm -rf $executable_file

# 3. 编译，生成可执行文件
cmake \
    -D OpenCV_DIR=$opencv_dir \
    $cpp_ws_dir

make -j${thread_num}

# 5. 运行可执行文件
echo "Runing $executable_file"
$executable_file
