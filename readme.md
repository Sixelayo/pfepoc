made on windows, for VSCode

Expected time to set up : probably 2 hours

# get library

follow this tutorial :

> **NOTE** read the note about commands **BEFORE** running said command (that is to say don't forget to include visualisation module)
> https://pointclouds.org/documentation/tutorials/pcl_vcpkg_windows.html

Installing pcl and all its dependency with vcpk may take an hour or so depening on your config

Also you should probably add cmake to your path if not already done.
vcpkg will normaly download a suitable version at `...\vcpkg\downloads\tools\cmake-3.30.1-windows\cmake-3.30.1-windows-i386\bin`


## fix

It's unfortunately possible that your installation will be messed up anyways for *reasons*.

At the very least I had to :
- reinstal lz4 `>./vcpkg.exe install lz4`
- in `[vcpkg Path]\installed\x64-windows\share\lz4\lz4Config.cmake` and comment lines : 29 and 31 to 33 (to force shared librairy use)

and maybe others random things I don't remebers. Please share your fix here.


## Compile

Run VScode task "CMake Configure" then "Cmake Build" in cpp file.

Or manually run commands :

>cd build
>cmake .. -DCMAKE_TOOLCHAIN_FILE=O:/vcpkg/scripts/buildsystems/vcpkg.cmake
>cmake --build . --config Release


Program will be compiled in `visualisation/build/Release/` Folder

Checking that it works :

(in `/build/`) Try 

Viewing :
```cmd
.\Release\pcl_visualizer_demo.exe -view -file <any path>
```

Ressampling :
```cmd
.\Release\pcl_visualizer_demo.exe -foo
.\Release\pcl_visualizer_demo.exe -foo
```

Comparing :
```cmd
.\Release\pcl_visualizer_demo.exe -foo
.\Release\pcl_visualizer_demo.exe -foo
```



## visualisation

https://pcl.readthedocs.io/projects/tutorials/en/master/cloud_viewer.html#cloud-viewer
https://pcl.readthedocs.io/projects/tutorials/en/master/pcl_visualizer.html#pcl-visualizer

You can find example pcd files in pcl git repo `\pcl-master\test`

> The fivepeoiple.pcd seem irrelevant ? I think that it has many points with the exact same coordinate (why?) so I'll use cturtle for testing. Also for reasons I cannot explain, cturtle is WAAAAY faster to process with mdna than five_people which is sus to say the least => pretty sure five_people.pcd is wrong as viewed in blender

## generating sample data

Install this add-on
> https://github.com/MarkHedleyJones/blender-pcd-io

Node set up ... lorem impsum varying density ...

# other


different compiler should propably use ?
cmake .. -G "Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=O:/vcpkg/scripts/buildsystems/vcpkg.cmake

