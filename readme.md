made on windows, for VSCode



# get library

follow this tutorial :

> **NOTE** read the note about commands **BEFORE** running said command (that is to say don't forget to include visualisation module)
> https://pointclouds.org/documentation/tutorials/pcl_vcpkg_windows.html

Also you should probably add cmake to your path if not already done.
vcpkg will normaly download a suitable version at `O:\vcpkg\downloads\tools\cmake-3.30.1-windows\cmake-3.30.1-windows-i386\bin`


## fix

Your installation will probably be fucked up anyways for *reasons*.

At the very least I had to :
- reinstal lz4 `>./vcpkg.exe install lz4`
- in `[vcpkg Path]\installed\x64-windows\share\lz4\lz4Config.cmake` and comment lines : 29 and 31 to 33 (to force shared librairy use)

and maybe others random things I don't remebers. Please share your fix here.


## Compile

Run VScode task "Cmake Build" in cpp file.

Or manually run commands :

>cd build
>cmake .. -DCMAKE_TOOLCHAIN_FILE=O:/vcpkg/scripts/buildsystems/vcpkg.cmake
>cmake --build . --config Release


Program will be compiled in `/Release/` Folder
Try `.\Release\pcl_visualizer_demo.exe -l O:\pfe\pfepoc\example_pcd\five_people.pcd`



## visualisation

https://pcl.readthedocs.io/projects/tutorials/en/master/cloud_viewer.html#cloud-viewer
https://pcl.readthedocs.io/projects/tutorials/en/master/pcl_visualizer.html#pcl-visualizer

You can find example pcd files in pcl git repo `\pcl-master\test`

# other


different compiler should propably use ?
cmake .. -G "Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=O:/vcpkg/scripts/buildsystems/vcpkg.cmake