made on windows, for VSCode

Expected time to set up : probably 2 hours

# get library

follow this tutorial :

> **NOTE** read the note about commands **BEFORE** running said command (that is to say don't forget to include visualisation module)
> https://pointclouds.org/documentation/tutorials/pcl_vcpkg_windows.html

Installing pcl and all its dependency with vcpk may take an hour or so depening on your config

TLDR :
- install vcpkg
- run vcpkg install script in powershell
- `./vcpkg.exe install pcl[visualisation]` (not sure this was this ?)

Also you should probably add cmake to your path if not already done.
vcpkg will normaly download a suitable version at `...\vcpkg\downloads\tools\cmake-3.30.1-windows\cmake-3.30.1-windows-i386\bin`

You probably need Visual studio btw

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

(in `/visualisation/`) Try 

Viewing :
```cmd
.\build\Release\pcl_visualizer_demo.exe -view -file ..\example_pcd\five_people.pcd
```

Ressampling example :
```cmd
.\build\Release\pcl_visualizer_demo.exe -sample -mode rand 0.5 -file ..\example_pcd\foo1.pcd -o ..\example_pcd\fooBin.pcd -prev
.\build\Release\pcl_visualizer_demo.exe -view -file ..\example_pcd\fooBin.pcd
.\build\Release\pcl_visualizer_demo.exe -sample -mode mdwo 1 2 -file O:\pfe\example_pcd\kokurav2.pcd -save ..\example_pcd\sampled_kokurav2.pcd -prev -binary
```

Comparing :
```cmd
.\build\Release\pcl_visualizer_demo.exe -sample -mode mdwo 0.03 0.09 -file ..\example_pcd\foo1.pcd -save ..\example_pcd\samp1.pcd -prev -binary
 .\build\Release\pcl_visualizer_demo.exe -sample -mode mdwo 0.03 0.09 -file ..\example_pcd\foo2.pcd -save ..\example_pcd\samp2.pcd -prev -binary
.\build\Release\pcl_visualizer_demo.exe -compare -file_src ..\example_pcd\samp1.pcd -file_comp ..\example_pcd\samp2.pcd -save ..\example_pcd\comp.pcd -binary -prev -octree_res 0.09 -threshold 0.04 -all
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


# final

## getting pcd format

1. Converting XYZ files to PCD (took 10min per cloud on my computer)
```
.\build\Release\pcl_visualizer_demo.exe -convert_XYZ_to_PCD -file_xyz ..\example_pcd\final\premier_nuage.xyz -file_pcd ..\example_pcd\final\nuage1_raw.pcd
```
```
.\build\Release\pcl_visualizer_demo.exe -convert_XYZ_to_PCD -file_xyz ..\example_pcd\final\second_nuage.xyz -file_pcd ..\example_pcd\final\nuage2_raw.pcd
```

## first test low rez

2. random downsampling for the moment (~20s per cloud)
```
.\build\Release\pcl_visualizer_demo.exe -sample -mode rand 0.1 -file ..\example_pcd\final\nuage1_raw.pcd -save ..\example_pcd\final\nuage1_10percent.pcd -binary -prev
```
```
.\build\Release\pcl_visualizer_demo.exe -sample -mode rand 0.1 -file ..\example_pcd\final\nuage2_raw.pcd -save ..\example_pcd\final\nuage2_10percent.pcd -binary -prev
```

3. Poisson disk downsampling (~1min per cloud)
```
.\build\Release\pcl_visualizer_demo.exe -sample -mode mdwo 0.01 0.03 -file ..\example_pcd\final\nuage1_10percent.pcd -prev -save ..\example_pcd\final\nuage1_10percent_mdwo01_03.pcd -binary
```
```
.\build\Release\pcl_visualizer_demo.exe -sample -mode mdwo 0.01 0.03 -file ..\example_pcd\final\nuage2_10percent.pcd -prev -save ..\example_pcd\final\nuage2_10percent_mdwo01_03.pcd -binary
```

4. comparison
```
.\build\Release\pcl_visualizer_demo.exe -compare -file_src ..\example_pcd\final\nuage1_10percent_mdwo01_03.pcd -file_comp ..\example_pcd\final\nuage2_10percent_mdwo01_03.pcd -octree_res 0.06 -threshold 0.015 -all -prev
```

## high rez


2. Poisson disk downsampling (~10min per cloud)
```
.\build\Release\pcl_visualizer_demo.exe -sample -mode mdwo 0.004 0.008 -file ..\example_pcd\final\nuage1_raw.pcd -prev -save ..\example_pcd\final\nuage1_raw__mdwo004_008.pcd -binary
```
```
.\build\Release\pcl_visualizer_demo.exe -sample -mode mdwo 0.004 0.008 -file ..\example_pcd\final\nuage2_raw.pcd -prev -save ..\example_pcd\final\nuage2_raw__mdwo004_008.pcd -binary
```

3. comparison (~3min)

```
.\build\Release\pcl_visualizer_demo.exe -compare -file_src ..\example_pcd\final\nuage1_raw__mdwo004_008.pcd -file_comp ..\example_pcd\final\nuage2_raw__mdwo004_008.pcd -octree_res 0.008 -threshold 0.005 -all -prev -save ..\example_pcd\final\comp_raw_t007.pcd -binary
```

## very high rez

compare raw cloud directly w/o any sampling

