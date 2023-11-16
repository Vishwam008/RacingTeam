# CarMaker 

* To edit parameters:
    * Main GUI --> Extras --> CMRosIF --> Edit Parameters
    * PROJECT/Data/Config/CMRosIFParameters 
        
* While building an error may be encountered:

    ```

    Executing: 'make -C src cm-ros1 --no-print-directory'

    LD     CarMaker.linux64
    /usr/bin/ld:./../lib//libcmcppifloader-linux64.so: file format not recognized; treating as linker script
    /usr/bin/ld:./../lib//libcmcppifloader-linux64.so:0: syntax error
    collect2: error: ld returned 1 exit status
    make: *** [Makefile:67: CarMaker.linux64] Error 1
    ```
    actually `libcmcppifloader-linux64.so` is supposed to be a symbolic link but it gets broken when the release is passed around. Go to the directory PROJECT/lib/ and run the command `sudo ln -snf libcmcppifloader-linux64.so.1.0.0 libcmcppifloader-linux64.so` to recreate the link.

* When you build if you get `MK ros_ws: Permission denied`, modify the permissions of build.sh in ros1/ros_ws and set it to executable. 