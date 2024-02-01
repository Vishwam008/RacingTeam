## Headers not found
* ros2 does not detect 'Header' datatype directly in msg files. It will ignore entire msg files due to that. Add `std_msgs/` before 'Header'.

## Type error
* Location of parameters in declaration of subscribers and publishers is inverse to that of ros2. `self.create_publisher(datatype, topic, queue_size)`

## setuptools Deprecation warning
* `RuntimeError: 'distutils.core.setup()' was never called -- perhaps 'setup.py' is not a Distutils setup script`

* Use setuptools version `58.2.0` and not the latest one. `pip install setuptools==58.2.0`

## Launch files executable not found
* `launch.substitutions.substitution_failure.SubstitutionFailure: executable 'sub.py' not found on the libexec directory '/home/vishwam/colcon_ws/install/hello_new/lib/hello_new'`
* Do not use .py in the executable. Just use the name of the file.
