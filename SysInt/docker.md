# Containers
<a href="https://youtu.be/pTFZFxd4hOI?si=69S6WmAiZ4Y52x-h">Tutorial I am using.</a>
<a href="http://wiki.ros.org/docker/Tutorials">Ros docker tutorials</a>

Basically used to run an application on different machines that has several dependencies. 

A container has the app and all of its dependencies of the corresponding versions.

Makes app platform independent and can be run on any device running any version of the dependencies.

When compiled creates an image that can uploaded on the coloud like github and can then be used on any platform.

```
FROM node:alpine 
COPY . /app
WORKDIR /app
CMD node app.js
```

Import the alpine version of linux.

Copy all the files onto the app directory in the current directory

Change the Working directory to /app

CMD is used to execute a command: we are executing node app.js

`docker build -t hello-docker .` 	Used to build our package -t \<name\> is the tag used to identify the package and '.' signifies the current working directory.

`docker ps` lists all the running containers.

`docker run --it ubuntu` runs ubuntu container in interactive mode and we get into the shell as the root user.

Container is the running environment for an image

`docker run -p6000:6379` Run the image with tethering to port number 6000 of the host and the port number of 6379.


## Ros Docker
`docker pull ros:noetic-robot` to pull the noetic image of ros.

`$ docker run -it ros:noetic-robot` to run the image

Open a new tab and find the container id of the previous container using `docker ps -l`

`$ docker exec -it <ID> bash` to open a new terminal tab in the same container as the previous one.

`source ros_entrypoint.sh` to setup the environment.

Good to go!

## Ros containers
Do we need multiple containers?