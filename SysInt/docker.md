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

`docker container stop CONTAINER_ID` to stop a container.


## Ros Docker
`docker pull ros:noetic-robot` to pull the noetic image of ros.

`$ docker run -it ros:noetic-robot` to run the image

Open a new tab and find the container id of the previous container using `docker ps -l`

`$ docker exec -it <ID> bash` to open a new terminal tab in the same container as the previous one.

`source ros_entrypoint.sh` to setup the environment.

Good to go!

## Ros containers
Do we need multiple containers?

## Using docker hub
Login into docker hub by using: `sudo docker login`
<ol>
	<li>Create an account</li>
	<li>Create a repo</li>
	<li>Any changes made to a container can be pushed online to an image by: `sudo docker commit CONTAINER_ID USRNM/REPO:TAG_NAME`</li>
	<li>Run the same image using `sudo docker run -it USRNM/REPO:TAG_NAME`</li>
</ol>

To create a new repo and commit an image to it:
```
docker tag local-image:tagname new-repo:tagname
docker push new-repo:tagname
```

## To do:
Explore docker with vs code.

## docker images
`docker image rm -f IMG_NAME` removes the image with tag IMG_NAME and f: forcefully
`docker run -it` i:interactive t:tty:terminal to type in


## docker container
`docker container ls` list all the running containers

`docker container ls -a` list even stopped containers

`docker container start CNTR_ID` start the container having the given id

`docker container rm`

`docker container prune` remove all non running containers

`docker run --rm --name CNTR_NM IMG_NM` rm: remove automatically after the container is stopped name: give the container a name

`docker exec -it CNTR_NM bash (or CMND)` open a new terminal in  a running container or execute the given command

Connecting volumes of container to host: `docker run -it -v ABS_PATH_HOST:ABS_PATH_CONT`

`docker commit c3f279d17e0a  svendowideit/testimage:version3` to commit a docker container to an image


## A custom image
Create a folder for the image and create a file called Dockerfile. In that file:

`FROM <starting image>`

`RUN apt-get update && apt-get install -y nano && rm -rf /var/lib/apt/lists/*` installs nano 

`COPY dir1/ /dir2/` copies all contents of dir1 in the current folder into dir2 of root in the image

`docker build -t NAME .` 	Used to build our image. NAME is the tag used to identify the IMAGE and '.' signifies the current working directory.

RUN is used when building the image.

CMD is used after starting the container.

## Conda
`conda config --set auto_activate_base false` to stop conda from running automatically on startup.


