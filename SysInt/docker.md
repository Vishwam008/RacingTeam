# Containers
<a href="https://youtu.be/pTFZFxd4hOI?si=69S6WmAiZ4Y52x-h">Tutorial I am using.</a>

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
