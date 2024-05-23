# pros_crane

## Description
This repository is based on the image 'ghcr.io/otischung/pros_ai_image' and is used to build the image for the crane project.

## Usage

### Docker run command on windows or mac
```bash
docker run -it --rm -v "$(pwd)/src:/workspaces/src"  --env-file ./.env ghcr.io/otischung/pros_ai_image:latest /bin/bash
```

### Docker run command on raspberry pi
```bash
docker run -it --rm -v "$(pwd)/src:/workspaces/src" --network bridge --device=/dev/usb_robot_arm --env-file ./.env ghcr.io/otischung/pros_ai_image:latest /bin/bash
```
or 
```bash
./crane4282_control.sh
``` 
### docker build
#### build docker image
```bash
docker build -t pros_crane . --no-cache
```

#### run docker container
```bash
docker run -it --rm pros_crane /bin/bash
```
#### push docker image
```bash
.scripts/local_build.sh
```

## Resources
- [ghcr.io/otischung/pros_ai_image](https://github.com/otischung/pros_AI_image)