docker run -it --rm -v "$(pwd)/src:/workspaces/src" --network bridge --device=/dev/usb_robot_arm --env-file ./.env ghcr.io/otischung/pros_ai_image:latest /bin/bash
