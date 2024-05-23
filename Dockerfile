FROM ghcr.io/otischung/pros_ai_image:1.3.0

RUN rm -rf /workspaces/src/*
COPY ./src /workspaces/src

WORKDIR /workspaces

CMD ["bash", "-l"]