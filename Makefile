### CUSTOMISABLE VARIABLES

# Username used to prefix Docker images with.
WHOAMI ?= ${USER}

# Location of Docker binary
DOCKER ?= docker

# What is a suitable name for the current git branch? Use it as the Docker tag.
# If this is not a git repo, or there is no HEAD ref, use "latest" as a
# fallback.
TAG ?= $(shell git name-rev HEAD --name-only 2>/dev/null || echo "latest")

### DERIVED VARIABLES

# Name of image to build
DEV_IMAGE_NAME = $(WHOAMI)/robotic-surgery:$(TAG)

# These options perform some basic magic to allow X11 programs to
# tunnel through to the host.
COMMON_RUN_OPTS = -ti -u ros -w /home/ros/workspace -e HOME=/home/ros \
		  -e QT_X11_NO_MITSHM=1 -e DISPLAY=${DISPLAY} \
		  -v /tmp/.X11-unix:/tmp/.X11-unix \
		  "$(DEV_IMAGE_NAME)"

# Command used to launch a login shell into the image
LOGIN_RUN_OPTS = $(COMMON_RUN_OPTS) /bin/bash -l

all: image

# Build a Docker image for this repo
image:
	"$(DOCKER)" build -t "$(DEV_IMAGE_NAME)" .

# Launch a shell in the Docker image *after* building
login: image
	$(DOCKER) run --rm $(LOGIN_RUN_OPTS)

# Launch a shell in the Docker image *without* building. Usually you don't want
# to do this.
no-build-login:
	$(DOCKER) run --rm $(LOGIN_RUN_OPTS)

.PHONY: all build login image no-build-login

.DEFAULT: all
